#!/usr/bin/env python3
"""Derive link-quality thresholds from measured logs (no eyeballing).

Two independent, pure analyses that turn recorded data into numbers:

  1. derive_signal_thresholds(records)
     Input : combined dashboard records (dashboard.py --log), each carrying
             net_rssi on the same timestamp as vid_err / vid_fps_ratio.
     Output: the RSSI where the video first degrades (signal_warn_dbm) and
             where it breaks (signal_danger_dbm) -> QualityConfig overrides.

  2. derive_reconverge_times(events)
     Input : timestamped reconnect events (link_down, neighbor_lost,
             route_changed, video_recovered), e.g. produced by a controlled
             link-break run plus metrics_agent / video_probe.
     Output: per-stage outage statistics that split the two stutter causes:
             reconverge (BATMAN path switch, cause A) vs video_recovery
             (keyframe wait, cause B).

Both functions are pure (log in -> dict out) so they are unit-tested with
fixtures today; the real logs are produced once the AR9271 dongle arrives.

Usage:
    # signal thresholds from a combined dashboard log
    python3 monitor/calibrate_thresholds.py --records combined.jsonl

    # reconnect timing from an events log
    python3 monitor/calibrate_thresholds.py --events reconnect_events.jsonl
"""

from __future__ import annotations  # 3.9 compatibility

import argparse
import json
import sys


# Video-degradation thresholds. Default to QualityConfig so the calibrator
# classifies "bad video" exactly the way the live judge does; fall back to
# literals if the controller package is unavailable.
try:
    from controller.quality_supervisor import QualityConfig as _QC

    _cfg = _QC()
    ERR_WARN = _cfg.err_warn_rate
    ERR_DANGER = _cfg.err_danger_rate
    FPS_WARN_RATIO = _cfg.fps_warn_ratio
    FPS_DANGER_RATIO = _cfg.fps_danger_ratio
except Exception:  # pragma: no cover - defensive fallback
    ERR_WARN, ERR_DANGER = 0.20, 1.00
    FPS_WARN_RATIO, FPS_DANGER_RATIO = 0.85, 0.60


def percentile(values: list, p: float):
    """Linear-interpolated percentile. Returns None for an empty list."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    k = (len(ordered) - 1) * (p / 100.0)
    low = int(k)
    high = min(low + 1, len(ordered) - 1)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (k - low)


def video_level(record: dict,
                err_warn: float = ERR_WARN, err_danger: float = ERR_DANGER,
                fps_warn: float = FPS_WARN_RATIO,
                fps_danger: float = FPS_DANGER_RATIO) -> str:
    """Classify one record's video as GOOD / WARN / DANGER (same rules as the judge)."""
    level = "GOOD"
    err = record.get("vid_err")
    if err is not None:
        if err > err_danger:
            return "DANGER"
        if err > err_warn:
            level = "WARN"
    ratio = record.get("vid_fps_ratio")
    if ratio is not None:
        if ratio < fps_danger:
            return "DANGER"
        if ratio < fps_warn and level == "GOOD":
            level = "WARN"
    return level


def derive_signal_thresholds(records: list, min_samples: int = 5,
                             margin_db: int = 1) -> dict:
    """Find the RSSI at which video degrades, from RSSI<->video correlation.

    Returns suggested signal_warn_dbm / signal_danger_dbm plus the sample
    counts behind them so a human can sanity-check before adopting them.
    """
    warn_rssis: list = []      # RSSI at samples where video is WARN or worse
    danger_rssis: list = []    # RSSI at samples where video is DANGER
    ok_count = 0
    used = 0
    for rec in records:
        rssi = rec.get("net_rssi")
        if rssi is None:
            continue
        used += 1
        level = video_level(rec)
        if level == "DANGER":
            danger_rssis.append(rssi)
            warn_rssis.append(rssi)
        elif level == "WARN":
            warn_rssis.append(rssi)
        else:
            ok_count += 1

    result: dict = {
        "samples_with_rssi": used,
        "video_ok": ok_count,
        "video_warn_or_worse": len(warn_rssis),
        "video_danger": len(danger_rssis),
        "signal_warn_dbm": None,
        "signal_danger_dbm": None,
        "notes": [],
    }

    # The strongest (least-negative) RSSI at which trouble still appears, nudged
    # up by a margin so the warning fires just before real degradation.
    if len(warn_rssis) >= min_samples:
        result["signal_warn_dbm"] = int(round(percentile(warn_rssis, 90))) + margin_db
    else:
        result["notes"].append(
            f"not enough WARN samples ({len(warn_rssis)}<{min_samples}); "
            "record more data at weaker signal"
        )
    if len(danger_rssis) >= min_samples:
        result["signal_danger_dbm"] = int(round(percentile(danger_rssis, 90))) + margin_db
    else:
        result["notes"].append(
            f"not enough DANGER samples ({len(danger_rssis)}<{min_samples})"
        )

    warn = result["signal_warn_dbm"]
    danger = result["signal_danger_dbm"]
    if warn is not None and danger is not None and not (warn > danger):
        # warn must trigger at a stronger (less negative) signal than danger.
        result["signal_warn_dbm"] = danger + 2
        result["notes"].append("adjusted warn above danger to keep ordering")
    return result


def group_cycles(events: list) -> list:
    """Split a flat event stream into reconnect cycles.

    A cycle opens on a link_down (forced, controlled break) OR on a
    neighbor_lost when none is open yet (natural reconnect, no forcing needed).
    This makes organically observed reconnects first-class: batman reroutes and
    reconnects on its own, so a real field log needs no artificial link_down.
    neighbor_lost is the reliable anchor since a gradual RF fade has no single
    "down" instant.
    """
    ordered = sorted(events, key=lambda e: e.get("ts", 0.0))
    cycles: list = []
    current = None
    for event in ordered:
        etype = event.get("type")
        ts = event.get("ts")
        if ts is None or etype is None:
            continue
        opens_cycle = etype == "link_down" or (
            etype == "neighbor_lost" and (current is None or "neighbor_lost" in current)
        )
        if opens_cycle:
            if current is not None:
                cycles.append(current)
            current = {etype: ts}
        elif current is not None:
            current.setdefault(etype, ts)  # keep the first of each stage
    if current is not None:
        cycles.append(current)
    return cycles


def _stage_stats(values: list) -> dict:
    return {
        "count": len(values),
        "p50": round(percentile(values, 50), 3) if values else None,
        "p95": round(percentile(values, 95), 3) if values else None,
        "max": round(max(values), 3) if values else None,
    }


def derive_reconverge_times(events: list) -> dict:
    """Measure per-stage outage durations across reconnect cycles.

    Works on natural reconnects (batman reroutes on its own) or forced ones.
    neighbor_lost is the anchor; link_down is optional and only present in a
    forced-break run.

    detect_delay : link_down    -> neighbor_lost   (forced runs only)
    reconverge   : neighbor_lost -> route_changed  (BATMAN path switch, cause A)
    video_recovery: route_changed -> video_recovered (keyframe wait, cause B)
    total_outage : neighbor_lost -> video_recovered (what the operator feels)
    """
    cycles = group_cycles(events)
    detect: list = []
    reconverge: list = []
    video_recovery: list = []
    total: list = []
    for cycle in cycles:
        ld = cycle.get("link_down")
        nl = cycle.get("neighbor_lost")
        rc = cycle.get("route_changed")
        vr = cycle.get("video_recovered")
        if ld is not None and nl is not None:
            detect.append(nl - ld)
        if nl is not None and rc is not None:
            reconverge.append(rc - nl)
        if rc is not None and vr is not None:
            video_recovery.append(vr - rc)
        start = ld if ld is not None else nl
        if start is not None and vr is not None:
            total.append(vr - start)
    return {
        "cycles": len(cycles),
        "detect_delay_s": _stage_stats(detect),
        "reconverge_s": _stage_stats(reconverge),
        "video_recovery_s": _stage_stats(video_recovery),
        "total_outage_s": _stage_stats(total),
    }


def read_jsonl(path: str) -> list:
    records = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


def _print_signal_report(result: dict) -> None:
    print("== Signal thresholds (from RSSI <-> video correlation) ==")
    print(f"samples with RSSI : {result['samples_with_rssi']}")
    print(f"video ok          : {result['video_ok']}")
    print(f"video warn+       : {result['video_warn_or_worse']}")
    print(f"video danger      : {result['video_danger']}")
    print(f"suggested signal_warn_dbm   = {result['signal_warn_dbm']}")
    print(f"suggested signal_danger_dbm = {result['signal_danger_dbm']}")
    for note in result["notes"]:
        print(f"  note: {note}")
    if result["signal_warn_dbm"] is not None:
        print("\nconfig lines (review before adopting):")
        print(f"SIGNAL_WARN_DBM={result['signal_warn_dbm']}")
        print(f"SIGNAL_DANGER_DBM={result['signal_danger_dbm']}")


def _print_reconverge_report(result: dict) -> None:
    print("== Reconnect timing (controlled link-break cycles) ==")
    print(f"cycles analyzed: {result['cycles']}")
    for stage in ("detect_delay_s", "reconverge_s",
                  "video_recovery_s", "total_outage_s"):
        s = result[stage]
        print(f"{stage:18s} count={s['count']:3d} "
              f"p50={s['p50']} p95={s['p95']} max={s['max']}")
    rc = result["reconverge_s"]["p50"]
    vr = result["video_recovery_s"]["p50"]
    if rc is not None and vr is not None:
        dominant = "keyframe wait (cause B)" if vr > rc else "BATMAN reconverge (cause A)"
        print(f"\ndominant stutter cause (median): {dominant}")


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Derive link thresholds / reconnect timing from logs."
    )
    p.add_argument("--records", default=None,
                   help="combined dashboard log (dashboard.py --log) for signal thresholds")
    p.add_argument("--events", default=None,
                   help="timestamped reconnect events log for reconnect timing")
    args = p.parse_args(argv)
    if not args.records and not args.events:
        p.error("pass --records and/or --events")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.records:
        result = derive_signal_thresholds(read_jsonl(args.records))
        _print_signal_report(result)
    if args.events:
        if args.records:
            print()
        result = derive_reconverge_times(read_jsonl(args.events))
        _print_reconverge_report(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
