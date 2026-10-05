"""Reconstruct directed selected BATMAN paths from asynchronous node reports.

These are routing-table estimates, not traces of individual video packets.
Never infer a reverse path by reversing the forward path.
"""

import math


def selected_path(latest, source, destination, now, timeout=12.0):
    result = {
        "source": source, "destination": destination, "path": [source],
        "steps": [], "status": "unavailable", "reason": "",
        "hops": None, "mode": None, "age_s": None,
        "evidence": "selected_originator_tables",
    }
    # Only self-reported hard-interface identities resolve originator MACs.
    # An ARP entry for bat0 may be a client MAC, not its originator MAC.
    owners = {}
    for name, info in latest.items():
        for mac in info["snap"].get("mesh_macs", []):
            owners.setdefault(mac.lower(), set()).add(name)

    def resolve(mac):
        names = owners.get((mac or "").lower(), set())
        return next(iter(names)) if len(names) == 1 else None

    ages = []
    for _ in range(len(latest) + 1):
        current = result["path"][-1]
        info = latest.get(current)
        if not info:
            result.update(status="partial", reason="missing_report:" + current)
            break
        age = max(0.0, now - info["recv"])
        ages.append(age)
        if age >= timeout:
            result.update(status="stale", reason="stale_report:" + current)
            break
        if current == destination:
            result.update(status="complete", reason="",
                          hops=len(result["steps"]),
                          mode="direct" if len(result["steps"]) == 1 else "relay")
            break
        candidates = []
        for link in info["snap"].get("links", []):
            if not link.get("nexthop"):
                continue
            # Legacy reports can supply an already-resolved peer name.
            peer = resolve(link.get("mac")) or link.get("peer")
            if peer == destination:
                candidates.append(link)
        if len(candidates) != 1:
            result.update(status="partial",
                          reason=("ambiguous_route:" if candidates else "missing_route:") + current)
            break
        route = candidates[0]
        seen = route.get("last_seen_s")
        if not isinstance(seen, (float, int)) or not math.isfinite(seen) or seen < 0:
            result.update(status="partial", reason="unknown_route_age:" + current)
            break
        # OGM age at collection plus collector age is conservative.
        route_age = age + seen
        ages.append(route_age)
        if route_age >= timeout:
            result.update(status="stale", reason="stale_route:" + current)
            break
        next_node = resolve(route["nexthop"])
        if not next_node:
            result.update(status="partial", reason="unknown_nexthop:" + route["nexthop"])
            break
        result["steps"].append({
            "from": current, "to": next_node,
            "destination_tq": route.get("tq"), "age_s": round(route_age, 2),
        })
        result["path"].append(next_node)
        if next_node in result["path"][:-1]:
            result.update(status="loop", reason="route_loop:" + next_node)
            break
    else:
        result.update(status="loop", reason="hop_limit")
    result["age_s"] = round(max(ages), 2) if ages else None
    return result


def video_routes(latest, now, timeout=12.0):
    return {
        "forward": selected_path(latest, "head", "base", now, timeout),
        "reverse": selected_path(latest, "base", "head", now, timeout),
        "note": "라우팅 표 기반 추정 · 개별 영상 패킷의 실측 경로가 아닙니다",
    }
