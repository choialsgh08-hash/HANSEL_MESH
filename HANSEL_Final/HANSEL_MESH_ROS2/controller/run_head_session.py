#!/usr/bin/env python3
"""PC entrypoint. Existing BASE keyboard + Head runtime, one terminal."""
from __future__ import annotations

import argparse
import datetime
import ipaddress
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[1]


def q(value):
    return shlex.quote(str(value))


def validate(c):
    for name in ("base", "head"):
        if not c[name] or c[name].startswith("-") or any(x.isspace() for x in c[name]):
            raise ValueError(f"Invalid SSH destination: {name}")
    for name in ("head_ip", "pc_ip"):
        ipaddress.ip_address(c[name])
    for name in ("ros_root", "radar_root", "nano_port", "radar_port"):
        if not c[name].startswith("/"):
            raise ValueError(f"Absolute path required: {name}")
    for name, limit in (("maximum_pwm_percent", 100), ("speed_scale", 1)):
        if not 0 <= float(c[name]) <= limit:
            raise ValueError(f"Invalid {name}")
    for name in ("mesh_domain", "local_domain"):
        if not 0 <= int(c[name]) <= 232:
            raise ValueError(f"Invalid {name}")
    if c["mesh_domain"] == c["local_domain"]:
        raise ValueError("Sensor domain must differ from mesh domain")
    for name in ("map_local_port", "video_port"):
        if not 1024 <= int(c[name]) <= 65534:
            raise ValueError(f"Invalid {name}")


def commands(c, remote_sdp):
    root, radar = q(c["ros_root"]), q(c["radar_root"])
    mesh = f"cd {root} && export ROS_DOMAIN_ID={int(c['mesh_domain'])} && unset ROS_LOCALHOST_ONLY && source scripts/ros_env.sh && "
    local = f"cd {root} && export HANSEL_LOCAL_DOMAIN={int(c['local_domain'])} && source scripts/ros_local_env.sh && "
    with_nodes = c.get("with_nodes", False)
    units = "['head','node1','node2']" if with_nodes else "['head']"
    steps = [
        ("head", "unit", mesh + f"HEAD_MAX_PWM={float(c['maximum_pwm_percent'])} bash robot/run_unit.sh head {q(c['nano_port'])}"),
        ("base", "operator", mesh + f"HEAD_ONLY={'false' if with_nodes else 'true'} USE_RQT=false bash controller/run_operator_stack.sh"),
        ("head", "inputs", mesh + f"export PYTHONPATH={root}/ros2_ws/src/hansel_localization${{PYTHONPATH:+:$PYTHONPATH}} && python3 -m hansel_localization.local_inputs"),
        ("head", "odometry", local + "python3 -m hansel_localization.head_odometry_node --ros-args "
         "--params-file ros2_ws/src/hansel_bringup/config/head_session_odometry.yaml "
         '-p local_input_socket:=/tmp/hansel-inputs-$(id -u).sock'),
        ("head", "map", local + f"python3 scripts/run_head_map.py --host {q(c['head_ip'])}"),
        ("head", "radar", f"cd {radar} && HANSEL_RADAR_PATH_MODE=responsive RADAR_BIND={q(c['head_ip'])} RADAR_SERIAL_PORT={q(c['radar_port'])} bash scripts/start_head_radar_mesh.sh"),
        ("head", "camera", f"cd {radar} && RTP_IMPL=ffmpeg CAMERA_SDP_FILE={q(remote_sdp)} bash scripts/start_camera_stream.sh {q(c['pc_ip'])} {int(c['video_port'])}"),
        ("base", "keyboard", mesh + f'ACTIVE_TARGETS="{units}" SPEED_SCALE={float(c["speed_scale"])} bash controller/run_keyboard_control.sh'),
    ]
    if with_nodes:
        for unit in ("node1", "node2"):
            steps.insert(1, (unit, "unit", mesh + f"UNIT_MAX_PWM={float(c['maximum_pwm_percent'])} bash robot/run_node.sh {unit}"))
    return steps


class Session:
    def __init__(self, config, logs):
        self.config = config
        self.logs = logs
        self.tmp = tempfile.TemporaryDirectory(prefix="hansel-")
        self.hosts = ["base", "head"] + (["node1", "node2"] if config.get("with_nodes") else [])
        self.sockets = {host: str(Path(self.tmp.name) / host) for host in self.hosts}
        self.masters = []
        self.children = []

    def ssh(self, host):
        return ["ssh", "-S", self.sockets[host], "-o", "BatchMode=yes", self.config[host]]

    def connect(self):
        for host in self.hosts:
            ssh_log = self.logs / f"ssh-{host}.log"
            cmd = ["ssh", "-M", "-N", "-f", "-S", self.sockets[host],
                   "-E", str(ssh_log),
                   "-o", "ControlPersist=no", "-o", "ServerAliveInterval=3",
                   "-o", "ServerAliveCountMax=3", "-o", "ConnectTimeout=10"]
            if host != "base":
                proxy = shlex.join(["ssh", "-S", self.sockets["base"], "-o", "BatchMode=yes",
                                    "-W", "%h:%p", self.config["base"]])
                cmd += ["-o", "ProxyCommand=" + proxy]
            print(f"Connecting {host} (SSH authentication stays in SSH)", flush=True)
            try:
                subprocess.run(cmd + [self.config[host]], check=True)
            except subprocess.CalledProcessError as exc:
                raise RuntimeError(
                    f"SSH connection failed: {host} ({self.config[host]}), exit={exc.returncode}; see {ssh_log}"
                ) from exc
            self.masters.append(host)

    def forward_map(self):
        # Old browser tabs poll continuously; do not forward during SSH login.
        subprocess.run(
            ["ssh", "-S", self.sockets["head"], "-O", "forward",
             "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes", "-L",
             f"127.0.0.1:{self.config['map_local_port']}:{self.config['head_ip']}:8082",
             self.config["head"]],
            check=True, timeout=10, capture_output=True, text=True,
        )

    def start(self, host, name, command):
        label = f"{host}-{name}"
        log = (self.logs / f"{label}.log").open("wb")
        helper = self.config["ros_root"] + "/scripts/run_managed.py"
        remote = shlex.join(["python3", helper, name, "--", "bash", "-lc", "set -e; " + command])
        proc = subprocess.Popen(self.ssh(host) + [remote], stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
        self.children.append((label, proc, log))
        print(f"Started {name}: {log.name}", flush=True)

    def read(self, host, command):
        return subprocess.check_output(self.ssh(host) + [command], timeout=8)

    def close(self):
        for name, proc, log in reversed(self.children):
            if proc.stdin:
                try:
                    proc.stdin.close()  # SSH EOF makes remote owner stop its group.
                except OSError:
                    pass
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            log.close()
        for host in reversed(self.masters):
            try:
                subprocess.run(["ssh", "-S", self.sockets[host], "-O", "exit", self.config[host]],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.tmp.cleanup()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT / "configs/head_session.json")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--no-browser", action="store_true")
    p.add_argument("--with-nodes", action="store_true", help="Also start node1/node2 drive; default is Head only")
    args = p.parse_args(argv)
    c = json.loads(args.config.read_text())
    c["with_nodes"] = args.with_nodes
    if args.with_nodes:
        for host in ("node1", "node2"):
            if not c.get(host) or c[host].startswith("-") or any(x.isspace() for x in c[host]):
                raise ValueError(f"Set {host} SSH destination in session config")
    validate(c)
    run_id = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}"
    remote_sdp = f"/tmp/hansel-camera-{run_id}.sdp"
    steps = commands(c, remote_sdp)
    if args.dry_run:
        for host, name, command in steps:
            print(f"[{host}/{name}] {command}")
        return 0
    logs = ROOT / "runtime/sessions" / run_id
    logs.mkdir(parents=True)
    print(f"Session logs: {logs}", flush=True)
    session = Session(c, logs)
    player = None
    try:
        session.connect()
        # Inspect clock skew; never step system time during a running mission.
        offsets = {}
        for host in session.hosts:
            before = time.time()
            remote = float(session.read(host, "date +%s.%N"))
            offsets[host] = remote - (before + time.time()) / 2
        if any(abs(offsets["base"] - v) > 0.3 for v in offsets.values()):
            print("WARNING: Unit clocks differ. Check timecheck/chrony before driving.", flush=True)
        for host, name, command in steps[:-1]:
            if name == "camera" and args.no_camera:
                continue
            session.start(host, name, command)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if not args.no_camera and player is None and shutil.which("ffplay"):
                try:
                    data = session.read("head", "cat " + q(remote_sdp) + " 2>/dev/null")
                    if b"a=rtpmap:" in data and b"m=video" in data:
                        sdp = logs / "camera.sdp"
                        sdp.write_bytes(data)
                        video_log = (logs / "ffplay.log").open("wb")
                        player = subprocess.Popen(["ffplay", "-protocol_whitelist", "file,udp,rtp", "-fflags", "nobuffer",
                                                   "-flags", "low_delay", "-framedrop", str(sdp)], stdout=video_log, stderr=video_log)
                        video_log.close()
                except (subprocess.SubprocessError, OSError):
                    pass
            time.sleep(0.5)
        for name, proc, _ in session.children:
            print(f"{name}: {'running (check data health in map)' if proc.poll() is None else 'FAILED: see ' + str(logs / (name + '.log'))}")
        if not args.no_camera and player is None:
            print("Camera receiver not ready: see head-camera.log; no stale SDP was reused.")
        failed_drive = any(proc.poll() is not None for name, proc, _ in session.children if name.endswith(("-unit", "-operator")))
        if failed_drive:
            raise RuntimeError("Drive/operator did not start; inspect logs (possibly an existing manual session).")
        try:
            session.forward_map()
        except (OSError, subprocess.SubprocessError) as exc:
            detail = getattr(exc, "stderr", None) or str(exc)
            print(f"Map tunnel not ready: {detail.strip()}; see {logs / 'ssh-head.log'}. Drive session continues.", flush=True)
        else:
            if not args.no_browser:
                webbrowser.open(f"http://127.0.0.1:{c['map_local_port']}/")
        print(f"PWM limit={c['maximum_pwm_percent']}%, target scale={c['speed_scale']}. X stop, G enable, P exit all. Logs: {logs}", flush=True)
        host, _, command = steps[-1]
        # SSH allocates a remote PTY; the team's keyboard remains authoritative.
        keyboard = subprocess.Popen(session.ssh(host)[:-1] + ["-tt", c[host], "bash -lc " + q(command)])
        try:
            keyboard.wait()
        finally:
            if keyboard.poll() is None:
                keyboard.terminate()
                try:
                    keyboard.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    keyboard.kill()
                    keyboard.wait()
    except KeyboardInterrupt:
        print("Stopping owned session...")
    finally:
        if player is not None and player.poll() is None:
            player.terminate()
            try:
                player.wait(timeout=3)
            except subprocess.TimeoutExpired:
                player.kill()
                player.wait()
        session.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        sys.exit(str(exc))
