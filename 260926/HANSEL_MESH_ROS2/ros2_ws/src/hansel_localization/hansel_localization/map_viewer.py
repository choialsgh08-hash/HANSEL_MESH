"""Read-only odometry web viewer; ROS imports are needed only in live mode."""

from __future__ import annotations

import argparse
import copy
import http.client
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


SCHEMA = "hansel.odometry-map.v1"
PATH_LIMIT = 1000
RELAY_LIMIT = 32
RADAR_RESPONSE_LIMIT = 8 * 1024 * 1024


class RadarProxy:
    """Forward only the existing radar viewer's four read-only resources."""

    routes = {
        "/radar/": ("/", "text/html; charset=utf-8"),
        "/radar_scene.js": ("/radar_scene.js", "text/javascript; charset=utf-8"),
        "/radar_panel.js": ("/radar_panel.js", "text/javascript; charset=utf-8"),
        "/api/radar": ("/api/radar", "application/json; charset=utf-8"),
    }

    def __init__(self, origin):
        parsed = urlsplit(origin)
        if (parsed.scheme not in ("http", "https") or not parsed.hostname
                or parsed.username is not None or parsed.password is not None
                or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
            raise ValueError("radar URL must be an http(s) origin without credentials, path or query")
        self.host = parsed.hostname
        self.port = parsed.port or (443 if parsed.scheme == "https" else 80)
        self.connection_type = http.client.HTTPSConnection if parsed.scheme == "https" else http.client.HTTPConnection

    def read(self, route, accept_gzip=False):
        path, content_type = self.routes[route]
        connection = self.connection_type(self.host, self.port, timeout=1.0)
        try:
            connection.request("GET", path, headers={"Accept-Encoding": "gzip" if accept_gzip else "identity"})
            response = connection.getresponse()
            if response.status != 200:
                raise OSError("radar resource unavailable")
            encoding = response.getheader("Content-Encoding", "identity").lower()
            if encoding not in ("identity", "gzip") or (encoding == "gzip" and not accept_gzip):
                raise OSError("unsupported radar response encoding")
            body = response.read(RADAR_RESPONSE_LIMIT + 1)
            if len(body) > RADAR_RESPONSE_LIMIT:
                raise OSError("radar response exceeds size limit")
            return body, content_type, encoding
        except http.client.HTTPException as exc:
            raise OSError("invalid radar HTTP response") from exc
        finally:
            connection.close()


def pose_xy_yaw(pose):
    x, y = float(pose.position.x), float(pose.position.y)
    q = pose.orientation
    values = [x, y, float(q.x), float(q.y), float(q.z), float(q.w)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite pose")
    if abs(x) > 1e7 or abs(y) > 1e7:
        raise ValueError("pose outside viewer bounds")
    norm = math.sqrt(sum(v * v for v in values[2:]))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("zero quaternion")
    qx, qy, qz, qw = [v / norm for v in values[2:]]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {"x": x, "y": y, "yaw": yaw}


class MapState:
    def __init__(self, frame="odom", mode="live", clock=time.monotonic):
        self.frame = frame
        self.mode = mode
        self.clock = clock
        self.lock = threading.RLock()
        self.pose = None
        self.pose_received = None
        self.path = []
        self.relays = []
        self.diagnostics = {}
        self.origin_revision = 0
        self.invalid_messages = 0

    def _frame(self, frame):
        if frame != self.frame:
            raise ValueError("frame mismatch")

    def receive(self, kind, msg):
        with self.lock:
            try:
                getattr(self, "_" + kind)(msg)
            except (ValueError, TypeError, AttributeError, OverflowError):
                self.invalid_messages += 1

    def _odometry(self, msg):
        self._frame(msg.header.frame_id)
        pose = pose_xy_yaw(msg.pose.pose)
        pose["stamp"] = {
            "sec": int(msg.header.stamp.sec),
            "nanosec": int(msg.header.stamp.nanosec),
        }
        self.pose = pose
        self.pose_received = self.clock()

    def _path(self, msg):
        self._frame(msg.header.frame_id)
        path = []
        for stamped in msg.poses[-PATH_LIMIT:]:
            self._frame(stamped.header.frame_id or msg.header.frame_id)
            point = pose_xy_yaw(stamped.pose)
            path.append({"x": point["x"], "y": point["y"]})
        if len(path) < len(self.path):
            # A restarted odometry process may publish a short new path without
            # having emitted the zero service's DELETEALL marker first.
            self.pose = None
            self.pose_received = None
            self.relays = []
            self.diagnostics.pop("odometry", None)
            self.origin_revision += 1
        self.path = path

    def _markers(self, msg):
        # The localization publisher sends full relay snapshots, plus DELETEALL
        # when its origin is reset. Do not replay retained RelayPose event history.
        if any(marker.action == 3 for marker in msg.markers):
            self.pose = None
            self.pose_received = None
            self.path = []
            self.relays = []
            self.diagnostics.pop("odometry", None)
            self.origin_revision += 1
        labels = {
            marker.id - 1000: marker.text[:100]
            for marker in msg.markers
            if marker.ns == "hansel_relay_labels" and marker.action == 0
        }
        relays = []
        for marker in msg.markers:
            if marker.ns != "hansel_relays" or marker.action != 0:
                continue
            self._frame(marker.header.frame_id)
            relays.append({
                **pose_xy_yaw(marker.pose),
                "id": str(marker.id),
                "label": labels.get(marker.id, "Relay " + str(marker.id)),
                "provisional": True,
            })
            if len(relays) == RELAY_LIMIT:
                break
        self.relays = relays

    def _diagnostics(self, msg):
        names = {
            "hansel/head/odometry": "odometry",
            "hansel/head/unit_controller": "unit",
        }
        for status in msg.status:
            if status.name in names:
                self.diagnostics[names[status.name]] = {
                    "level": int(status.level),
                    "message": status.message[:240],
                    "values": {v.key[:80]: v.value[:160] for v in status.values[:60]},
                    "received": self.clock(),
                }

    def snapshot(self):
        with self.lock:
            now = self.clock()
            diagnostics = copy.deepcopy(self.diagnostics)
            for item in diagnostics.values():
                item["age_s"] = max(0, now - item.pop("received"))
            return {
                "schema": SCHEMA,
                "mode": self.mode,
                "frame_id": self.frame,
                "generated_at": time.time(),
                "origin_revision": self.origin_revision,
                "pose": copy.deepcopy(self.pose),
                "pose_age_s": None if self.pose_received is None else max(0, now - self.pose_received),
                "path": copy.deepcopy(self.path),
                "path_limit": PATH_LIMIT,
                "relays": copy.deepcopy(self.relays),
                "diagnostics": diagnostics,
                "invalid_messages": self.invalid_messages,
                "notice": "Relative odometry only. Provisional distance calibration. Relay positions are estimates. Path is a bounded snapshot, not a full mission log.",
            }


def make_server(state, assets, host, port, radar_url="http://192.168.50.10:8081"):
    assets = Path(assets)
    radar = RadarProxy(radar_url)
    files = {
        "/": ("index.html", "text/html; charset=utf-8"),
        "/map.css": ("map.css", "text/css; charset=utf-8"),
        "/map.js": ("map.js", "text/javascript; charset=utf-8"),
        "/radar-embed.css": ("radar-embed.css", "text/css; charset=utf-8"),
        "/radar-embed.js": ("radar-embed.js", "text/javascript; charset=utf-8"),
        "/radar-retry.js": ("radar-retry.js", "text/javascript; charset=utf-8"),
        "/radar-offline.html": ("radar-offline.html", "text/html; charset=utf-8"),
    }
    for filename, _ in files.values():
        if not (assets / filename).is_file():
            raise FileNotFoundError(assets / filename)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5)

        def do_GET(self):
            route = urlsplit(self.path).path
            status = 200
            encoding = "identity"
            if route == "/api/state":
                body = json.dumps(state.snapshot(), allow_nan=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif route in files:
                filename, content_type = files[route]
                body = (assets / filename).read_bytes()
            elif route in radar.routes:
                try:
                    body, content_type, encoding = radar.read(route, "gzip" in self.headers.get("Accept-Encoding", ""))
                except OSError:
                    status = 502
                    if route == "/radar/":
                        body = (assets / "radar-offline.html").read_bytes()
                        content_type = "text/html; charset=utf-8"
                    else:
                        body = b'{"error":"Radar server unavailable"}'
                        content_type = "application/json; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if encoding == "gzip":
                self.send_header("Content-Encoding", "gzip")
                self.send_header("Vary", "Accept-Encoding")
            if route == "/radar/":
                # Legacy radar HTML has inline styles/bootstrap. Keep that policy
                # inside its frame; the combined shell stays external-script-only.
                policy = "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; object-src 'none'; frame-ancestors 'self'; base-uri 'none'"
            else:
                policy = "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-src 'self'; img-src 'self' blob:; object-src 'none'; frame-ancestors 'none'; base-uri 'none'"
            self.send_header("Content-Security-Policy", policy)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def log_message(self, format, *args):
            if args and str(args[1] if len(args) > 1 else "").startswith("5"):
                super().log_message(format, *args)

    return ThreadingHTTPServer((host, port), Handler)


def create_ros_node(state):
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import MarkerArray

    node = Node("head_map_viewer")
    retained = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(Odometry, "/hansel/head/odometry", lambda m: state.receive("odometry", m), 1)
    node.create_subscription(RosPath, "/hansel/head/path", lambda m: state.receive("path", m), retained)
    node.create_subscription(MarkerArray, "/hansel/localization/relay_markers", lambda m: state.receive("markers", m), retained)
    node.create_subscription(DiagnosticArray, "/diagnostics", lambda m: state.receive("diagnostics", m), 10)
    return node


def update_demo(state, elapsed):
    # Synthetic samples are exclusively opt-in and never used as live fallback.
    with state.lock:
        t = elapsed * 0.25
        x, y = 1.8 * math.sin(t), 1.1 * (1 - math.cos(t))
        yaw = math.atan2(1.1 * math.sin(t), 1.8 * math.cos(t))
        state.pose = {"x": x, "y": y, "yaw": yaw, "stamp": {"sec": 0, "nanosec": 0}}
        state.pose_received = state.clock()
        state.path.append({"x": x, "y": y})
        state.path = state.path[-PATH_LIMIT:]
        state.relays = [
            {"id": "0", "label": "node1 (demo)", "x": 0.6, "y": 0.065, "yaw": 0.21, "provisional": True},
            {"id": "1", "label": "node2 (demo)", "x": 1.5, "y": 0.49, "yaw": 0.74, "provisional": True},
        ]


def main(assets, argv=None):
    parser = argparse.ArgumentParser(description="Read-only HANSEL relative odometry map. No motor commands or sensor-port access.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address; expose only on a trusted mesh")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--frame", default="odom")
    parser.add_argument("--radar-url", default="http://192.168.50.10:8081", help="Existing radar HTTP origin; only viewer assets and GET /api/radar are forwarded")
    parser.add_argument("--demo", action="store_true", help="Synthetic PC preview; no ROS required")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    try:
        RadarProxy(args.radar_url)
    except ValueError as exc:
        parser.error(str(exc))
    state = MapState(args.frame, "demo" if args.demo else "live")
    node = None
    rclpy = None
    server = None
    thread = None
    try:
        if not args.demo:
            try:
                import rclpy
            except ImportError:
                parser.exit(1, "[ERROR] Source the Head ROS setup first, or use --demo for a synthetic PC preview.\n")
            rclpy.init(args=[])
            node = create_ros_node(state)
        server = make_server(state, assets, args.host, args.port, args.radar_url)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"[INFO] {'SYNTHETIC DEMO' if args.demo else 'READ-ONLY LIVE'} map: http://{args.host}:{args.port}/", flush=True)
        start = time.monotonic()
        while args.demo or rclpy.ok():
            if args.demo:
                update_demo(state, time.monotonic() - start)
                time.sleep(0.1)
            else:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        parser.exit(1, f"[ERROR] {exc}\n")
    finally:
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(timeout=3)
            server.server_close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
