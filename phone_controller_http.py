#!/usr/bin/env python3
"""HTTP routes and server for the phone controller."""

import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from phone_controller_state import PHONE_DIAGNOSTICS
from phone_controller_ui import HTML


def make_handler(shared, map_state, odom_session, shutdown_event, supervisor, node):
    class Handler(BaseHTTPRequestHandler):
        server_version = "Robot1Phone/1.0"

        def log_message(self, fmt, *args):
            return

        def send_body(self, body, content_type, status=200):
            started = getattr(self, "_phone_request_started", None)
            path = getattr(self, "_phone_request_path", "unknown")
            if started is not None:
                PHONE_DIAGNOSTICS.record(
                    "http_request",
                    "end",
                    duration_ms=(time.monotonic() - started) * 1000.0,
                    detail=f"path={path},status={status},bytes={len(body)}",
                )
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            self._phone_request_started = time.monotonic()
            self._phone_request_path = parsed.path
            PHONE_DIAGNOSTICS.record(
                "http_request", "start", detail=f"path={parsed.path}"
            )

            if parsed.path == "/":
                self.send_body(HTML.encode("utf-8"), "text/html; charset=utf-8")
                return

            if parsed.path == "/cmd":
                query = parse_qs(parsed.query)
                key = query.get("key", ["x"])[0]
                speed = query.get("speed", [None])[0]
                shared.set_key(key, speed)
                self.send_body(b"OK\n", "text/plain; charset=utf-8")
                return

            if parsed.path == "/shutdown":
                shared.set_key("x", 0)
                shutdown_event.set()
                self.send_body(b"SHUTTING DOWN\n", "text/plain; charset=utf-8")
                return

            if parsed.path == "/reset":
                shared.set_key("x", 0)
                session_ok, session_message = odom_session.reset()
                if not session_ok:
                    body = json.dumps({
                        "ok": False,
                        "message": session_message,
                    }).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8", 409)
                    return
                success, message = node.reset_slam()
                if success:
                    map_state.reset_for_new_map()
                    body = json.dumps({
                        "ok": True,
                        "message": message,
                        "odom_message": session_message,
                    }).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8")
                else:
                    body = json.dumps({"ok": False, "message": message}).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8", 503)
                return

            if parsed.path == "/odom_test/finish":
                shared.set_key("x", 0)
                query = parse_qs(parsed.query)
                allow_stationary = query.get("allow_stationary", ["0"])[0].lower() in {
                    "1", "true", "yes", "on"
                }
                status = odom_session.request_finish(allow_stationary=allow_stationary)
                body = json.dumps(status, separators=(",", ":")).encode("utf-8")
                http_status = 202 if status["state"] == "FINALIZING" else 200
                if status["state"] == "ERROR":
                    http_status = 409
                self.send_body(body, "application/json; charset=utf-8", http_status)
                return

            if parsed.path == "/odom_test/config":
                query = parse_qs(parsed.query)
                mode = query.get("mode", [None])[0]
                direction = query.get("direction", [None])[0]
                turns = query.get("turns", [None])[0]
                success, message = odom_session.configure(
                    mode=mode,
                    direction=direction,
                    expected_turns=turns,
                )
                status = odom_session.status()
                status["ok"] = success
                status["message"] = message
                body = json.dumps(status, separators=(",", ":")).encode("utf-8")
                self.send_body(
                    body,
                    "application/json; charset=utf-8",
                    200 if success else 409,
                )
                return

            if parsed.path == "/map.json":
                try:
                    known_version = int(parse_qs(parsed.query).get("version", ["0"])[0])
                except (TypeError, ValueError):
                    known_version = 0
                current_version = map_state.version_number()
                if current_version > 0 and known_version == current_version:
                    body = json.dumps({
                        "ok": True,
                        "unchanged": True,
                        "version": current_version,
                    }).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8")
                    return
                body = map_state.map_body()
                if body is None:
                    body = json.dumps({
                        "ok": False,
                        "status": "Waiting for /map from slam_toolbox...",
                    }).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            if parsed.path == "/map_off.json":
                try:
                    known_version = int(parse_qs(parsed.query).get("version", ["0"])[0])
                except (TypeError, ValueError):
                    known_version = 0
                current_version = map_state.map_off_version_number()
                if current_version > 0 and known_version == current_version:
                    body = json.dumps({
                        "ok": True,
                        "unchanged": True,
                        "version": current_version,
                    }).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8")
                    return
                body = map_state.map_off_body()
                if body is None:
                    body = json.dumps({
                        "ok": False,
                        "status": "Waiting for /map_off from slam_toolbox_off...",
                    }).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            if parsed.path == "/pose.json":
                query = parse_qs(parsed.query)
                try:
                    requested = int(query.get("path_from", ["0"])[0])
                except (TypeError, ValueError):
                    requested = 0
                try:
                    requested_off = int(query.get("path_off_from", ["0"])[0])
                except (TypeError, ValueError):
                    requested_off = 0
                body = json.dumps(
                    map_state.pose_snapshot(requested, requested_off),
                    separators=(",", ":"),
                ).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            if parsed.path == "/status":
                key, speed_rpm, last_update = shared.status()
                lidar = map_state.lidar_snapshot()
                body = json.dumps({
                    "key": key,
                    "speed_rpm": speed_rpm,
                    "seconds_since_command": max(0.0, time.monotonic() - last_update),
                    "map": map_state.has_map(),
                    "lidar": lidar,
                    "odom_test": odom_session.status(),
                    "startup": supervisor.startup_status(map_state),
                }).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            self.send_body(b"Not found\n", "text/plain; charset=utf-8", 404)

    return Handler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
