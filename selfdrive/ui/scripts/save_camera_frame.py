#!/usr/bin/env python3
"""
Save camera frame script - triggers camera frame capture via IPC.

Usage:
    python save_camera_frame.py [--host HOST] [--port PORT]

Or trigger via SSH command:
    cd /data/openpilot && python selfdrive/ui/scripts/save_camera_frame.py

Default socket path: /tmp/camera_save_trigger.sock
"""

import argparse
import os
import socket
import sys
import time

SOCKET_PATH = "/tmp/camera_save_trigger.sock"
DEFAULT_HOST = "localhost"
DEFAULT_PORT = 8357


def trigger_save(sock_path: str = SOCKET_PATH, timeout: float = 2.0) -> bool:
    """Send trigger signal via Unix domain socket."""
    try:
        if os.path.exists(sock_path):
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect(sock_path)
            sock.sendall(b"SAVE_FRAME\n")
            response = sock.recv(1024)
            sock.close()
            return response == b"OK\n"
    except (socket.error, OSError) as e:
        pass

    return False


def trigger_save_tcp(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 2.0) -> bool:
    """Send trigger signal via TCP socket."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.sendall(b"SAVE_FRAME\n")
        response = sock.recv(1024)
        sock.close()
        return response == b"OK\n"
    except (socket.error, OSError) as e:
        pass

    return False


def trigger_via_params() -> bool:
    """Trigger via params - write to a param that UI monitors."""
    from openpilot.common.params import Params
    try:
        params = Params()
        timestamp = str(int(time.time() * 1000))
        params.put("SaveCameraFrameTrigger", timestamp.encode('utf-8'))
        return True
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser(description="Trigger camera frame save")
    parser.add_argument("--host", default=DEFAULT_HOST, help="TCP host (default: localhost)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port (default: 8357)")
    parser.add_argument("--unix", help="Unix socket path (default: /tmp/camera_save_trigger.sock)")
    parser.add_argument("--params", action="store_true", help="Use params-based trigger (no server needed)")
    args = parser.parse_args()

    if args.params:
        if trigger_via_params():
            print("Trigger sent via params")
            sys.exit(0)
        else:
            print("Failed to trigger via params", file=sys.stderr)
            sys.exit(1)

    success = False
    if args.unix:
        success = trigger_save(args.unix)
    else:
        success = trigger_save() or trigger_save_tcp(args.host, args.port)

    if success:
        print("Camera frame save triggered")
        sys.exit(0)
    else:
        print("Failed to trigger: UI server not running or no camera available", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
