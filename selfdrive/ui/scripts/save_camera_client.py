#!/usr/bin/env python3
"""
Camera save client - triggers camera frame capture via Unix socket or TCP.

Usage:
    # Via Unix socket (default)
    python save_camera_client.py

    # Via TCP
    python save_camera_client.py --tcp --host 192.168.1.100 --port 8358

    # Via network with custom port
    python save_camera_client.py --tcp --port 8358

Commands:
    SAVE_FRAME - capture all cameras
    SAVE_ROAD  - capture road camera only
    SAVE_WIDE  - capture wide road camera only
    SAVE_DRIVER - capture driver camera only
    QUIT       - shutdown server
"""

import argparse
import os
import socket
import sys
import time

UNIX_SOCKET_PATH = "/tmp/my_camera_save.sock"
DEFAULT_HOST = "localhost"
DEFAULT_PORT = 8358


def send_command_unix(sock_path: str, command: str, timeout: float = 5.0) -> bool:
    """Send command via Unix domain socket."""
    try:
        if not os.path.exists(sock_path):
            print(f"Unix socket not found: {sock_path}", file=sys.stderr)
            return False

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(sock_path)
        sock.sendall(f"{command}\n".encode('utf-8'))
        response = sock.recv(1024)
        sock.close()

        if response == b"OK\n":
            return True
        elif response == b"QUIT_ACK\n":
            print("Server shutdown requested")
            return True
        else:
            print(f"Server response: {response}", file=sys.stderr)
            return False

    except socket.timeout:
        print("Timeout waiting for server response", file=sys.stderr)
        return False
    except OSError as e:
        print(f"Socket error: {e}", file=sys.stderr)
        return False


def send_command_tcp(host: str, port: int, command: str, timeout: float = 5.0) -> bool:
    """Send command via TCP socket."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.sendall(f"{command}\n".encode('utf-8'))
        response = sock.recv(1024)
        sock.close()

        if response == b"OK\n":
            return True
        elif response == b"QUIT_ACK\n":
            print("Server shutdown requested")
            return True
        else:
            print(f"Server response: {response}", file=sys.stderr)
            return False

    except socket.timeout:
        print("Timeout waiting for server response", file=sys.stderr)
        return False
    except OSError as e:
        print(f"Connection error: {e}", file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(description="Camera save client")
    parser.add_argument("--unix-socket", default=UNIX_SOCKET_PATH, help="Unix socket path")
    parser.add_argument("--tcp", action="store_true", help="Use TCP instead of Unix socket")
    parser.add_argument("--host", default=DEFAULT_HOST, help="TCP host (default: localhost)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port (default: 8358)")
    parser.add_argument("--command", default="SAVE_FRAME",
                        choices=["SAVE_FRAME", "SAVE_ROAD", "SAVE_WIDE", "SAVE_DRIVER", "QUIT"],
                        help="Command to send (default: SAVE_FRAME)")

    args = parser.parse_args()

    if args.tcp:
        success = send_command_tcp(args.host, args.port, args.command)
    else:
        success = send_command_unix(args.unix_socket, args.command)

    if success:
        if args.command == "QUIT":
            print("Shutdown command sent")
        else:
            print(f"{args.command} completed successfully")
        sys.exit(0)
    else:
        print("Command failed", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
