#!/usr/bin/env python3
"""
Camera frame save server - saves camera frames from camerad to disk.

Listens on Unix socket and TCP port, captures frames when triggered,
and saves them as PNG to /data/myCam/ directory.

Usage:
    python save_camera_server.py [--unix-socket PATH] [--tcp-port PORT]

Socket commands:
    SAVE_FRAME - capture and save current frame
    SAVE_ROAD  - capture road camera only
    SAVE_DRIVER - capture driver camera only
    QUIT       - shutdown server
"""

import argparse
import os
import socket
import threading
import signal
import sys
import time
from datetime import datetime

import numpy as np
from PIL import Image

import cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.system.hardware import HARDWARE

SAVE_DIR = "/data/myCam"
UNIX_SOCKET_PATH = "/tmp/my_camera_save.sock"
TCP_PORT = 8358

VISION_STREAMS = {
    "roadCameraState": VisionStreamType.VISION_STREAM_ROAD,
    "driverCameraState": VisionStreamType.VISION_STREAM_DRIVER,
    "wideRoadCameraState": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def yuv_to_rgb(y, u, v):
    """Convert YUV420 to RGB."""
    ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
    vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)

    yuv = np.dstack((y, ul, vl)).astype(np.int16)
    yuv[:, :, 1:] -= 128

    m = np.array([
        [1.00000,  1.00000, 1.00000],
        [0.00000, -0.39465, 2.03211],
        [1.13983, -0.58060, 0.00000],
    ])
    rgb = np.dot(yuv, m).clip(0, 255)
    return rgb.astype(np.uint8)


def extract_image(buf):
    """Extract RGB image from NV12 buffer."""
    uv_height = ((buf.height // 2) + 15) // 16 * 16
    uv_plane_size = buf.stride * uv_height

    y = np.array(buf.data[:buf.uv_offset], dtype=np.uint8).reshape((-1, buf.stride))[:buf.height, :buf.width]
    uv_data = buf.data[buf.uv_offset:buf.uv_offset + uv_plane_size]
    u = np.array(uv_data[::2], dtype=np.uint8).reshape((-1, buf.stride//2))[:buf.height//2, :buf.width//2]
    v = np.array(uv_data[1::2], dtype=np.uint8).reshape((-1, buf.stride//2))[:buf.height//2, :buf.width//2]

    return yuv_to_rgb(y, u, v)


def save_frame_to_png(frame_type: str, save_path: str) -> bool:
    """Capture a frame from the specified camera and save as PNG."""
    try:
        stream_type = VISION_STREAMS.get(frame_type)
        if stream_type is None:
            print(f"Unknown frame type: {frame_type}")
            return False

        vipc_client = VisionIpcClient("camerad", stream_type, True)
        vipc_client.connect(True)

        buf = vipc_client.recv()
        rgb = extract_image(buf)

        img = Image.fromarray(rgb)
        img.save(save_path, "PNG")
        return True
    except Exception as e:
        print(f"Error capturing {frame_type}: {e}")
        return False


def ensure_save_dir():
    """Ensure save directory exists."""
    os.makedirs(SAVE_DIR, exist_ok=True)


def generate_filename(camera_type: str) -> str:
    """Generate filename with timestamp."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return f"{camera_type}_{ts}.png"


def handle_client_unix(client_sock, client_addr):
    """Handle a client connection on Unix socket."""
    try:
        while True:
            data = client_sock.recv(256).decode('utf-8').strip()
            if not data:
                break

            if data == "QUIT":
                client_sock.sendall(b"QUIT_ACK\n")
                client_sock.close()
                return

            elif data == "SAVE_FRAME":
                result = save_all_frames()
                client_sock.sendall(b"OK\n" if result else b"ERROR\n")

            elif data == "SAVE_ROAD":
                ensure_save_dir()
                filename = generate_filename("road")
                filepath = os.path.join(SAVE_DIR, filename)
                result = save_frame_to_png("roadCameraState", filepath)
                client_sock.sendall(b"OK\n" if result else b"ERROR\n")

            elif data == "SAVE_DRIVER":
                ensure_save_dir()
                filename = generate_filename("driver")
                filepath = os.path.join(SAVE_DIR, filename)
                result = save_frame_to_png("driverCameraState", filepath)
                client_sock.sendall(b"OK\n" if result else b"ERROR\n")

            elif data == "SAVE_WIDE":
                ensure_save_dir()
                filename = generate_filename("wide")
                filepath = os.path.join(SAVE_DIR, filename)
                result = save_frame_to_png("wideRoadCameraState", filepath)
                client_sock.sendall(b"OK\n" if result else b"ERROR\n")

            else:
                client_sock.sendall(b"UNKNOWN_CMD\n")

    except Exception as e:
        print(f"Unix client error: {e}")
        client_sock.close()


def handle_client_tcp(client_sock, client_addr):
    """Handle a client connection on TCP socket."""
    try:
        client_sock.settimeout(30)
        while True:
            try:
                data = client_sock.recv(256).decode('utf-8').strip()
                if not data:
                    break

                if data == "QUIT":
                    client_sock.sendall(b"QUIT_ACK\n")
                    client_sock.close()
                    return

                elif data == "SAVE_FRAME":
                    result = save_all_frames()
                    client_sock.sendall(b"OK\n" if result else b"ERROR\n")

                elif data == "SAVE_ROAD":
                    ensure_save_dir()
                    filename = generate_filename("road")
                    filepath = os.path.join(SAVE_DIR, filename)
                    result = save_frame_to_png("roadCameraState", filepath)
                    client_sock.sendall(b"OK\n" if result else b"ERROR\n")

                elif data == "SAVE_DRIVER":
                    ensure_save_dir()
                    filename = generate_filename("driver")
                    filepath = os.path.join(SAVE_DIR, filename)
                    result = save_frame_to_png("driverCameraState", filepath)
                    client_sock.sendall(b"OK\n" if result else b"ERROR\n")

                elif data == "SAVE_WIDE":
                    ensure_save_dir()
                    filename = generate_filename("wide")
                    filepath = os.path.join(SAVE_DIR, filename)
                    result = save_frame_to_png("wideRoadCameraState", filepath)
                    client_sock.sendall(b"OK\n" if result else b"ERROR\n")

                else:
                    client_sock.sendall(b"UNKNOWN_CMD\n")

            except socket.timeout:
                break

    except Exception as e:
        print(f"TCP client error: {e}")
    finally:
        client_sock.close()


def save_all_frames() -> bool:
    """Save all camera frames."""
    ensure_save_dir()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    road_path = os.path.join(SAVE_DIR, f"road_{ts}.png")
    wide_path = os.path.join(SAVE_DIR, f"wide_{ts}.png")
    driver_path = os.path.join(SAVE_DIR, f"driver_{ts}.png")

    results = []
    results.append(save_frame_to_png("roadCameraState", road_path))
    results.append(save_frame_to_png("wideRoadCameraState", wide_path))
    results.append(save_frame_to_png("driverCameraState", driver_path))

    return all(results)


def run_unix_server(sock_path: str, running_event):
    """Run Unix domain socket server."""
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(sock_path)
    server.listen(5)

    print(f"Unix server listening on {sock_path}")

    while running_event.is_set():
        server.settimeout(1.0)
        try:
            client, addr = server.accept()
            threading.Thread(target=handle_client_unix, args=(client, addr), daemon=True).start()
        except socket.timeout:
            continue
        except Exception as e:
            if running_event.is_set():
                print(f"Unix server error: {e}")

    server.close()
    if os.path.exists(sock_path):
        os.unlink(sock_path)


def run_tcp_server(port: int, running_event):
    """Run TCP socket server."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(5)

    print(f"TCP server listening on port {port}")

    while running_event.is_set():
        server.settimeout(1.0)
        try:
            client, addr = server.accept()
            print(f"TCP client connected from {addr}")
            threading.Thread(target=handle_client_tcp, args=(client, addr), daemon=True).start()
        except socket.timeout:
            continue
        except Exception as e:
            if running_event.is_set():
                print(f"TCP server error: {e}")

    server.close()


def main():
    parser = argparse.ArgumentParser(description="Camera frame save server")
    parser.add_argument("--unix-socket", default=UNIX_SOCKET_PATH, help="Unix socket path")
    parser.add_argument("--tcp-port", type=int, default=TCP_PORT, help="TCP port")
    args = parser.parse_args()

    ensure_save_dir()
    print(f"Save directory: {SAVE_DIR}")

    running_event = threading.Event()
    running_event.set()

    def signal_handler(sig, frame):
        print("\nShutting down server...")
        running_event.clear()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    threads = []
    threads.append(threading.Thread(target=run_unix_server, args=(args.unix_socket, running_event), daemon=True))
    threads.append(threading.Thread(target=run_tcp_server, args=(args.tcp_port, running_event), daemon=True))

    for t in threads:
        t.start()

    print("Camera save server running. Press Ctrl+C to stop.")

    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
