import os
import time
import socket
import threading
import pyray as rl
import numpy as np
from PIL import Image
from enum import IntEnum
import cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.sidebar import Sidebar, SIDEBAR_WIDTH
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout, PanelType
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.layouts.onboarding import OnboardingWindow
from openpilot.common.swaglog import cloudlog


CAMERA_SAVE_SOCKET = "/tmp/camera_save_trigger.sock"
CAMERA_SAVE_PORT = 8357


class MainState(IntEnum):
  HOME = 0
  SETTINGS = 1
  ONROAD = 2


class MainLayout(Widget):
  def __init__(self):
    super().__init__()

    self._pm = messaging.PubMaster(['bookmarkButton'])

    self._sidebar = Sidebar()
    self._current_mode = MainState.HOME
    self._prev_onroad = False

    # Initialize layouts
    self._layouts = {MainState.HOME: HomeLayout(), MainState.SETTINGS: SettingsLayout(), MainState.ONROAD: AugmentedRoadView()}

    self._sidebar_rect = rl.Rectangle(0, 0, 0, 0)
    self._content_rect = rl.Rectangle(0, 0, 0, 0)

    # Start socket server for remote trigger
    self._socket_server_thread: threading.Thread | None = None
    self._socket_running = False
    self._start_socket_server()

    # Set callbacks
    self._setup_callbacks()

    gui_app.push_widget(self)

    # Start onboarding if terms or training not completed, make sure to push after self
    self._onboarding_window = OnboardingWindow()
    if not self._onboarding_window.completed:
      gui_app.push_widget(self._onboarding_window)

  def _start_socket_server(self):
    if self._socket_server_thread is not None and self._socket_server_thread.is_alive():
      return

    self._socket_running = True
    self._socket_server_thread = threading.Thread(target=self._socket_server_loop, daemon=True)
    self._socket_server_thread.start()

  def _handle_trigger(self):
    cloudlog.debug("Received SAVE_FRAME trigger")
    ui_state.trigger_save_camera_frame()

  def _socket_server_loop(self):
    # Cleanup existing socket
    if os.path.exists(CAMERA_SAVE_SOCKET):
      try:
        os.unlink(CAMERA_SAVE_SOCKET)
      except OSError:
        pass

    # Setup Unix socket
    unix_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    unix_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    unix_server.bind(CAMERA_SAVE_SOCKET)
    unix_server.listen(5)
    os.chmod(CAMERA_SAVE_SOCKET, 0o666)

    # Setup TCP socket
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind(("0.0.0.0", CAMERA_SAVE_PORT))
    tcp_server.listen(5)
    tcp_server.settimeout(0.5)

    unix_server.settimeout(0.5)

    try:
      while self._socket_running:
        # Check Unix socket
        try:
          conn, _ = unix_server.accept()
          try:
            data = conn.recv(1024)
            if data == b"SAVE_FRAME\n":
              self._handle_trigger()
              conn.sendall(b"OK\n")
            else:
              conn.sendall(b"INVALID\n")
          except socket.error:
            pass
          finally:
            conn.close()
        except socket.timeout:
          pass

        # Check TCP socket
        try:
          conn, _ = tcp_server.accept()
          try:
            data = conn.recv(1024)
            if data == b"SAVE_FRAME\n":
              self._handle_trigger()
              conn.sendall(b"OK\n")
            else:
              conn.sendall(b"INVALID\n")
          except socket.error:
            pass
          finally:
            conn.close()
        except socket.timeout:
          pass
    finally:
      unix_server.close()
      tcp_server.close()
      if os.path.exists(CAMERA_SAVE_SOCKET):
        try:
          os.unlink(CAMERA_SAVE_SOCKET)
        except OSError:
          pass

  def _render(self, _):
    self._handle_onroad_transition()
    if ui_state.consume_save_camera_frame_pending():
      self._save_camera_frame()
    self._render_main_content()

  def _setup_callbacks(self):
    ui_state.add_save_camera_frame_callback(self._save_camera_frame)
    self._sidebar.set_callbacks(on_settings=self._on_settings_clicked,
                                on_flag=self._on_bookmark_clicked,
                                open_settings=lambda: self.open_settings(PanelType.TOGGLES),
                                on_metric_click=self._on_metric_clicked,
                                on_connect_metric_click=self._on_connect_metric_clicked)
    # self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self.open_settings(PanelType.FIREHOSE))
    self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self._set_current_layout(MainState.ONROAD))
    self._layouts[MainState.HOME].set_settings_callback(lambda: self.open_settings(PanelType.TOGGLES))
    self._layouts[MainState.SETTINGS].set_callbacks(on_close=self._set_mode_for_state)
    self._layouts[MainState.ONROAD].set_click_callback(self._on_onroad_clicked)
    # device.add_interactive_timeout_callback(self._set_mode_for_state)

  def _on_metric_clicked(self):
    ui_state.toggle_v_ego_override()

  def _on_connect_metric_clicked(self):
    ui_state.trigger_save_camera_frame()

  def _save_camera_frame(self):
    CAM_SAVE_DIR = "/data/myCam"
    os.makedirs(CAM_SAVE_DIR, exist_ok=True)
    timestamp = int(time.time() * 1000)
    saved_count = 0

    for stream_type in (VisionStreamType.VISION_STREAM_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD):
      client = VisionIpcClient("camerad", stream_type, conflate=True)
      if client.connect(False) and client.num_buffers:
        buf = client.recv(timeout_ms=500)
        if buf is not None:
          uv_height = ((buf.height // 2) + 15) // 16 * 16
          uv_plane_size = buf.stride * uv_height

          y = np.array(buf.data[:buf.uv_offset], dtype=np.uint8).reshape((-1, buf.stride))[:buf.height, :buf.width]
          uv_data = buf.data[buf.uv_offset:buf.uv_offset + uv_plane_size]
          u = np.array(uv_data[::2], dtype=np.uint8).reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]
          v = np.array(uv_data[1::2], dtype=np.uint8).reshape((-1, buf.stride // 2))[:buf.height // 2, :buf.width // 2]

          ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
          vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
          yuv = np.dstack((y, ul, vl)).astype(np.int16)
          yuv[:, :, 1:] -= 128
          m = np.array([
            [1.00000, 1.00000, 1.00000],
            [0.00000, -0.39465, 2.03211],
            [1.13983, -0.58060, 0.00000],
          ])
          rgb = np.dot(yuv, m).clip(0, 255).astype(np.uint8)

          stream_name = "road" if stream_type == VisionStreamType.VISION_STREAM_ROAD else "wide"
          filepath = os.path.join(CAM_SAVE_DIR, f"{stream_name}_{timestamp}.png")
          Image.fromarray(rgb).save(filepath, "PNG")
          saved_count += 1
          cloudlog.debug(f"Saved camera frame: {filepath}")

    if saved_count == 0:
      cloudlog.warning("No camera frames captured for save")

  def _update_layout_rects(self):
    self._sidebar_rect = rl.Rectangle(self._rect.x, self._rect.y, SIDEBAR_WIDTH, self._rect.height)

    x_offset = SIDEBAR_WIDTH if self._sidebar.is_visible else 0
    self._content_rect = rl.Rectangle(self._rect.x + x_offset, self._rect.y, self._rect.width - x_offset, self._rect.height)

  def _handle_onroad_transition(self):
    show_camera = ui_state.started or ui_state.camera_preview
    if show_camera != self._prev_onroad:
      self._prev_onroad = show_camera
      self._set_mode_for_state()

  def _set_mode_for_state(self):
    # Check if should show camera preview (either onroad or camera_preview enabled)
    show_camera = ui_state.started or ui_state.camera_preview

    if show_camera:
      # Don't hide sidebar from interactive timeout
      if self._current_mode != MainState.ONROAD:
        self._sidebar.set_visible(False)
      self._set_current_layout(MainState.ONROAD)
    else:
      self._set_current_layout(MainState.HOME)
      self._sidebar.set_visible(True)

  def _set_current_layout(self, layout: MainState):
    if layout != self._current_mode:
      self._layouts[self._current_mode].hide_event()
      self._current_mode = layout
      self._layouts[self._current_mode].show_event()

  def open_settings(self, panel_type: PanelType):
    self._layouts[MainState.SETTINGS].set_current_panel(panel_type)
    self._set_current_layout(MainState.SETTINGS)
    self._sidebar.set_visible(False)

  def _on_settings_clicked(self):
    self.open_settings(PanelType.DEVICE)

  def _on_bookmark_clicked(self):
    user_bookmark = messaging.new_message('bookmarkButton')
    user_bookmark.valid = True
    self._pm.send('bookmarkButton', user_bookmark)

  def _on_onroad_clicked(self):
    self._sidebar.set_visible(not self._sidebar.is_visible)

  def _render_main_content(self):
    # Render sidebar
    if self._sidebar.is_visible:
      self._sidebar.render(self._sidebar_rect)

    content_rect = self._content_rect if self._sidebar.is_visible else self._rect
    self._layouts[self._current_mode].render(content_rect)
