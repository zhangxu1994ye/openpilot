import math
import numpy as np
import pyray as rl
from msgq.visionipc import VisionStreamType
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.selfdrive.ui.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.label import gui_label


class DriverCameraDialog(CameraView):
  # Text display constants
  TEXT_FONT_SIZE = 18
  TEXT_LINE_HEIGHT = 25
  TEXT_LEFT_MARGIN = 10
  TEXT_TOP_MARGIN = 10
  def __init__(self):
    super().__init__("camerad", VisionStreamType.VISION_STREAM_DRIVER)
    self.driver_state_renderer = DriverStateRenderer()
    # TODO: this can grow unbounded, should be given some thought
    # device.add_interactive_timeout_callback(gui_app.pop_widget)
    ui_state.params.put_bool("IsDriverViewEnabled", True)

  def hide_event(self):
    super().hide_event()
    ui_state.params.put_bool("IsDriverViewEnabled", False)
    self.close()

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    gui_app.pop_widget()

  def __del__(self):
    self.close()

  def _render(self, rect):
    super()._render(rect)

    if not self.frame:
      gui_label(
        rect,
        tr("camera starting"),
        font_size=100,
        font_weight=FontWeight.BOLD,
        alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
      )
      return -1

    self._draw_face_detection(rect)
    self._draw_driver_state_info(rect)
    self.driver_state_renderer.render(rect)

    return -1

  def _draw_face_detection(self, rect: rl.Rectangle) -> None:
    driver_state = ui_state.sm["driverStateV2"]
    is_rhd = driver_state.wheelOnRightProb > 0.5
    driver_data = driver_state.rightDriverData if is_rhd else driver_state.leftDriverData
    face_detect = driver_data.faceProb > 0.7
    if not face_detect:
      return

    # Get face position and orientation
    face_x, face_y = driver_data.facePosition
    face_std = max(driver_data.faceOrientationStd[0], driver_data.faceOrientationStd[1])
    alpha = 0.7
    if face_std > 0.15:
      alpha = max(0.7 - (face_std - 0.15) * 3.5, 0.0)

    # use approx instead of distort_points
    # TODO: replace with distort_points
    fbox_x = int(1080.0 - 1714.0 * face_x)
    fbox_y = int(-135.0 + (504.0 + abs(face_x) * 112.0) + (1205.0 - abs(face_x) * 724.0) * face_y)
    box_size = 220

    line_color = rl.Color(255, 255, 255, int(alpha * 255))
    rl.draw_rectangle_rounded_lines_ex(
      rl.Rectangle(fbox_x - box_size / 2, fbox_y - box_size / 2, box_size, box_size),
      35.0 / box_size / 2,
      10,
      10,
      line_color,
    )

  def _draw_text(self, rect: rl.Rectangle, text: str, y: float, color: rl.Color = rl.WHITE) -> float:
    """Draw a single line of text and return the y position for the next line."""
    font = gui_app.font(FontWeight.NORMAL)
    text_size = measure_text_cached(font, text, self.TEXT_FONT_SIZE)
    text_rect = rl.Rectangle(self.TEXT_LEFT_MARGIN, y, rect.width - self.TEXT_LEFT_MARGIN * 2, self.TEXT_LINE_HEIGHT)
    gui_label(
      text_rect,
      text,
      font_size=self.TEXT_FONT_SIZE,
      color=color,
      font_weight=FontWeight.NORMAL,
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
    )
    return y + self.TEXT_LINE_HEIGHT

  def _draw_driver_state_info(self, rect: rl.Rectangle) -> None:
    """Draw detailed driver state information on the screen."""
    driver_state = ui_state.sm["driverStateV2"]
    is_rhd = driver_state.wheelOnRightProb > 0.5
    driver_data = driver_state.rightDriverData if is_rhd else driver_state.leftDriverData
    face_detected = driver_data.faceProb > 0.7

    text_y = self.TEXT_TOP_MARGIN
    line_height = self.TEXT_LINE_HEIGHT

    # Basic driver state info
    text_y = self._draw_text(rect, f"RHD: {'true' if is_rhd else 'false'}", text_y)
    text_y = self._draw_text(rect, f"Frame ID: {driver_state.frameId}", text_y)
    text_y = self._draw_text(rect, f"Model Exec Time: {driver_state.modelExecutionTime:.3f}ms", text_y)

    # Face detection section
    text_y += 10
    face_color = rl.Color(0, 255, 0) if face_detected else rl.Color(255, 0, 0)
    text_y = self._draw_text(rect, "=== FACE DETECTION ===", text_y, face_color)

    text_y = self._draw_text(rect, f"Face Prob: {driver_data.faceProb * 100:.1f}%", text_y)

    face_pos = driver_data.facePosition
    if len(face_pos) >= 2:
      text_y = self._draw_text(rect, f"Face Position: ({face_pos[0]:.3f}, {face_pos[1]:.3f})", text_y)

    face_pos_std = driver_data.facePositionStd
    if len(face_pos_std) >= 2:
      text_y = self._draw_text(rect, f"Face Pos Std: ({face_pos_std[0]:.3f}, {face_pos_std[1]:.3f})", text_y)

    face_orient = driver_data.faceOrientation
    if len(face_orient) >= 3:
      pitch_deg = face_orient[0] * 180.0 / math.pi
      yaw_deg = face_orient[1] * 180.0 / math.pi
      roll_deg = face_orient[2] * 180.0 / math.pi
      text_y = self._draw_text(rect, f"Face Orient (deg): ({pitch_deg:.1f}, {yaw_deg:.1f}, {roll_deg:.1f})", text_y)

    face_orient_std = driver_data.faceOrientationStd
    if len(face_orient_std) >= 3:
      text_y = self._draw_text(rect, f"Face Orient Std: ({face_orient_std[0]:.3f}, {face_orient_std[1]:.3f}, {face_orient_std[2]:.3f})", text_y)

    # Eye detection section
    text_y += 10
    text_y = self._draw_text(rect, "=== EYE DETECTION ===", text_y, rl.Color(0, 255, 255))

    text_y = self._draw_text(rect, f"Left Eye Prob: {driver_data.leftEyeProb * 100:.1f}%", text_y)
    text_y = self._draw_text(rect, f"Right Eye Prob: {driver_data.rightEyeProb * 100:.1f}%", text_y)

    # Blink detection with color coding
    left_blink = driver_data.leftBlinkProb > 0.5
    right_blink = driver_data.rightBlinkProb > 0.5
    text_y = self._draw_text(rect, f"Left Blink Prob: {driver_data.leftBlinkProb * 100:.1f}%", text_y,
                             rl.Color(255, 255, 0) if left_blink else rl.WHITE)
    text_y = self._draw_text(rect, f"Right Blink Prob: {driver_data.rightBlinkProb * 100:.1f}%", text_y,
                             rl.Color(255, 255, 0) if right_blink else rl.WHITE)

    # Other detection section
    text_y += 10
    text_y = self._draw_text(rect, "=== OTHER DETECTION ===", text_y, rl.Color(255, 128, 0))

    sunglasses = driver_data.sunglassesProb > 0.5
    text_y = self._draw_text(rect, f"Sunglasses Prob: {driver_data.sunglassesProb * 100:.1f}%", text_y,
                             rl.Color(255, 255, 0) if sunglasses else rl.WHITE)

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    driver_view_ratio = 2.0

    # Get stream dimensions
    if self.frame:
      stream_width = self.frame.width
      stream_height = self.frame.height
    else:
      # Default values if frame not available
      stream_width = 1928
      stream_height = 1208

    yscale = stream_height * driver_view_ratio / stream_width
    xscale = yscale * rect.height / rect.width * stream_width / stream_height

    return np.array([
      [xscale, 0.0, 0.0],
      [0.0, yscale, 0.0],
      [0.0, 0.0, 1.0]
    ])


if __name__ == "__main__":
  gui_app.init_window("Driver Camera View")

  driver_camera_view = DriverCameraDialog()
  gui_app.push_widget(driver_camera_view)
  try:
    for _ in gui_app.render():
      ui_state.update()
  finally:
    driver_camera_view.close()
