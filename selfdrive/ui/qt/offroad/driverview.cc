#include "selfdrive/ui/qt/offroad/driverview.h"

#include <algorithm>
#include <QPainter>

#include "selfdrive/ui/qt/util.h"

const int FACE_IMG_SIZE = 130;

DriverViewWindow::DriverViewWindow(QWidget* parent) : CameraWidget("camerad", VISION_STREAM_DRIVER, true, parent) {
  face_img = loadPixmap("../assets/img_driver_face_static.png", {FACE_IMG_SIZE, FACE_IMG_SIZE});
  QObject::connect(this, &CameraWidget::clicked, this, &DriverViewWindow::done);
  QObject::connect(device(), &Device::interactiveTimeout, this, [this]() {
    if (isVisible()) {
      emit done();
    }
  });
}

void DriverViewWindow::showEvent(QShowEvent* event) {
  params.putBool("IsDriverViewEnabled", true);
  device()->resetInteractiveTimeout(60);
  CameraWidget::showEvent(event);
}

void DriverViewWindow::hideEvent(QHideEvent* event) {
  params.putBool("IsDriverViewEnabled", false);
  stopVipcThread();
  CameraWidget::hideEvent(event);
}

void DriverViewWindow::paintGL() {
  CameraWidget::paintGL();

  std::lock_guard lk(frame_lock);
  QPainter p(this);
  // startup msg
  if (frames.empty()) {
    p.setPen(Qt::white);
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setFont(InterFont(100, QFont::Bold));
    p.drawText(geometry(), Qt::AlignCenter, tr("camera starting"));
    return;
  }

  const auto &sm = *(uiState()->sm);
  cereal::DriverStateV2::Reader driver_state = sm["driverStateV2"].getDriverStateV2();
  bool is_rhd = driver_state.getWheelOnRightProb() > 0.5;
  auto driver_data = is_rhd ? driver_state.getRightDriverData() : driver_state.getLeftDriverData();

  bool face_detected = driver_data.getFaceProb() > 0.7;
  if (face_detected) {
    auto fxy_list = driver_data.getFacePosition();
    auto std_list = driver_data.getFaceOrientationStd();
    float face_x = fxy_list[0];
    float face_y = fxy_list[1];
    float face_std = std::max(std_list[0], std_list[1]);

    float alpha = 0.7;
    if (face_std > 0.15) {
      alpha = std::max(0.7 - (face_std-0.15)*3.5, 0.0);
    }
    const int box_size = 220;
    // use approx instead of distort_points
    int fbox_x = 1080.0 - 1714.0 * face_x;
    int fbox_y = -135.0 + (504.0 + std::abs(face_x)*112.0) + (1205.0 - std::abs(face_x)*724.0) * face_y;
    p.setPen(QPen(QColor(255, 255, 255, alpha * 255), 10));
    p.drawRoundedRect(fbox_x - box_size / 2, fbox_y - box_size / 2, box_size, box_size, 35.0, 35.0);
  }

  // Draw detailed driver state information
  p.setRenderHint(QPainter::TextAntialiasing);
  int text_y = 30;
  int line_height = 25;
  int font_size = 18;
  QFont font = InterFont(font_size);
  p.setFont(font);

  // Basic driver state info
  p.setPen(Qt::white);
  p.drawText(10, text_y, QString("RHD: %1").arg(is_rhd ? "true" : "false"));
  text_y += line_height;

  p.drawText(10, text_y, QString("Frame ID （帧 ID）: %1").arg(driver_state.getFrameId()));
  text_y += line_height;

  p.drawText(10, text_y, QString("Model Exec Time （模型执行时间）: %1ms").arg(driver_state.getModelExecutionTime(), 0, 'f', 3));
  text_y += line_height;

  // p.drawText(10, text_y, QString("GPU Exec Time （GPU 执行时间）: %1ms").arg(driver_state.getGpuExecutionTime(), 0, 'f', 3));
  // text_y += line_height;

  // Global probabilities
  p.drawText(10, text_y, QString("Poor Vision Prob （视野不良概率）: %1%").arg(driver_state.getPoorVisionProb() * 100, 0, 'f', 1));
  text_y += line_height;

  p.drawText(10, text_y, QString("Wheel Right Prob （右舵概率）: %1%").arg(driver_state.getWheelOnRightProb() * 100, 0, 'f', 1));
  text_y += line_height;

  // Face detection section
  text_y += 10;
  p.setPen(face_detected ? Qt::green : Qt::red);
  p.drawText(10, text_y, "=== FACE DETECTION ===");
  text_y += line_height;

  p.setPen(Qt::white);
  p.drawText(10, text_y, QString("Face Prob （面部检测概率 >0.7 认为检测到人脸）: %1%").arg(driver_data.getFaceProb() * 100, 0, 'f', 1));
  text_y += line_height;

  auto face_pos = driver_data.getFacePosition();
  if (face_pos.size() >= 2) {
    p.drawText(10, text_y, QString("Face Position （面部位置）: (%1, %2)").arg(face_pos[0], 0, 'f', 3).arg(face_pos[1], 0, 'f', 3));
    text_y += line_height;
  }

  auto face_pos_std = driver_data.getFacePositionStd();
  if (face_pos_std.size() >= 2) {
    p.drawText(10, text_y, QString("Face Pos Std （面部位置标准差）: (%1, %2)").arg(face_pos_std[0], 0, 'f', 3).arg(face_pos_std[1], 0, 'f', 3));
    text_y += line_height;
  }

  auto face_orient = driver_data.getFaceOrientation();
  if (face_orient.size() >= 3) {
    p.drawText(10, text_y, QString("Face Orient （面部朝向）: (%1, %2, %3)").arg(face_orient[0], 0, 'f', 3).arg(face_orient[1], 0, 'f', 3).arg(face_orient[2], 0, 'f', 3));
    text_y += line_height;
  }

  auto face_orient_std = driver_data.getFaceOrientationStd();
  if (face_orient_std.size() >= 3) {
    p.drawText(10, text_y, QString("Face Orient Std （面部朝向标准差）: (%1, %2, %3)").arg(face_orient_std[0], 0, 'f', 3).arg(face_orient_std[1], 0, 'f', 3).arg(face_orient_std[2], 0, 'f', 3));
    text_y += line_height;
  }

  // Eye detection section
  text_y += 10;
  p.setPen(Qt::cyan);
  p.drawText(10, text_y, "=== EYE DETECTION ===");
  text_y += line_height;

  p.setPen(Qt::white);
  p.drawText(10, text_y, QString("Left Eye Prob （左眼检测概率）: %1%").arg(driver_data.getLeftEyeProb() * 100, 0, 'f', 1));
  text_y += line_height;

  p.drawText(10, text_y, QString("Right Eye Prob （右眼检测概率）: %1%").arg(driver_data.getRightEyeProb() * 100, 0, 'f', 1));
  text_y += line_height;

  // Blink detection with color coding
  bool left_blink = driver_data.getLeftBlinkProb() > 0.5;
  bool right_blink = driver_data.getRightBlinkProb() > 0.5;

  p.setPen(left_blink ? Qt::yellow : Qt::white);
  p.drawText(10, text_y, QString("Left Blink Prob （左眼眨眼概率）: %1%").arg(driver_data.getLeftBlinkProb() * 100, 0, 'f', 1));
  text_y += line_height;

  p.setPen(right_blink ? Qt::yellow : Qt::white);
  p.drawText(10, text_y, QString("Right Blink Prob （右眼眨眼概率）: %1%").arg(driver_data.getRightBlinkProb() * 100, 0, 'f', 1));
  text_y += line_height;

  // Other detection section
  text_y += 10;
  p.setPen(QColor(255, 128, 0)); // Orange
  p.drawText(10, text_y, "=== OTHER DETECTION ===");
  text_y += line_height;

  bool sunglasses = driver_data.getSunglassesProb() > 0.5;
  p.setPen(sunglasses ? Qt::yellow : Qt::white);
  p.drawText(10, text_y, QString("Sunglasses Prob （太阳镜检测概率）: %1%").arg(driver_data.getSunglassesProb() * 100, 0, 'f', 1));
  text_y += line_height;

  bool occluded = driver_data.getOccludedProb() > 0.5;
  p.setPen(occluded ? Qt::red : Qt::white);
  p.drawText(10, text_y, QString("Occluded Prob （遮挡检测概率）: %1%").arg(driver_data.getOccludedProb() * 100, 0, 'f', 1));
  text_y += line_height;

  // Ready state probabilities
  auto ready_prob = driver_data.getReadyProb();
  if (ready_prob.size() > 0) {
    QString ready_text = "Ready Prob （准备就绪概率）: [";
    for (size_t i = 0; i < ready_prob.size(); ++i) {
      if (i > 0) ready_text += ", ";
      ready_text += QString::number(ready_prob[i] * 100, 'f', 1) + "%";
    }
    ready_text += "]";
    p.setPen(Qt::green);
    p.drawText(10, text_y, ready_text);
    text_y += line_height;
  }

  auto not_ready_prob = driver_data.getNotReadyProb();
  if (not_ready_prob.size() > 0) {
    QString not_ready_text = "Not Ready Prob （未准备就绪概率）: [";
    for (size_t i = 0; i < not_ready_prob.size(); ++i) {
      if (i > 0) not_ready_text += ", ";
      not_ready_text += QString::number(not_ready_prob[i] * 100, 'f', 1) + "%";
    }
    not_ready_text += "]";
    p.setPen(QColor(255, 100, 100)); // Light red
    p.drawText(10, text_y, not_ready_text);
    text_y += line_height;
  }

  // icon
  const int img_offset = 60;
  const int img_x = is_rhd ? rect().right() - FACE_IMG_SIZE - img_offset : rect().left() + img_offset;
  const int img_y = rect().bottom() - FACE_IMG_SIZE - img_offset;
  p.setOpacity(face_detected ? 1.0 : 0.2);
  p.drawPixmap(img_x, img_y, face_img);
}
