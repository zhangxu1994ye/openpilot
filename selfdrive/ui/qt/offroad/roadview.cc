#include "selfdrive/ui/qt/offroad/roadview.h"

RoadViewWindow::RoadViewWindow(QWidget* parent) : AnnotatedCameraWidget(VISION_STREAM_ROAD, parent) {
}

void RoadViewWindow::mouseReleaseEvent(QMouseEvent *event) {
  AnnotatedCameraWidget::mouseReleaseEvent(event);
  emit done();
}
