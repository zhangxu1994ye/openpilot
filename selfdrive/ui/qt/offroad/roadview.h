#pragma once

#include "selfdrive/ui/qt/onroad/annotated_camera.h"

class RoadViewWindow : public AnnotatedCameraWidget {
  Q_OBJECT

public:
  explicit RoadViewWindow(QWidget* parent);

signals:
  void done();

protected:
  void mouseReleaseEvent(QMouseEvent *event) override;
};
