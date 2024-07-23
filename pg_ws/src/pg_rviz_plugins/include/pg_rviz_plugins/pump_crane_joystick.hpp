//
// Created by matej on 23/12/23.
//

#ifndef PLANT_GUARD_PUMP_CRANE_JOYSTICK_HPP
#define PLANT_GUARD_PUMP_CRANE_JOYSTICK_HPP

#include <QWidget>

namespace pg_rviz_plugins
{

// BEGIN_TUTORIAL
// PumpCraneJoystick implements a control which translates mouse Y values
// into linear velocities and mouse X values into angular velocities.
//
// For maximum reusability, this class is only responsible for user
// interaction and display inside its widget.  It does not make any
// ROS or RViz calls.  It communicates its data to the outside just
// via Qt signals.
class PumpCraneJoystick : public QWidget
{
  Q_OBJECT

public:
  // This class is not instantiated by pluginlib::ClassLoader, so the
  // constructor has no restrictions.
  explicit PumpCraneJoystick(QWidget * parent = 0);

  // We override QWidget::paintEvent() to do custom painting.
  virtual void paintEvent(QPaintEvent * event);
  virtual void resizeEvent(QResizeEvent * event);

  // We override the mouse events and leaveEvent() to keep track of
  // what the mouse is doing.
  virtual void mouseMoveEvent(QMouseEvent * event);
  virtual void mousePressEvent(QMouseEvent * event);
  virtual void mouseReleaseEvent(QMouseEvent * event);
  virtual void leaveEvent(QEvent * event);

  // Override sizeHint() to give the layout managers some idea of a
  // good size for this.
  virtual QSize sizeHint() const {return QSize(150, 150);}

Q_SIGNALS:
  // We emit outputJoystick() whenever it changes.
  void outputJoystick(int direction, bool pump);

protected:
  // mouseMoveEvent() and mousePressEvent() need the same math to
  // figure the velocities, so I put that in here.
  void sendJoystickFromMouse(int x, int y, int width, int height);

  // A function to emit zero velocity.
  void stop();

  struct Button {
    int direction;
    bool pump;
    QPolygon polygon;
  };
  // Finally the member variables:
  size_t selected_button_;
  size_t mouse_over_button_;
  std::vector<Button> buttons_;
};
// END_TUTORIAL

}  // end namespace pg_rviz_plugins

#endif //PLANT_GUARD_PUMP_CRANE_JOYSTICK_HPP
