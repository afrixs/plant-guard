//
// Created by matej on 23/12/23.
//

#include "pg_rviz_plugins/pump_crane_joystick.hpp"
#define _USE_MATH_DEFINES
#include <stdio.h>

#include <QPainter>
#include <QMouseEvent>

#include <cmath>

namespace pg_rviz_plugins
{

// BEGIN_TUTORIAL
// The PumpCraneJoystick constructor does the normal Qt thing of
// passing the parent widget to the superclass constructor, then
// initializing the member variables.
PumpCraneJoystick::PumpCraneJoystick(QWidget * parent)
    : QWidget(parent)
{
}

void PumpCraneJoystick::resizeEvent(QResizeEvent * event)
{
  (void)event;

  auto new_size = event->size();
  auto w = new_size.width();
  auto h = new_size.height();
  auto size = (( w > h ) ? h : w) - 1;
  auto hpad = (w - size) / 2;
  auto vpad = (h - size) / 2;

  buttons_.clear();

  Button button;
  button.direction = 0;
  button.pump = false;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad + size/3, vpad + size/2)
    << QPoint(hpad + size/3, vpad + size*2/3)
    << QPoint(hpad + size*2/3, vpad + size*2/3)
    << QPoint(hpad + size*2/3, vpad + size/2);
  buttons_.push_back(button);

  button.direction = 1;
  button.pump = false;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad, vpad + size/2)
    << QPoint(hpad, vpad + size)
    << QPoint(hpad + size/2, vpad + size)
    << QPoint(hpad + size/2, vpad + size/2);
  buttons_.push_back(button);

  button.direction = -1;
  button.pump = false;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad + size/2, vpad + size/2)
    << QPoint(hpad + size/2, vpad + size)
    << QPoint(hpad + size, vpad + size)
    << QPoint(hpad + size, vpad + size/2);
  buttons_.push_back(button);

  button.direction = 0;
  button.pump = true;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad + size/3, vpad + size/3)
    << QPoint(hpad + size/3, vpad + size/2)
    << QPoint(hpad + size*2/3, vpad + size/2)
    << QPoint(hpad + size*2/3, vpad + size/3);
  buttons_.push_back(button);

  button.direction = 1;
  button.pump = true;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad, vpad)
    << QPoint(hpad, vpad + size/2)
    << QPoint(hpad + size/2, vpad + size/2)
    << QPoint(hpad + size/2, vpad);
  buttons_.push_back(button);

  button.direction = -1;
  button.pump = true;
  button.polygon.clear();
  button.polygon
    << QPoint(hpad + size/2, vpad)
    << QPoint(hpad + size/2, vpad + size/2)
    << QPoint(hpad + size, vpad + size/2)
    << QPoint(hpad + size, vpad);
  buttons_.push_back(button);

  QWidget::resizeEvent(event);
}

// This paintEvent() is complex because of the drawing of the two
// arc-arrows representing wheel motion.  It is not particularly
// relevant to learning how to make an RViz plugin, so I will kind of
// skim it.
void PumpCraneJoystick::paintEvent(QPaintEvent * event)
{
  (void)event;

  // The background color and crosshair lines are drawn differently
  // depending on whether this widget is enabled or not.  This gives a
  // nice visual indication of whether the control is "live".
  QColor background;
  QColor line_color;
  QColor pump_background;
  if (isEnabled()) {
    background = Qt::white;
    line_color = Qt::black;
    pump_background = Qt::blue;
  } else {
    background = Qt::lightGray;
    line_color = Qt::darkGray;
    pump_background = Qt::darkBlue;
  }

  QPainter painter(this);
  painter.setPen(line_color);

  for (size_t i = buttons_.size() - 1;  i < buttons_.size(); i--) {
    QColor button_color = buttons_[i].pump ? pump_background : background;
    if (i == selected_button_) {
      button_color = button_color.lighter(60);
    } else if (i == mouse_over_button_) {
      button_color = button_color.lighter(30);
    }
    painter.setBrush(button_color);
    painter.drawPolygon(buttons_[i].polygon);
  }
}

// Every mouse move event received here sends a velocity because Qt
// only sends us mouse move events if there was previously a
// mouse-press event while in the widget.
void PumpCraneJoystick::mouseMoveEvent(QMouseEvent * event)
{
  sendJoystickFromMouse(event->x(), event->y(), width(), height());
}

// Mouse-press events should send the velocities too, of course.
void PumpCraneJoystick::mousePressEvent(QMouseEvent * event)
{
  sendJoystickFromMouse(event->x(), event->y(), width(), height());
}

// When the mouse leaves the widget but the button is still held down,
// we don't get the leaveEvent() because the mouse is "grabbed" (by
// default from Qt).  However, when the mouse drags out of the widget
// and then other buttons are pressed (or possibly other
// window-manager things happen), we will get a leaveEvent() but not a
// mouseReleaseEvent().  Without catching this event you can have a
// robot stuck "on" without the user controlling it.
void PumpCraneJoystick::leaveEvent(QEvent * event)
{
  (void)event;
  stop();
}

// The ordinary way to stop: let go of the mouse button.
void PumpCraneJoystick::mouseReleaseEvent(QMouseEvent * event)
{
  (void)event;
  stop();
}

// Compute and emit linear and angular velocities based on Y and X
// mouse positions relative to the central square.
void PumpCraneJoystick::sendJoystickFromMouse(int x, int y, int /*width*/, int /*height*/)
{
  size_t last_selected_button = selected_button_;
  for (size_t i = 0; i < buttons_.size(); i++) {
    if (buttons_[i].polygon.containsPoint(QPoint(x, y), Qt::OddEvenFill)) {
      selected_button_ = i;
      break;
    }
  }
  if (selected_button_ != last_selected_button && selected_button_ < buttons_.size()) {
    Q_EMIT outputJoystick(buttons_[selected_button_].direction, buttons_[selected_button_].pump);
    update();
  }
}

// How to stop: emit velocities of 0!
void PumpCraneJoystick::stop()
{
  selected_button_ = buttons_.size();
  Q_EMIT outputJoystick(0, false);
  update();
}
// END_TUTORIAL

}  // end namespace pg_rviz_plugins