/*
 * File: tf_monitor_panel.cpp
 * Author: Kane Weng
 * Date: Apr 7, 2026
 *
 * Description:
 * Implementation of the TfMonitorPanel RViz 2 plugin. Contains the logic to
 * actively query the tf2 buffer, extract the spatial relationship between the
 * 'map' and 'base_link' frames, and format the resulting translation and yaw
 * rotation into a clean, inline UI display.
 *
 * Key Features:
 * - Quaternion Conversion: Translates standard ROS 2 quaternion rotations into
 * human-readable Euler yaw angles (degrees) for quick visual debugging.
 * - Exception Handling: Safely catches tf2::TransformException errors to notify
 * the operator directly in the GUI if the transform tree breaks.
 * - Compact Layout: one line, the panel never taller than that line; padding in
 * em so it scales with the (DPI-aware) panel font.
 *
 * Usage:
 * Compiled as part of the nift_rviz_plugins pluginlib library.
 */

#include "nift_rviz_plugins/tf_monitor_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QString>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace nift_rviz_plugins
{

TfMonitorPanel::TfMonitorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;

  tf_label_ = new QLabel("Waiting for map -> base_link...");
  // Proportional font (digits are fixed-width in it): ~25% narrower than monospace
  tf_label_->setStyleSheet(
    "background-color: #2c3e50; "
    "color: #ecf0f1; "
    "padding: 0.35em 0.6em; "
    "border-radius: 0.25em;"
  );
  layout->addWidget(tf_label_);

  setLayout(layout);
  // Never taller than its one line: the dock area hands the rest to the other panels
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  node_ = rclcpp::Node::make_shared("rviz_tf_monitor_panel_node");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  QTimer * timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateTfDisplay()));
  timer->start(100);
}

TfMonitorPanel::~TfMonitorPanel() {}

double getYawFromTransform(const geometry_msgs::msg::TransformStamped & t)
{
  tf2::Quaternion q(
    t.transform.rotation.x,
    t.transform.rotation.y,
    t.transform.rotation.z,
    t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw * 180.0 / M_PI;
}

void TfMonitorPanel::updateTfDisplay()
{
  if (!rclcpp::ok()) {
    return;   // shutting down: spinning a dead context throws and aborts RViz
  }
  rclcpp::spin_some(node_);
  // Removed the <br><br> to keep it inline
  QString display_text = "<b style='color: #f39c12;'>map &rarr; base_link:</b> ";

  try {
    auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    double yaw = getYawFromTransform(t);

    // Formatted into a single line, dropping the Z axis
    display_text += QString("X: %1 | Y: %2 | Yaw: %3&deg;")
      .arg(t.transform.translation.x, 0, 'f', 2)
      .arg(t.transform.translation.y, 0, 'f', 2)
      .arg(yaw, 0, 'f', 1);
  } catch (const tf2::TransformException & ex) {
    display_text += "<span style='color: #e74c3c;'>Transform Missing!</span>";
  }

  tf_label_->setText(display_text);
}

} // namespace nift_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nift_rviz_plugins::TfMonitorPanel, rviz_common::Panel)
