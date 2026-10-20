/*
 * File: tf_monitor_panel.hpp
 * Author: Kane Weng
 * Date: Apr 7, 2026
 *
 * Description:
 * Header file for the TF Monitor RViz 2 panel. This plugin provides a quick,
 * on-screen verification that the critical coordinate transforms (TF) are
 * actively being broadcast by the localization nodes. It prevents the need to
 * constantly run `tf2_echo` in a separate terminal during testing.
 *
 * Key Features:
 * - Direct TF Listening: Integrates a dedicated tf2_ros::Buffer and
 * TransformListener to query the active ROS 2 transform tree.
 * - Periodic UI Polling: Uses a QTimer to consistently refresh the transform
 * status display on the GUI at a set interval.
 * - Localization Verification: Highly useful for ensuring the dynamic
 * 'map' -> 'base_link' broadcast (from shuttle_tf_node) is healthy.
 *
 * Usage:
 * Compiled automatically as a pluginlib export. In RViz 2, navigate to
 * Panels -> Add New Panel -> nift_rviz_plugins -> TfMonitorPanel.
 */

#ifndef NIFT_RVIZ_PLUGINS_TF_MONITOR_PANEL_HPP
#define NIFT_RVIZ_PLUGINS_TF_MONITOR_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <memory>

namespace nift_rviz_plugins
{

class TfMonitorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TfMonitorPanel(QWidget * parent = nullptr);
  ~TfMonitorPanel() override;

private slots:
  void updateTfDisplay();

private:
  QLabel * tf_label_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace nift_rviz_plugins

#endif // NIFT_RVIZ_PLUGINS_TF_MONITOR_PANEL_HPP
