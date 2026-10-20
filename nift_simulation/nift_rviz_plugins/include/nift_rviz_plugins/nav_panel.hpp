/*
 * File: nav_panel.hpp
 * Author: Kane Weng
 * Date: Mar 21, 2026
 *
 * Description:
 * Header file for the NiFT shuttle's custom RViz 2 navigation panel.
 * This GUI plugin provides operators with a direct interface to interact
 * with and monitor the autonomous navigation stack directly from within
 * the RViz visualization environment.
 *
 * Key Features:
 * - One-Click Activation: Start Navigation button calls /start_navigation.
 * - Manual Toggle: Enable/Disable manual mode via /toggle_manual_mode.
 * - Stop: the soft, resumable stop via /safe_stop (SAFE_STOP, then Resume).
 * - E-STOP: the latching hardware E-stop via /trigger_estop (ESTOP: restart the
 *   controller, then power-cycle the shuttle).
 * - Reply line: shows each service's reply (success or the reason it was refused).
 * - Keyboard Teleop Zone: Click-to-focus WASD area that publishes Twist
 *   to /teleop_cmd_vel. Auto-switches to MANUAL mode on first keypress.
 * - Live Telemetry Feed: Subscribes to /nav_telemetry and updates the
 *   panel display with real-time metrics.
 * - Camera Buttons: Chase / Top-down / Map / Orbit switch the 3D view to the
 *   saved view of that name in the RViz config.
 *
 * Usage:
 * Compiled automatically as a pluginlib export. In RViz 2, navigate to
 * Panels -> Add New Panel -> nift_rviz_plugins -> NavPanel.
 */

#ifndef NIFT_RVIZ_PLUGINS_NAV_PANEL_HPP
#define NIFT_RVIZ_PLUGINS_NAV_PANEL_HPP

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QElapsedTimer>

#include <functional>

namespace nift_rviz_plugins
{

class NavPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavPanel(QWidget * parent = nullptr);
  ~NavPanel() override;
  bool eventFilter(QObject * obj, QEvent * event) override;

protected Q_SLOTS:
  void onStartButtonClicked();
  void onManualButtonClicked();
  void onStopButtonClicked();
  void onEstopButtonClicked();
  void onResumeButtonClicked();
  void publishTeleop();

protected:
  void telemetryCallback(const std_msgs::msg::String::SharedPtr msg);
  void setTeleopActive(bool active);
  void updateTeleopFromKeys();
  void switchView(const QString & name);
  void applyStatus(const QString & status);
  void checkTelemetryAge();
  void showReply(const QString & service, bool ok, const QString & message);
  void callTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const QString & service, std::function<void(bool)> on_reply = nullptr);

  // ── Navigation buttons ───────────────────────────────────────────────── //
  QPushButton * start_button_;
  QPushButton * manual_button_;
  QPushButton * stop_button_;
  QPushButton * estop_button_;
  QPushButton * resume_button_;
  QLabel * reply_label_;
  QLabel * telemetry_label_;
  bool is_safe_stop_ {false};
  bool is_estop_ {false};
  QElapsedTimer telemetry_age_;     // since the last /nav_telemetry

  // ── Keyboard teleop zone ─────────────────────────────────────────────── //
  QFrame * teleop_zone_;
  QLabel * teleop_hint_label_;
  QTimer * teleop_timer_;
  QDoubleSpinBox * speed_spinbox_;
  QDoubleSpinBox * steer_spinbox_;
  float teleop_v_     {0.0f};
  float teleop_delta_ {0.0f};
  bool key_w_ {false};
  bool key_s_ {false};
  bool key_a_ {false};
  bool key_d_ {false};
  bool is_manual_mode_ {false};

  // ── ROS ──────────────────────────────────────────────────────────────── //
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr manual_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr safe_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
};

} // namespace nift_rviz_plugins

#endif // NIFT_RVIZ_PLUGINS_NAV_PANEL_HPP
