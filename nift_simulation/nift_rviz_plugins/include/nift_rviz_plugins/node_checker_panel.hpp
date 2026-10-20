/*
 * File: node_checker_panel.hpp
 * Author: Kane Weng
 * Date: Apr 7, 2026
 *
 * Description:
 * Header file for the Node Checker RViz 2 panel. This plugin serves as a
 * centralized health monitor for the NiFT software stack. It subscribes to
 * standard ROS 2 diagnostic messages and parses them into a human-readable
 * text display, allowing operators to see exactly which nodes are healthy,
 * warning, or in an error state.
 *
 * Key Features:
 * - Diagnostic Aggregation: Listens to the standard '/diagnostics' topic to
 * gather status reports from the control, navigation, and hardware bridges.
 * - Thread-Safe Architecture: Utilizes a std::mutex to safely hand off
 * asynchronous ROS 2 callback data to the synchronous Qt GUI thread.
 * - Detailed State Tracking: Stores both the severity level (int8_t) and the
 * string message to provide comprehensive context on failures.
 *
 * Usage:
 * Compiled automatically as a pluginlib export. In RViz 2, navigate to
 * Panels -> Add New Panel -> nift_rviz_plugins -> NodeCheckerPanel.
 */

#ifndef NIFT_RVIZ_PLUGINS_NODE_CHECKER_PANEL_HPP
#define NIFT_RVIZ_PLUGINS_NODE_CHECKER_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <QVBoxLayout>
#include <QTextEdit>
#include <QTimer>
#include <map>
#include <string>
#include <mutex>
#include <utility> // For std::pair

namespace nift_rviz_plugins
{

class NodeCheckerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NodeCheckerPanel(QWidget * parent = nullptr);
  ~NodeCheckerPanel() override;

private slots:
  void updateUi();

private:
  void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  QTextEdit * text_display_;
  QString last_html_;       // last content set, to skip identical updates

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

  // UPDATED: Now stores both the Status Level (int8_t) and the Message (std::string)
  std::map<std::string, std::pair<int8_t, std::string>> status_map_;
  std::mutex map_mutex_;
};

} // namespace nift_rviz_plugins

#endif // NIFT_RVIZ_PLUGINS_NODE_CHECKER_PANEL_HPP
