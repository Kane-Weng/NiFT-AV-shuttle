/*
 * File: node_checker_panel.cpp
 * Author: Kane Weng
 * Date: Apr 7, 2026
 *
 * Description:
 * Implementation of the NodeCheckerPanel RViz 2 plugin. This file manages the
 * parsing of standard ROS 2 diagnostic arrays, applying thread-safe locks, and
 * dynamically generating an HTML table to visually represent the health state
 * of the system.
 *
 * Key Features:
 * - Dynamic HTML Formatting: Maps standard diagnostic byte levels (OK, WARN, ERROR)
 * to distinct colors and icons (Green Circle, Yellow Triangle, Red Cross) for
 * immediate visual feedback.
 * - Smart Context Display: Automatically expands failing or warning nodes to show
 * the specific error message, while keeping healthy nodes clean and compact.
 * - Compact Layout: entries flow into as many columns as the panel width allows
 * and the box is exactly as tall as its content, so the dock gives the spare
 * height to the other panels. Sizes are relative to the panel font (points,
 * scaled by the screen's Xft.dpi), never in pixels, so it reads the same on a
 * normal and a HiDPI screen.
 *
 * Usage:
 * Compiled as part of the nift_rviz_plugins pluginlib library.
 */

#include "nift_rviz_plugins/node_checker_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QString>
#include <QAbstractTextDocumentLayout>
#include <QFontMetrics>
#include <QtMath>
#include <algorithm>

namespace nift_rviz_plugins
{

NodeCheckerPanel::NodeCheckerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;

  text_display_ = new QTextEdit();
  text_display_->setReadOnly(true);
  text_display_->setStyleSheet(
    "background-color: #2c3e50; "
    "color: #ecf0f1; "
    "border: none; "
    "border-radius: 0.25em;"
  );
  // Margin in font-height units so it scales with the text
  text_display_->document()->setDocumentMargin(0.3 * text_display_->fontMetrics().height());

  // Box height follows the content (also when a warning adds a message line), so no scroll bars
  text_display_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  text_display_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  connect(
    text_display_->document()->documentLayout(),
    &QAbstractTextDocumentLayout::documentSizeChanged, this, [this](const QSizeF & size) {
      text_display_->setFixedHeight(qCeil(size.height()) + 2 * text_display_->frameWidth());
    });
  text_display_->setHtml("Waiting for /diagnostics...");
  layout->addWidget(text_display_);

  setLayout(layout);
  // Never taller than its content: the dock area hands the rest to the other panels
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  node_ = rclcpp::Node::make_shared("rviz_node_checker_panel");
  diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    std::bind(&NodeCheckerPanel::diagnosticCallback, this, std::placeholders::_1)
  );

  QTimer * timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateUi()));
  timer->start(200);
}

NodeCheckerPanel::~NodeCheckerPanel() {}

void NodeCheckerPanel::diagnosticCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  // UPDATED: Store both the level and the message
  for (const auto & status : msg->status) {
    status_map_[status.name] = {status.level, status.message};
  }
}

void NodeCheckerPanel::updateUi()
{
  if (!rclcpp::ok()) {
    return;   // shutting down: spinning a dead context throws and aborts RViz
  }
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lock(map_mutex_);

  if (status_map_.empty()) {
    return;
  }

  // The "System Health" heading is the dock title (navigation.rviz). Columns: as many as
  // the widest entry allows (at most 3), so a wide panel stays short.
  const QFontMetrics fm(text_display_->font());
  int widest = 0;
  for (const auto & entry : status_map_) {
    widest = std::max(widest, fm.horizontalAdvance("● " + QString::fromStdString(entry.first)));
  }
  const int avail = text_display_->viewport()->width() -
    qCeil(2 * text_display_->document()->documentMargin());
  const int count = static_cast<int>(status_map_.size());
  const int cols = std::clamp(avail / (widest + fm.height()), 1, std::min(3, count));
  const int per_col = (count + cols - 1) / cols;

  // One outer cell per column, each holding an (icon, name) table filled top to bottom.
  // The icon cells size to their glyph, so names line up whatever the font size.
  QString html = QString(
    "<table width='100%' cellspacing='0' cellpadding='0'><tr>"
    "<td width='%1%' valign='top'><table cellspacing='0' cellpadding='1'>")
    .arg(100 / cols);
  int index = 0;

  for (const auto & [name, data] : status_map_) {
    int8_t level = data.first;
    std::string message = data.second;

    QString color;
    QString icon;

    switch (level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        color = "#2ecc71"; // Green
        icon = "&#9679;";  // Circle
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        color = "#f1c40f"; // Yellow
        icon = "&#9650;";  // Triangle
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        color = "#e74c3c"; // Red
        icon = "&#10006;"; // Cross
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      default:
        color = "#95a5a6"; // Gray
        icon = "&#63;";    // Question mark
        break;
    }

    // Next column once this one is full
    if (index > 0 && index % per_col == 0) {
      html += QString(
        "</table></td><td width='%1%' valign='top'>"
        "<table cellspacing='0' cellpadding='1'>").arg(100 / cols);
    }
    // valign='top' so an entry with a message keeps its icon on the name line
    html += QString("<tr><td valign='top' style='color: %1;'>%2&nbsp;</td><td valign='top'>%3")
      .arg(color, icon, QString::fromStdString(name));

    // If OK, just print the name. If Warning/Error, print name AND the message below it
    // (<small> is relative to the panel font, unlike the old 11px).
    if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      html += QString("<br><small style='color: #bdc3c7;'><i>↳ %1</i></small>")
        .arg(QString::fromStdString(message));
    }
    html += "</td></tr>";
    ++index;
  }

  html += "</table></td></tr></table>";
  // Only re-set on change: setHtml every 200 ms re-laid-out the box and reset any selection
  if (html != last_html_) {
    last_html_ = html;
    text_display_->setHtml(html);
  }
}

} // namespace nift_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nift_rviz_plugins::NodeCheckerPanel, rviz_common::Panel)
