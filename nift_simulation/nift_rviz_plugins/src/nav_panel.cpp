/*
 * File: nav_panel.cpp
 * Author: Kane Weng
 * Date: Apr 8, 2026
 *
 * Description:
 * Implementation of the NavPanel RViz 2 plugin. Handles Qt GUI initialization,
 * ROS 2 node creation, and threading logic required to safely bridge async
 * ROS callbacks with the synchronous Qt UI thread.
 *
 * Key Features:
 * - Qt/ROS Integration: GUI updates via QMetaObject::invokeMethod to prevent
 *   cross-thread crashes when receiving ROS telemetry messages.
 * - Service Clients: start_navigation, toggle_manual_mode, safe_stop, trigger_estop,
 *   resume. Each reply (success, or why it was refused) is shown on the reply line.
 * - Mode from /nav_telemetry's STATUS line, parsed as "STATUS: value": SAFE_STOP shows
 *   Resume, ESTOP freezes everything but E-STOP. Both views end when telemetry stops
 *   for 2 s (the controller was restarted or is gone), so the panel never stays frozen.
 * - Keyboard Teleop: Click-to-focus QFrame captures WASD/arrow keys and
 *   publishes Twist at 10 Hz to /teleop_cmd_vel. Auto-enables MANUAL mode
 *   on first keypress if not already active.
 * - Camera buttons: switch the 3D view to one of the config's saved views
 *   (Chase / Top-down / Map / Orbit in navigation.rviz) through RViz's view
 *   manager. Pure RViz; nothing is published.
 * - DPI-independent styling: no pixel font sizes. Text inherits the panel font
 *   (in points, which Qt scales by the screen's Xft.dpi) and padding is in em,
 *   so the panel looks the same on a normal and a HiDPI screen. RViz does not
 *   enable Qt's HiDPI scaling, so a px size stays tiny on a 4K laptop.
 *
 * Usage:
 * Compiled as part of the nift_rviz_plugins pluginlib library.
 */

#include "nift_rviz_plugins/nav_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <QKeyEvent>
#include <QTimer>
#include <QElapsedTimer>
#include <QVector>
#include <QPair>
#include <algorithm>

namespace nift_rviz_plugins
{

namespace
{

// Push-button style sheet. Padding in em (font height) so it scales with the font.
QString buttonStyle(const QString & bg, const QString & fg = "white")
{
  return QString(
    "QPushButton { background-color: %1; color: %2; font-weight: bold;"
    " padding: 0.4em; }").arg(bg, fg);
}

// The manual toggle is disabled in SAFE_STOP; without a :disabled rule the style
// sheet made it look clickable.
const QString kDimWhenDisabled =
  "QPushButton:disabled { background-color: #4d5656; color: #95a5a6; }";

// trajectory_controller publishes /nav_telemetry at 10 Hz in SAFE_STOP and ESTOP. Silence this long
// means it was restarted (it starts in IDLE and sends nothing) or is gone.
const qint64 kStaleTelemetryMs = 2000;

// The STATUS value of a /nav_telemetry text ("STATUS:    SAFE_STOP" -> "SAFE_STOP"), "" if none.
// Splits each line at its first colon and trims, so the spacing of the line doesn't matter.
QString statusOf(const QString & text)
{
  for (const QString & line : text.split('\n', Qt::SkipEmptyParts)) {
    const int colon = line.indexOf(':');
    if (colon > 0 && line.left(colon).trimmed() == "STATUS") {
      return line.mid(colon + 1).trimmed();
    }
  }
  return QString();
}

// Camera buttons: neutral, lighter than the vehicle controls above them.
const QString kViewButtonStyle =
  "QPushButton { background-color: #34495e; color: #ecf0f1; padding: 0.25em 0.5em; }";

// Camera buttons -> saved view names in the RViz config (navigation.rviz, Views: Saved)
const char * const kViewNames[] = {"Chase", "Top-down", "Map", "Orbit"};

// Teleop zone frame. ID selector so the border does not cascade onto the hint label.
QString teleopZoneStyle(const QString & bg, const QString & border)
{
  return QString(
    "QFrame#teleopZone { background-color: %1; border: 0.12em solid %2;"
    " border-radius: 0.3em; }").arg(bg, border);
}

// /nav_telemetry is "Key:   value" lines (trajectory_controller._publish_telemetry).
// Laid out as a two-column key/value table so the box is used across its full width
// at half the height. Text in any other shape is returned unchanged (shown as before).
QString telemetryHtml(const QString & text)
{
  QVector<QPair<QString, QString>> items;
  for (const QString & line : text.split('\n', Qt::SkipEmptyParts)) {
    const int colon = line.indexOf(':');
    if (colon <= 0) {
      return text;
    }
    items.append({line.left(colon).trimmed(), line.mid(colon + 1).trimmed()});
  }
  if (items.isEmpty()) {
    return text;
  }

  // Two halves of the box, each a key/value table: first half of the lines left, rest right
  const int rows = (items.size() + 1) / 2;
  QString html = "<table width='100%' cellspacing='0' cellpadding='0'><tr>";
  for (int c = 0; c < 2; ++c) {
    html += "<td width='50%' valign='top'><table cellspacing='0' cellpadding='1'>";
    for (int i = c * rows; i < std::min<int>((c + 1) * rows, items.size()); ++i) {
      // STATUS value coloured by mode so it reads at a glance
      QString value_color = "#ecf0f1";
      if (items[i].first == "STATUS") {
        const QString & s = items[i].second;
        value_color = s == "NAVIGATING" ? "#5dade2" :
          s == "MANUAL" ? "#f0a35e" :
          s == "SAFE_STOP" ? "#fbbf24" :
          s == "ESTOP" ? "#ff6b5b" : "#ecf0f1";
      }
      html += QString(
        "<tr><td style='color: #95a5a6;'>%1&nbsp;&nbsp;</td>"
        "<td style='color: %3; font-family: monospace;'>%2</td></tr>")
        .arg(items[i].first, items[i].second, value_color);
    }
    html += "</table></td>";
  }
  return html + "</tr></table>";
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────────── //
//  Constructor                                                                 //
// ──────────────────────────────────────────────────────────────────────────── //

NavPanel::NavPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;

  // ── Row 1: Start Navigation ───────────────────────────────────────────── //
  start_button_ = new QPushButton("Start Navigation");
  start_button_->setStyleSheet(buttonStyle("#4e2ecc"));
  layout->addWidget(start_button_);

  // ── Row 2: Manual toggle + Stop + E-STOP side by side ─────────────────── //
  QHBoxLayout * ctrl_row = new QHBoxLayout;

  manual_button_ = new QPushButton("Enable Manual →");
  manual_button_->setStyleSheet(buttonStyle("#27ae60") + kDimWhenDisabled);

  // Stop: the soft stop (/safe_stop -> SAFE_STOP, then Resume). E-STOP latches until a restart
  stop_button_ = new QPushButton("■  Stop");
  stop_button_->setStyleSheet(buttonStyle("#5d6d7e") + kDimWhenDisabled);
  stop_button_->setToolTip("Safe stop: brake and hold, then Resume (/safe_stop)");

  estop_button_ = new QPushButton("⚠  E-STOP");
  estop_button_->setStyleSheet(buttonStyle("#c0392b"));
  estop_button_->setToolTip(
    "Hardware E-stop, latched: restart the controller, then power-cycle the shuttle "
    "(/trigger_estop)");

  ctrl_row->addWidget(manual_button_);
  ctrl_row->addWidget(stop_button_);
  ctrl_row->addWidget(estop_button_);
  layout->addLayout(ctrl_row);

  // ── Row 2b: Resume (only visible in SAFE_STOP) ────────────────────────── //
  resume_button_ = new QPushButton("▶  Resume Navigation");
  resume_button_->setStyleSheet(buttonStyle("#d97706"));
  resume_button_->setVisible(false);
  layout->addWidget(resume_button_);

  // ── Row 2c: the last service reply (hidden until the first one) ──────── //
  reply_label_ = new QLabel;
  reply_label_->setTextFormat(Qt::PlainText);
  reply_label_->setWordWrap(true);
  reply_label_->setVisible(false);
  layout->addWidget(reply_label_);

  // ── Row 3: Telemetry display (two-column table, see telemetryHtml) ───── //
  telemetry_label_ = new QLabel("Waiting for telemetry...");
  telemetry_label_->setStyleSheet(
    "background-color: #2c3e50; "
    "color: #ecf0f1; "
    "padding: 0.35em 0.6em; "
    "border-radius: 0.25em;");
  layout->addWidget(telemetry_label_);

  // ── Row 4: Speed / steer limits ──────────────────────────────────────── //
  QString spinbox_style =
    "QDoubleSpinBox {"
    "  background-color: #2c3e50; color: #ecf0f1;"
    "  border: 1px solid #566573; border-radius: 0.2em;"
    "  padding: 0.15em 0.3em; font-family: monospace; }";

  QHBoxLayout * limits_row = new QHBoxLayout;

  // Labels use the theme's text colour: the old light grey was unreadable on RViz's grey panel
  QLabel * speed_label = new QLabel("Speed:");
  speed_spinbox_ = new QDoubleSpinBox;
  speed_spinbox_->setRange(0.1, 3.0);
  speed_spinbox_->setSingleStep(0.1);
  speed_spinbox_->setValue(1.0);
  speed_spinbox_->setSuffix(" m/s");
  speed_spinbox_->setStyleSheet(spinbox_style);
  speed_spinbox_->setFocusPolicy(Qt::ClickFocus);   // don't steal arrow keys from teleop zone

  // Up to the shuttle's full lock: create_command_frame clips the CAN steering to it
  // (nift_common.can_protocol.STEER_MAX_DEG) and logs a warning every frame above it
  const double max_steer_deg = 15.0;   // owner: nift_common/vehicle.yaml max_steering_deg (C++ copy, tested)
  QLabel * steer_label = new QLabel("Steer:");
  steer_spinbox_ = new QDoubleSpinBox;
  steer_spinbox_->setRange(3.0, max_steer_deg);
  steer_spinbox_->setSingleStep(1.0);
  steer_spinbox_->setValue(max_steer_deg);
  steer_spinbox_->setSuffix(" °");
  steer_spinbox_->setStyleSheet(spinbox_style);
  steer_spinbox_->setFocusPolicy(Qt::ClickFocus);

  limits_row->addWidget(speed_label);
  limits_row->addWidget(speed_spinbox_);
  limits_row->addSpacing(fontMetrics().height());   // one line height: scales with the font
  limits_row->addWidget(steer_label);
  limits_row->addWidget(steer_spinbox_);
  limits_row->addStretch();
  layout->addLayout(limits_row);

  // ── Row 5: Keyboard teleop zone ───────────────────────────────────────── //
  teleop_zone_ = new QFrame;
  teleop_zone_->setObjectName("teleopZone");
  teleop_zone_->setFocusPolicy(Qt::ClickFocus);
  teleop_zone_->installEventFilter(this);

  QVBoxLayout * teleop_layout = new QVBoxLayout(teleop_zone_);
  teleop_hint_label_ = new QLabel(
    "🎮  Click to enable keyboard teleop\n"
    "WASD / Arrows · Space = stop");
  teleop_hint_label_->setAlignment(Qt::AlignCenter);
  teleop_hint_label_->setStyleSheet("font-family: monospace; color: #95a5a6;");
  teleop_layout->addWidget(teleop_hint_label_);

  teleop_zone_->setStyleSheet(teleopZoneStyle("#1e2a2e", "#566573"));
  layout->addWidget(teleop_zone_);

  // ── Row 6: Camera views (saved views of the RViz config, see switchView) ─ //
  QHBoxLayout * view_row = new QHBoxLayout;
  view_row->addWidget(new QLabel("View:"));
  for (const char * name : kViewNames) {
    QPushButton * button = new QPushButton(name);
    button->setStyleSheet(kViewButtonStyle);
    button->setToolTip(QString("Camera: the saved view \"%1\" (Views panel)").arg(name));
    button->setFocusPolicy(Qt::NoFocus);   // never takes keyboard focus: teleop stays active
    connect(
      button, &QPushButton::clicked, this,
      [this, view = QString(name)]() {switchView(view);});
    view_row->addWidget(button);
  }
  layout->addLayout(view_row);

  // Spare height goes below the controls instead of stretching the telemetry box
  layout->addStretch();

  setLayout(layout);

  // ── Signal connections ────────────────────────────────────────────────── //
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
  connect(manual_button_, SIGNAL(clicked()), this, SLOT(onManualButtonClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  connect(estop_button_, SIGNAL(clicked()), this, SLOT(onEstopButtonClicked()));
  connect(resume_button_, SIGNAL(clicked()), this, SLOT(onResumeButtonClicked()));

  // ── ROS node + clients + publisher ───────────────────────────────────── //
  node_ = rclcpp::Node::make_shared("rviz_nav_panel_node");
  nav_client_ = node_->create_client<std_srvs::srv::Trigger>("/start_navigation");
  manual_client_ = node_->create_client<std_srvs::srv::Trigger>("/toggle_manual_mode");
  safe_stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/safe_stop");
  estop_client_ = node_->create_client<std_srvs::srv::Trigger>("/trigger_estop");
  resume_client_ = node_->create_client<std_srvs::srv::Trigger>("/resume");
  teleop_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 10);

  telemetry_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/nav_telemetry", 10,
    std::bind(&NavPanel::telemetryCallback, this, std::placeholders::_1));

  // ── Spin timer: drives ROS callbacks at 50 ms ─────────────────────────── //
  QTimer * spin_timer = new QTimer(this);
  connect(
    spin_timer, &QTimer::timeout, this, [this]() {
      // On Ctrl-C rclcpp shuts the context down before Qt stops this timer; spinning then
      // throws RCLError ("context is not valid") and aborts RViz.
      if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
      }
    });
  spin_timer->start(50);

  // ── Telemetry age: leaves a stop view when the controller goes silent ─── //
  QTimer * age_timer = new QTimer(this);
  connect(age_timer, &QTimer::timeout, this, &NavPanel::checkTelemetryAge);
  age_timer->start(500);

  // ── Teleop publish timer (started/stopped with zone focus) ───────────── //
  teleop_timer_ = new QTimer(this);
  connect(teleop_timer_, &QTimer::timeout, this, &NavPanel::publishTeleop);
}

NavPanel::~NavPanel() {}

// ──────────────────────────────────────────────────────────────────────────── //
//  ROS callbacks                                                               //
// ──────────────────────────────────────────────────────────────────────────── //

void NavPanel::telemetryCallback(const std_msgs::msg::String::SharedPtr msg)
{
  QString text = QString::fromStdString(msg->data);
  QMetaObject::invokeMethod(
    telemetry_label_, "setText",
    Qt::QueuedConnection, Q_ARG(QString, telemetryHtml(text)));

  // Mode detection reads the raw message, not the formatted label text
  const QString status = statusOf(text);
  QMetaObject::invokeMethod(
    this, [this, status]() {
      telemetry_age_.restart();
      applyStatus(status);
    }, Qt::QueuedConnection);
}

void NavPanel::applyStatus(const QString & status)
{
  is_manual_mode_ = status == "MANUAL";
  is_safe_stop_ = status == "SAFE_STOP";
  is_estop_ = status == "ESTOP";

  // ── ESTOP: latched. Everything but E-STOP frozen; Resume would be refused ── //
  if (is_estop_) {
    resume_button_->setVisible(false);
    start_button_->setText("E-STOPPED: restart, then power-cycle");
    start_button_->setStyleSheet(buttonStyle("#7f1d1d", "#ff6b5b"));
    start_button_->setEnabled(false);
    manual_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    teleop_zone_->setEnabled(false);   // drops its focus: teleop stops with a zero Twist
    return;
  }
  teleop_zone_->setEnabled(true);

  // ── SAFE_STOP: show resume, freeze other controls ─────────────────── //
  if (is_safe_stop_) {
    resume_button_->setVisible(true);
    start_button_->setText("Vehicle in Safe Stop");
    start_button_->setStyleSheet(buttonStyle("#78350f", "#fbbf24"));
    start_button_->setEnabled(false);
    manual_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    return;
  }

  resume_button_->setVisible(false);
  manual_button_->setEnabled(true);
  stop_button_->setEnabled(true);

  if (is_manual_mode_) {
    manual_button_->setText("← Disable Manual");
    manual_button_->setStyleSheet(buttonStyle("#e67e22") + kDimWhenDisabled);

    start_button_->setText("Navigation Disabled in Manual Mode");
    start_button_->setStyleSheet(buttonStyle("#3498db"));
    start_button_->setEnabled(false);
  } else {
    manual_button_->setText("Enable Manual →");
    manual_button_->setStyleSheet(buttonStyle("#27ae60") + kDimWhenDisabled);
  }

  if (status == "IDLE") {
    start_button_->setText("Start Navigation");
    start_button_->setStyleSheet(buttonStyle("#4e2ecc"));
    start_button_->setEnabled(true);
  }
}

void NavPanel::checkTelemetryAge()
{
  // A stop view ends only with new telemetry, and a restarted controller sends none in IDLE.
  // So silence in a stop view means the controller was restarted or is gone: leave the view.
  if (!(is_safe_stop_ || is_estop_) || !telemetry_age_.isValid() ||
    telemetry_age_.elapsed() < kStaleTelemetryMs)
  {
    return;
  }
  const bool was_estop = is_estop_;
  applyStatus("IDLE");   // a restarted controller starts in IDLE
  showReply(
    "/nav_telemetry", false, was_estop ?
    "No telemetry for 2 s: controller restarted? Power-cycle the shuttle before driving." :
    "No telemetry for 2 s: controller restarted or gone?");
}

void NavPanel::showReply(const QString & service, bool ok, const QString & message)
{
  reply_label_->setText(QString("%1 %2: %3").arg(ok ? "✔" : "✖", service, message));
  reply_label_->setStyleSheet(
    QString(
      "background-color: #2c3e50; color: %1; padding: 0.25em 0.6em; border-radius: 0.25em;")
    .arg(ok ? "#2ecc71" : "#fbbf24"));
  reply_label_->setVisible(true);
}

void NavPanel::callTrigger(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const QString & service, std::function<void(bool)> on_reply)
{
  if (!client->service_is_ready()) {
    showReply(service, false, "service not available (is trajectory_controller running?)");
    if (on_reply) {
      on_reply(false);
    }
    return;
  }
  using Future = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, service, on_reply](Future future) {
      const auto reply = future.get();
      const bool ok = reply->success;
      const QString message = QString::fromStdString(reply->message);
      QMetaObject::invokeMethod(
        this, [this, service, ok, message, on_reply]() {
          showReply(service, ok, message);
          if (on_reply) {
            on_reply(ok);
          }
        }, Qt::QueuedConnection);
    });
}

// ──────────────────────────────────────────────────────────────────────────── //
//  Button slots                                                                //
// ──────────────────────────────────────────────────────────────────────────── //

void NavPanel::onStartButtonClicked()
{
  if (!nav_client_->wait_for_service(std::chrono::seconds(1))) {
    start_button_->setText("Service Not Found!");
    start_button_->setStyleSheet(buttonStyle("#e74c3c"));
    showReply(
      "/start_navigation", false,
      "service not available (is trajectory_controller running?)");
    return;
  }
  start_button_->setText("Starting...");
  start_button_->setStyleSheet(buttonStyle("#3498db"));
  start_button_->setEnabled(false);
  callTrigger(
    nav_client_, "/start_navigation", [this](bool ok) {
      if (ok) {
        start_button_->setText("Navigation Running");
      } else if (!is_safe_stop_ && !is_estop_ && !is_manual_mode_) {
        // Refused (the reply line says why): the controller stays in IDLE and sends no telemetry
        start_button_->setText("Start Navigation");
        start_button_->setStyleSheet(buttonStyle("#4e2ecc"));
        start_button_->setEnabled(true);
      }
    });
}

void NavPanel::onManualButtonClicked()
{
  callTrigger(manual_client_, "/toggle_manual_mode");
}

void NavPanel::onStopButtonClicked()
{
  callTrigger(safe_stop_client_, "/safe_stop");
  // telemetryCallback shows Resume when SAFE_STOP telemetry arrives
}

void NavPanel::onEstopButtonClicked()
{
  callTrigger(estop_client_, "/trigger_estop");
  // telemetryCallback freezes the panel when ESTOP telemetry arrives
}

void NavPanel::onResumeButtonClicked()
{
  callTrigger(resume_client_, "/resume");
  // telemetryCallback will hide resume_button_ and re-enable controls when IDLE telemetry arrives
}

// ──────────────────────────────────────────────────────────────────────────── //
//  Camera views                                                                //
// ──────────────────────────────────────────────────────────────────────────── //

void NavPanel::switchView(const QString & name)
{
  rviz_common::DisplayContext * context = getDisplayContext();
  if (context == nullptr) {
    return;   // panel not initialised by RViz yet
  }
  rviz_common::ViewManager * views = context->getViewManager();
  for (int i = 0; i < views->getNumViews(); ++i) {
    rviz_common::ViewController * view = views->getViewAt(i);
    if (view->getName() == name) {
      views->setCurrentFrom(view);   // the current view becomes a copy; the saved one is unchanged
      return;
    }
  }
  RCLCPP_WARN(
    node_->get_logger(), "No saved view named '%s' in this RViz config (Views panel)",
    name.toStdString().c_str());
}

// ──────────────────────────────────────────────────────────────────────────── //
//  Teleop                                                                      //
// ──────────────────────────────────────────────────────────────────────────── //

void NavPanel::setTeleopActive(bool active)
{
  if (active) {
    teleop_zone_->setStyleSheet(teleopZoneStyle("#1a2a1a", "#2ecc71"));
    teleop_hint_label_->setStyleSheet("font-family: monospace; color: #2ecc71;");
    teleop_hint_label_->setText(
      "🟢  TELEOP ACTIVE — WASD / Arrows · Space = stop\n"
      "v: 0.0 m/s   δ: 0.0°");
    teleop_timer_->start(100);   // 10 Hz
  } else {
    key_w_ = key_s_ = key_a_ = key_d_ = false;
    teleop_v_ = teleop_delta_ = 0.0f;
    teleop_timer_->stop();

    // Publish one zero Twist so the vehicle stops immediately
    teleop_pub_->publish(geometry_msgs::msg::Twist{});

    teleop_zone_->setStyleSheet(teleopZoneStyle("#1e2a2e", "#566573"));
    teleop_hint_label_->setStyleSheet("font-family: monospace; color: #95a5a6;");
    teleop_hint_label_->setText(
      "🎮  Click to enable keyboard teleop\n"
      "WASD / Arrows · Space = stop");
  }
}

void NavPanel::updateTeleopFromKeys()
{
  float lin = static_cast<float>(speed_spinbox_->value());
  float ang = static_cast<float>(steer_spinbox_->value() * M_PI / 180.0);
  teleop_v_ = (key_w_ ? lin : 0.0f) - (key_s_ ? lin : 0.0f);

  // A = wheels left (+delta, ROS convention), D = wheels right, in either gear, like a steering wheel:
  // reversing with the wheels left swings the back to the left, so S+A = back-left and S+D = back-right.
  teleop_delta_ = (key_a_ ? ang : 0.0f) - (key_d_ ? ang : 0.0f);
}

void NavPanel::publishTeleop()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = teleop_v_;
  msg.angular.z = teleop_delta_;
  teleop_pub_->publish(msg);

  teleop_hint_label_->setText(
    QString(
      "🟢  TELEOP ACTIVE — WASD / Arrows · Space = stop\n"
      "v: %1 m/s   δ: %2°")
    .arg(static_cast<double>(teleop_v_), 0, 'f', 1)
    .arg(static_cast<double>(teleop_delta_) * 180.0 / M_PI, 0, 'f', 1));
}

// ──────────────────────────────────────────────────────────────────────────── //
//  Event filter — keyboard capture for the teleop zone                        //
// ──────────────────────────────────────────────────────────────────────────── //

bool NavPanel::eventFilter(QObject * obj, QEvent * event)
{
  if (obj != teleop_zone_) {
    return QObject::eventFilter(obj, event);
  }

  switch (event->type()) {

    case QEvent::FocusIn:
      setTeleopActive(true);
      return false;

    case QEvent::FocusOut:
      setTeleopActive(false);
      return false;

    case QEvent::KeyPress: {
        QKeyEvent * ke = static_cast<QKeyEvent *>(event);
        if (ke->isAutoRepeat()) {return true;}

        bool changed = true;
        switch (ke->key()) {
          case Qt::Key_W: case Qt::Key_Up:    key_w_ = true;  break;
          case Qt::Key_S: case Qt::Key_Down:  key_s_ = true;  break;
          case Qt::Key_A: case Qt::Key_Left:  key_a_ = true;  break;
          case Qt::Key_D: case Qt::Key_Right: key_d_ = true;  break;
          case Qt::Key_Space:
            key_w_ = key_s_ = key_a_ = key_d_ = false;
            break;
          default: changed = false; break;
        }

        if (changed) {
          // Auto-switch to MANUAL on first keypress if not already there
          if (!is_manual_mode_ && manual_client_->service_is_ready()) {
            callTrigger(manual_client_, "/toggle_manual_mode");
          }
          updateTeleopFromKeys();
        }
        return true;
      }

    case QEvent::KeyRelease: {
        QKeyEvent * ke = static_cast<QKeyEvent *>(event);
        if (ke->isAutoRepeat()) {return true;}

        switch (ke->key()) {
          case Qt::Key_W: case Qt::Key_Up:    key_w_ = false; break;
          case Qt::Key_S: case Qt::Key_Down:  key_s_ = false; break;
          case Qt::Key_A: case Qt::Key_Left:  key_a_ = false; break;
          case Qt::Key_D: case Qt::Key_Right: key_d_ = false; break;
          default: break;
        }
        updateTeleopFromKeys();
        return true;
      }

    default:
      break;
  }

  return QObject::eventFilter(obj, event);
}

} // namespace nift_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nift_rviz_plugins::NavPanel, rviz_common::Panel)
