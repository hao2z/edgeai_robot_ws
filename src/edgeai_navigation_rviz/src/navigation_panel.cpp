#include <chrono>
#include <mutex>
#include <string>
#include <QFile>
#include <QDockWidget>
#include <QFontDatabase>
#include <QMainWindow>
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QScrollBar>
#include <QTextBrowser>
#include <QTimer>
#include <QTime>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace edgeai_navigation_rviz
{
class NavigationPanel : public rviz_common::Panel
{
public:
  explicit NavigationPanel(QWidget *parent = nullptr) : rviz_common::Panel(parent)
  {
    // Native Linux fonts remain the fallback; WSL can reuse the installed CJK font.
    const QString cjk("/mnt/c/Windows/Fonts/msyh.ttc");
    if (QFile::exists(cjk)) {
      const auto families = QFontDatabase::applicationFontFamilies(QFontDatabase::addApplicationFont(cjk));
      if (!families.empty()) {setFont(QFont(families.first(), 10));}
    }
    auto *layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(displayText("机器人 1 控制", "Robot 1 control"), this));
    auto *controls = new QHBoxLayout();
    for (const auto &item : {std::make_pair(displayText("自动探索", "Autonomous exploration"), false), std::make_pair(displayText("手动接管", "Manual control"), true)}) {
      auto *button = new QPushButton(item.first, this);
      controls->addWidget(button);
      connect(button, &QPushButton::clicked, this, [this, manual = item.second]() {
        if (!mode_client_ || !mode_client_->service_is_ready()) {return;}
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = manual;
        mode_client_->async_send_request(request);
      });
    }
    auto *stop = new QPushButton(displayText("停止任务", "Stop task"), this);
    controls->addWidget(stop);
    connect(stop, &QPushButton::clicked, this, [this]() {
      if (stop_client_ && stop_client_->service_is_ready()) {
        stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
    });
    layout->addLayout(controls);
    heading_toggle_ = new QCheckBox(displayText("手动目标要求朝向", "Require manual goal heading"), this);
    heading_toggle_->setEnabled(false);
    connect(heading_toggle_, &QCheckBox::toggled, this, [this](bool checked) {
      if (!heading_client_ || !heading_client_->service_is_ready()) {return;}
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = checked;
      heading_client_->async_send_request(request);
    });
    layout->addWidget(heading_toggle_);
    status_ = new QLabel(displayText("等待导航诊断消息", "Waiting for navigation diagnostics"), this);
    status_->setWordWrap(true);
    view_ = new QTextBrowser(this);
    view_->setOpenExternalLinks(false);
    view_->setStyleSheet("QTextBrowser {background:#f5f7fa;color:#172536;}");
    view_->setMinimumWidth(300);
    auto *detailsToggle = new QCheckBox(displayText("展开导航、数据时序与 TF 详情", "Show navigation, timing and TF details"), this);
    details_ = new QTextBrowser(this);
    details_->setOpenExternalLinks(false);
    details_->hide();
    details_->setMaximumHeight(240);
    connect(detailsToggle, &QCheckBox::toggled, details_, &QWidget::setVisible);
    auto *legendToggle = new QCheckBox(displayText("显示图例与说明", "Show legend and notes"), this);
    legend_ = new QTextBrowser(this);
    legend_->setOpenExternalLinks(false);
    legend_->setMaximumHeight(220);
    legendToggle->setChecked(true);
    connect(legendToggle, &QCheckBox::toggled, legend_, &QWidget::setVisible);
    layout->addWidget(status_);
    layout->addWidget(view_, 3);
    layout->addWidget(detailsToggle);
    layout->addWidget(details_);
    layout->addWidget(legendToggle);
    layout->addWidget(legend_);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      heading_toggle_->setEnabled(heading_client_ && heading_client_->service_is_ready());
      // Keep Qt/OGRE producing damage while stationary under WSLg/Xwayland.
      // This never raises, moves or restores a hidden/minimized window.
      auto *context = getDisplayContext();
      if (context && window()->isVisible() && !window()->isMinimized()) {
        context->queueRender();
        window()->update();
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (received_) {
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now()-received_at_).count();
        status_->setText((age < 1.5 ? displayText("诊断刷新正常 · 消息延迟 %1 秒", "Diagnostics active · message age %1 s").arg(age, 0, 'f', 1) :
          displayText("诊断消息已停止更新 · %1 秒", "Diagnostics stopped updating · %1 s").arg(age, 0, 'f', 1)) +
          QString::fromUtf8(" · %1").arg(QTime::currentTime().toString("HH:mm:ss")));
        status_->setStyleSheet(age < 1.5 ? "color:#17713c;" : "color:#b3261e;");
      }
      if (pending_) {
        const int scroll = view_->verticalScrollBar()->value();
        const auto parts = QString::fromUtf8(html_.c_str()).split("<!--LEGEND-->");
        const auto mainParts = parts.first().split("<!--DETAILS-->");
        view_->setHtml(mainParts.first());
        const int detailScroll = details_->verticalScrollBar()->value();
        if (mainParts.size() > 1) {details_->setHtml(mainParts.at(1));}
        details_->verticalScrollBar()->setValue(detailScroll);
        const int legendScroll = legend_->verticalScrollBar()->value();
        if (parts.size() > 1) {legend_->setHtml(parts.at(1));}
        legend_->verticalScrollBar()->setValue(legendScroll);
        view_->verticalScrollBar()->setValue(scroll);
        pending_ = false;
      }
    });
    timer_->start(200);
  }

  void onInitialize() override
  {
    QTimer::singleShot(800, this, [this]() {
      if (auto *main = qobject_cast<QMainWindow *>(window())) {main->showMaximized();}
    });
    QTimer::singleShot(0, this, [this]() {
      auto *main = qobject_cast<QMainWindow *>(window());
      for (QWidget *widget = parentWidget(); main && widget; widget = widget->parentWidget()) {
        if (auto *dock = qobject_cast<QDockWidget *>(widget)) {
          main->addDockWidget(Qt::RightDockWidgetArea, dock);
          main->resizeDocks({dock}, {360}, Qt::Horizontal);
          break;
        }
      }
    });
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {status_->setText("RViz ROS node unavailable"); return;}
    mode_client_ = abstraction->get_raw_node()->create_client<std_srvs::srv::SetBool>(
      "/navigation_authority/set_manual");
    stop_client_ = abstraction->get_raw_node()->create_client<std_srvs::srv::Trigger>(
      "/navigation_authority/stop");
    heading_client_ = abstraction->get_raw_node()->create_client<std_srvs::srv::SetBool>(
      "/navigation_authority/set_goal_heading");
    subscription_ = abstraction->get_raw_node()->create_subscription<std_msgs::msg::String>(
      "/diagnostics/navigation_status", rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        html_ = message->data;
        received_at_ = std::chrono::steady_clock::now();
        received_ = pending_ = true;
      });
  }

private:
  static QString displayText(const char *chinese, const char *english)
  {
    return QString::fromUtf8(qgetenv("EDGEAI_LANGUAGE") == "en" ? english : chinese);
  }
  QLabel *status_;
  QTextBrowser *view_;
  QTextBrowser *details_;
  QTextBrowser *legend_;
  QTimer *timer_;
  QCheckBox *heading_toggle_;
  std::mutex mutex_;
  std::string html_;
  bool pending_{false}, received_{false};
  std::chrono::steady_clock::time_point received_at_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_client_;
};
}  // namespace edgeai_navigation_rviz
PLUGINLIB_EXPORT_CLASS(edgeai_navigation_rviz::NavigationPanel, rviz_common::Panel)
