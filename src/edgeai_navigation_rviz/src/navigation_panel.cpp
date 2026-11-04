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
    auto *controls = new QHBoxLayout();
    for (const auto &item : {std::make_pair("自动探索", false), std::make_pair("手动接管", true)}) {
      auto *button = new QPushButton(QString::fromUtf8(item.first), this);
      controls->addWidget(button);
      connect(button, &QPushButton::clicked, this, [this, manual = item.second]() {
        if (!mode_client_ || !mode_client_->service_is_ready()) {return;}
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = manual;
        mode_client_->async_send_request(request);
      });
    }
    auto *stop = new QPushButton(QString::fromUtf8("停止任务"), this);
    controls->addWidget(stop);
    connect(stop, &QPushButton::clicked, this, [this]() {
      if (stop_client_ && stop_client_->service_is_ready()) {
        stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
    });
    layout->addLayout(controls);
    status_ = new QLabel(QString::fromUtf8("等待导航诊断消息"), this);
    view_ = new QTextBrowser(this);
    view_->setOpenExternalLinks(false);
    view_->setStyleSheet("QTextBrowser {background:#f5f7fa;color:#172536;}");
    view_->setMinimumWidth(300);
    auto *legendToggle = new QCheckBox(QString::fromUtf8("显示图例与说明"), this);
    legend_ = new QTextBrowser(this);
    legend_->setOpenExternalLinks(false);
    legend_->hide();
    connect(legendToggle, &QCheckBox::toggled, legend_, &QWidget::setVisible);
    layout->addWidget(status_);
    layout->addWidget(view_);
    layout->addWidget(legendToggle);
    layout->addWidget(legend_);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
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
        status_->setText((age < 1.5 ? QString::fromUtf8("诊断更新中") :
          QString::fromUtf8("诊断消息已停止更新 · %1 秒").arg(age, 0, 'f', 1)) +
          QString::fromUtf8(" · 帧 %1 · %2").arg(context ? context->getFrameCount() : 0)
            .arg(QTime::currentTime().toString("HH:mm:ss")));
      }
      if (pending_) {
        const int scroll = view_->verticalScrollBar()->value();
        const auto parts = QString::fromUtf8(html_.c_str()).split("<!--LEGEND-->");
        view_->setHtml(parts.first());
        if (parts.size() > 1) {legend_->setHtml(parts.at(1));}
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
  QLabel *status_;
  QTextBrowser *view_;
  QTextBrowser *legend_;
  QTimer *timer_;
  std::mutex mutex_;
  std::string html_;
  bool pending_{false}, received_{false};
  std::chrono::steady_clock::time_point received_at_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
};
}  // namespace edgeai_navigation_rviz
PLUGINLIB_EXPORT_CLASS(edgeai_navigation_rviz::NavigationPanel, rviz_common::Panel)
