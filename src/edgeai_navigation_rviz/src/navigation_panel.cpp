#include <chrono>
#include <mutex>
#include <string>
#include <QFile>
#include <QDockWidget>
#include <QFontDatabase>
#include <QMainWindow>
#include <QLabel>
#include <QScrollBar>
#include <QTextBrowser>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

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
    status_ = new QLabel(QString::fromUtf8("等待导航诊断消息"), this);
    view_ = new QTextBrowser(this);
    view_->setOpenExternalLinks(false);
    view_->setStyleSheet("QTextBrowser {background:#f5f7fa;color:#172536;}");
    view_->setMinimumWidth(300);
    layout->addWidget(status_);
    layout->addWidget(view_);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (received_) {
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now()-received_at_).count();
        status_->setText(age < 1.5 ? QString::fromUtf8("实时诊断 · 轮廓颜色不表示安全通过") :
          QString::fromUtf8("诊断消息已停止更新 · %1 秒").arg(age, 0, 'f', 1));
      }
      if (pending_) {
        const int scroll = view_->verticalScrollBar()->value();
        view_->setHtml(QString::fromUtf8(html_.c_str()));
        view_->verticalScrollBar()->setValue(scroll);
        pending_ = false;
      }
    });
    timer_->start(200);
  }

  void onInitialize() override
  {
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
  QTimer *timer_;
  std::mutex mutex_;
  std::string html_;
  bool pending_{false}, received_{false};
  std::chrono::steady_clock::time_point received_at_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // namespace edgeai_navigation_rviz
PLUGINLIB_EXPORT_CLASS(edgeai_navigation_rviz::NavigationPanel, rviz_common::Panel)
