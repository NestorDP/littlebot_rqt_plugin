

#include <pluginlib/class_list_macros.hpp>

#include <QTimer>

#include "littlebot_rqt_plugin/littlebot_rqt_plugin.hpp"

namespace littlebot_rqt_plugin
{
LittlebotRqtPlugin::LittlebotRqtPlugin()
: rqt_gui_cpp::Plugin(), widget_(new LittlebotGui())
{
  widget_->setObjectName("LittlebotGui");

  node_ = std::make_shared<rclcpp::Node>("littlebot_rqt_plugin");

  QTimer * ros_spin_timer = new QTimer(this);
  connect(ros_spin_timer, &QTimer::timeout, this, &LittlebotRqtPlugin::handleSpinOnTimer);
  ros_spin_timer->start(500);

  createPublisher();
  createSubscriber();
}

void LittlebotRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context.addWidget(widget_);
}

void LittlebotRqtPlugin::handleSpinOnTimer()
{
  rclcpp::spin_some(node_);
}

void LittlebotRqtPlugin::shutdownPlugin()
{
  node_.reset();
}

void LittlebotRqtPlugin::littlebotStatus()
{
  std_msgs::msg::String msg;
  msg.data = "Hello, world!";
  publisher_->publish(msg);
}

void LittlebotRqtPlugin::createPublisher()
{
  connect(widget_, &LittlebotGui::littlebotStatus, this, &LittlebotRqtPlugin::littlebotStatus);
  publisher_ = node_->create_publisher<std_msgs::msg::String>("littlebot_command", 10);
}

void LittlebotRqtPlugin::createSubscriber()
{
  connect(this, &LittlebotRqtPlugin::littlebotCommand, widget_, &LittlebotGui::littlebotCommand);
  auto message_callback = [this](std_msgs::msg::String msg) -> void {
    RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
    emit littlebotCommand(msg.data);
  };
  subscriber_ =
    node_->create_subscription<std_msgs::msg::String>("littlebot_text",
      10, message_callback);
}
}  // namespace littlebot_rqt_plugin

PLUGINLIB_EXPORT_CLASS(littlebot_rqt_plugin::LittlebotRqtPlugin, rqt_gui_cpp::Plugin)