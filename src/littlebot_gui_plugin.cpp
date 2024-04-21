

#include <pluginlib/class_list_macros.hpp>

#include <QTimer>

#include "littlebot_gui_plugin/littlebot_gui_plugin.hpp"

namespace littlebot_gui_plugin
{
LittlebotGuiPlugin::LittlebotGuiPlugin()
: rqt_gui_cpp::Plugin(), widget_(new LittlebotGui())
{
  widget_->setObjectName("LittlebotGui");

  node_ = std::make_shared<rclcpp::Node>("littlebot_gui_plugin");

  QTimer * ros_spin_timer = new QTimer(this);
  connect(ros_spin_timer, &QTimer::timeout, this, &LittlebotGuiPlugin::handleSpinOnTimer);
  ros_spin_timer->start(500);

  createPublisher();
  createSubscriber();
}

void LittlebotGuiPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context.addWidget(widget_);
}

void LittlebotGuiPlugin::handleSpinOnTimer()
{
  rclcpp::spin_some(node_);
}

void LittlebotGuiPlugin::shutdownPlugin()
{
  node_.reset();
}

void LittlebotGuiPlugin::sendCommand()
{
  std_msgs::msg::String msg;
  msg.data = "Hello, world!";
  publisher_->publish(msg);
}

void LittlebotGuiPlugin::createPublisher()
{
  connect(widget_, &LittlebotGui::sendCommand, this, &LittlebotGuiPlugin::sendCommand);
  publisher_ = node_->create_publisher<std_msgs::msg::String>("littlebot_command", 10);
}

void LittlebotGuiPlugin::createSubscriber()
{
  connect(this, &LittlebotGuiPlugin::writeText, widget_, &LittlebotGui::writeText);  
  auto message_callback = [this](std_msgs::msg::String msg) -> void {
    RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
    emit writeText(msg.data);};
  subscriber_ =
    node_->create_subscription<std_msgs::msg::String>("littlebot_text",
      10, message_callback);
}
}  // namespace littlebot_gui_plugin

PLUGINLIB_EXPORT_CLASS(littlebot_gui_plugin::LittlebotGuiPlugin, rqt_gui_cpp::Plugin)