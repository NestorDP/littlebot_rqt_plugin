

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

  connect(widget_, &LittlebotGui::sendCommand, this, &LittlebotGuiPlugin::sendCommand);

  publisher_ = node_->create_publisher<std_msgs::msg::String>("littlebot_command", 10);
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
}  // namespace littlebot_gui_plugin

PLUGINLIB_EXPORT_CLASS(littlebot_gui_plugin::LittlebotGuiPlugin, rqt_gui_cpp::Plugin)