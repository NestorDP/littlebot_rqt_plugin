

#include <pluginlib/class_list_macros.hpp>

#include <QTimer>

#include "littlebot_rqt_plugin/littlebot_rqt_plugin.hpp"

namespace littlebot_rqt_plugin
{
LittlebotRqtPlugin::LittlebotRqtPlugin()
: rqt_gui_cpp::Plugin(), gui_(new LittlebotGui()), comm_(new LittlebotComm())
{
  gui_->setObjectName("LittlebotGui");
  comm_->setObjectName("LittlebotComm");

  node_ = std::make_shared<rclcpp::Node>("littlebot_rqt_plugin");

  QTimer * ros_spin_timer = new QTimer(this);
  connect(ros_spin_timer, &QTimer::timeout, this, &LittlebotRqtPlugin::handleSpinOnTimer);
  ros_spin_timer->start(500);

  connect(gui_, &LittlebotGui::connectHardware, comm_, &LittlebotComm::connectHardware);
  connect(gui_, &LittlebotGui::disconnectHardware, comm_, &LittlebotComm::disconnectHardware);
  connect(gui_, &LittlebotGui::sendVelocitiesCommand, comm_, &LittlebotComm::receiveVelocitiesCommand);
  connect(gui_, &LittlebotGui::startCapture, comm_, &LittlebotComm::startTimer);
  connect(gui_, &LittlebotGui::stopCapture, comm_, &LittlebotComm::stopTimer);

  connect(comm_, &LittlebotComm::errorOccurred, gui_, &LittlebotGui::showError);
  connect(comm_, &LittlebotComm::connectionStatus, gui_, &LittlebotGui::updateWidgetsWithConnectionState);
  connect(comm_, &LittlebotComm::sendDataStatus, gui_, &LittlebotGui::receiveDataStatus);

  createPublisher();
  createSubscriber();
}

void LittlebotRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context.addWidget(gui_);
}

void LittlebotRqtPlugin::handleSpinOnTimer()
{
  rclcpp::spin_some(node_);
}

void LittlebotRqtPlugin::shutdownPlugin()
{
  // clean up rclcpp node
  node_.reset();

  // delete owned Qt objects to ensure nothing created by this plugin
  // remains on the heap when the plugin library is unloaded.
  if (comm_) {
    delete comm_;
    comm_ = nullptr;
  }
  if (gui_) {
    delete gui_;
    gui_ = nullptr;
  }
}

void LittlebotRqtPlugin::littlebotStatus()
{
  std_msgs::msg::String msg;
  msg.data = "Hello, world!";
  publisher_->publish(msg);
}

void LittlebotRqtPlugin::createPublisher()
{
  connect(gui_, &LittlebotGui::littlebotStatus, this, &LittlebotRqtPlugin::littlebotStatus);
  publisher_ = node_->create_publisher<std_msgs::msg::String>("littlebot_command", 10);
}

void LittlebotRqtPlugin::createSubscriber()
{
  connect(this, &LittlebotRqtPlugin::littlebotCommand, gui_, &LittlebotGui::littlebotCommand);
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