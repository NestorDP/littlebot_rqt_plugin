// @ Copyright 2025-2026 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include <QTimer>

#include <pluginlib/class_list_macros.hpp>

#include "littlebot_rqt_plugin/littlebot_rqt_plugin.hpp"

namespace littlebot_rqt_plugin
{
LittlebotRqtPlugin::LittlebotRqtPlugin()
: rqt_gui_cpp::Plugin(),
  gui_(new LittlebotGui()),
  comm_(new LittlebotComm())
{
  gui_->setObjectName("LittlebotGui");
  comm_->setObjectName("LittlebotComm");

  node_ = std::make_shared<rclcpp::Node>("littlebot_rqt_plugin");

  QTimer *ros_spin_timer = new QTimer(this);
  connect(ros_spin_timer, &QTimer::timeout, this,
          &LittlebotRqtPlugin::handleSpinOnTimer);
  ros_spin_timer->start(500);

  connect(gui_, &LittlebotGui::connectHardware, comm_,
          &LittlebotComm::connectHardware);
  connect(gui_, &LittlebotGui::disconnectHardware, comm_,
          &LittlebotComm::disconnectHardware);
  connect(gui_, &LittlebotGui::sendVelocitiesCommand, comm_,
          &LittlebotComm::velocitiesCommand);
  connect(gui_, &LittlebotGui::startCapture, comm_,
          &LittlebotComm::startTimer);
  connect(gui_, &LittlebotGui::stopCapture, comm_,
          &LittlebotComm::stopTimer);
  connect(gui_, &LittlebotGui::requestDataStatus, comm_,
          &LittlebotComm::updateStatusFromHardware);

  connect(comm_, &LittlebotComm::errorOccurred, gui_,
          &LittlebotGui::showError);
  connect(comm_, &LittlebotComm::connectionStatus, gui_,
          &LittlebotGui::updateWidgetsWithConnectionState);
  connect(comm_, &LittlebotComm::dataStatus, gui_,
          &LittlebotGui::updateWidgetsWithDataStatus);
  connect(comm_, &LittlebotComm::protocolMessage, gui_,
          &LittlebotGui::printProtocolMessage);

  createPublisher();
  createSubscriber();
}

void LittlebotRqtPlugin::initPlugin(qt_gui_cpp::PluginContext & context)
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

  if (comm_) {
    comm_->disconnect();
    comm_->setParent(nullptr);
    comm_->deleteLater();
    comm_ = nullptr;
  }
  if (gui_) {
    gui_->disconnect();
    gui_->close();
    gui_->setParent(nullptr);
    gui_->deleteLater();
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
}  // namespace littlebot_rqt_plugin


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

PLUGINLIB_EXPORT_CLASS(littlebot_rqt_plugin::LittlebotRqtPlugin,
                       rqt_gui_cpp::Plugin)
