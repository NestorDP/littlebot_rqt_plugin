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

#pragma once

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_rqt_plugin/littlebot_gui.hpp"

namespace littlebot_rqt_plugin
{
class LittlebotRqtPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  LittlebotRqtPlugin();

  ~LittlebotRqtPlugin() override;

signals:
  void littlebotCommand(const std::string & text);

private:
  void initPlugin(qt_gui_cpp::PluginContext & context) override;

  void handleSpinOnTimer();

  void shutdownPlugin();

  void littlebotStatus();

  void createPublisher();

  void createSubscriber();

  LittlebotGui * gui_;

  LittlebotComm * comm_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  QTimer *ros_spin_timer_{nullptr};
};
}  // namespace littlebot_rqt_plugin
