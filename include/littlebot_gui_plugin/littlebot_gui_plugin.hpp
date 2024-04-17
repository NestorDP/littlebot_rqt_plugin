#pragma once

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "littlebot_gui_plugin/littlebot_gui.hpp"

namespace littlebot_gui_plugin
{
class LittlebotGuiPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  LittlebotGuiPlugin();
  
  ~LittlebotGuiPlugin() override = default;

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void handleSpinOnTimer();

  void sendCommand();

  LittlebotGui * widget_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};
}  // namespace littlebot_gui_plugin