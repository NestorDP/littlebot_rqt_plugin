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
  
  ~LittlebotRqtPlugin() override = default;

signals:
  void littlebotCommand(const std::string &text);

private:
  void initPlugin(qt_gui_cpp::PluginContext& context) override;

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

};
}  // namespace littlebot_rqt_plugin