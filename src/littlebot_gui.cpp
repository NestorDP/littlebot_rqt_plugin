#include <iostream>

#include "littlebot_rqt_plugin/littlebot_gui.hpp"

namespace littlebot_rqt_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
    : QDialog(parent)
{
    ui_.setupUi(this);
    connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommandButtonClicked);
}

void LittlebotGui::sendCommandButtonClicked()
{
    littlebotStatus();
}

void LittlebotGui::littlebotCommand(const std::string &text)
{
    // ui_.textBrowser->append(QString::fromStdString(text));
}

}  // namespace littlebot_rqt_plugin