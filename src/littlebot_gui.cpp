#include <iostream>

#include "littlebot_gui_plugin/littlebot_gui.hpp"

namespace littlebot_gui_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
    : QDialog(parent)
{
    ui_.setupUi(this);
    connect(ui_.pushButton, &QPushButton::clicked, this, &LittlebotGui::ButtonClicked);
}

void LittlebotGui::ButtonClicked()
{
    sendCommand();
    std::cout << "Button clicked" << std::endl;
}

void LittlebotGui::writeText(const std::string &text)
{
    ui_.textBrowser->append(QString::fromStdString(text));
}

}  // namespace littlebot_gui_plugin