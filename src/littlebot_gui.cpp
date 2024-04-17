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
    std::cout << "Button clicked" << std::endl;
}

}  // namespace littlebot_gui_plugin