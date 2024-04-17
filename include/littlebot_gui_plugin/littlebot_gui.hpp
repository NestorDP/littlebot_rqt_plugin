#pragma once

#include "ui_littlebot_gui.h"

namespace littlebot_gui_plugin
{

class LittlebotGui : public QDialog
{
    Q_OBJECT

public:
    explicit LittlebotGui(QWidget *parent = nullptr);

    ~LittlebotGui() override = default;

signals:
    void sendCommand(const std::string & command);

private:
    Ui::LittlebotGui ui_;

    void ButtonClicked();
};

}  // namespace littlebot_gui_plugin