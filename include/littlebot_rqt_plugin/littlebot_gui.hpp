#pragma once

#include "ui_littlebot_gui.h"

namespace littlebot_rqt_plugin
{
class LittlebotGui : public QDialog
{
    Q_OBJECT

public:
    explicit LittlebotGui(QWidget *parent = nullptr);

    ~LittlebotGui() override = default;

signals:
    void sendCommand();

public slots:
    void writeText(const std::string &text);

private:
    Ui::LittlebotGui ui_;

    void ButtonClicked();
};

}  // namespace littlebot_rqt_plugin