#pragma once

/**
 * To more information about the serial library used,
 * please visit: https://github.com/NestorDP/cppserial
 */
#include "libserial/serial.hpp"

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
    void littlebotStatus();

public slots:
    void littlebotCommand(const std::string &text);

private:
    Ui::LittlebotGui ui_;

    void getStatusButtonClicked();

    void sendCommandButtonClicked();

    libserial::Ports devices_;
};

}  // namespace littlebot_rqt_plugin