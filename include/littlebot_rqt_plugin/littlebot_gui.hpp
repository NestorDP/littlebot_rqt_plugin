#pragma once

/**
 * To more information about the serial library used,
 * please visit: https://github.com/NestorDP/cppserial
 */
#include "libserial/ports.hpp"
#include "libserial/device.hpp"

#include "littlebot_base/littlebot_driver.hpp"

#include "ui_littlebot_gui.h"

namespace littlebot_rqt_plugin
{
class LittlebotGui : public QDialog
{
    Q_OBJECT

public:
    explicit LittlebotGui(QWidget *parent = nullptr);

    ~LittlebotGui() override = default;

    void updateAvailableDevices();

signals:
    void littlebotStatus();

public slots:
    void littlebotCommand(const std::string &text);

private:
    Ui::LittlebotGui ui_;

    void getStatusButtonClicked();

    void sendCommandButtonClicked();

    libserial::Ports available_devices_;

    int selected_device_index_{-1};

    int current_number_of_devices_{0};
};

}  // namespace littlebot_rqt_plugin