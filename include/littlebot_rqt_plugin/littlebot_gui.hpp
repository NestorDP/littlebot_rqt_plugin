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

    void getStatus();

    void sendCommand();

    void connecteHardware();

    libserial::Ports available_devices_;

    uint16_t selected_device_index_{0};

    int current_number_of_devices_{0};

    std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;
};

}  // namespace littlebot_rqt_plugin