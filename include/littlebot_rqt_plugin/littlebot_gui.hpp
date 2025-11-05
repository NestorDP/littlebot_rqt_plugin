// @ Copyright 2023-2025 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
/**
 * @class LittlebotGui
 * @brief GUI class for the Littlebot RQT plugin
 */
class LittlebotGui : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new Littlebot GUI object
     * @param parent Parent widget
     */
    explicit LittlebotGui(QWidget *parent = nullptr);

    /**
     * @brief Destroy the Littlebot GUI object
     */
    ~LittlebotGui() override = default;

    /**
     * @brief Update the list of available serial devices
     */
    void updateAvailableDevices();

    /**
     * @brief Update the status display in the GUI
     */
    void updateStatusDisplay();

signals:
    /**
     * @brief Signal emitted when Littlebot status is requested
     */
    void littlebotStatus();

public slots:
    /**
     * @brief Slot to handle Littlebot command input
     * @param text Command text
     */
    void littlebotCommand(const std::string &text);

private:
    /**
     * @brief Get the status of the Littlebot
     */
    void getStatus();

    /**
     * @brief Send a command to the Littlebot
     */
    void sendCommand();

    /**
     * @brief Connect to the Littlebot hardware
     */
    void connectHardware();

    /**
     * @brief UI object for the Littlebot GUI
     */
    Ui::LittlebotGui ui_;

    /**
     * @brief Available serial devices
     */
    libserial::Ports available_devices_;

    /**
     * @brief Selected device index
     */
    uint16_t selected_device_index_{0};

    /**
     * @brief Current number of available devices
     */
    int current_number_of_devices_{0};

    /**
     * @brief Shared pointer to the Littlebot driver
     */
    std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

    std::map<std::string, float> command_velocities_{
        {"left_wheel", 0.0f},
        {"right_wheel", 0.0f}
    };

    std::map<std::string, float> status_positions_{
        {"left_wheel", 0.0f},
        {"right_wheel", 0.0f}
    };

    std::map<std::string, float> status_velocities_{
        {"left_wheel", 0.0f},
        {"right_wheel", 0.0f}
    };
};

}  // namespace littlebot_rqt_plugin