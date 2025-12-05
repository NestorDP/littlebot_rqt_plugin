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

#include "ui_littlebot_gui.h"

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <QVector>
#include <QThread> // for QThread::msleep

#include "littlebot_rqt_plugin/littlebot_comm.hpp"

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
    // void updateStatusDisplay();

    /**
     * @brief Update the curves to plot with new data
     */
    void updateCurvesToPlot();

    /**
     * @brief Update the plots with the latest curves
     */
    void updatePlots();

signals:
    /**
     * @brief Signal emitted when Littlebot status is requested
     */
    void littlebotStatus();

    void connectHardware(QString portName);

    void disconnectHardware();

    void sendVelocitiesCommand(const QVector<float> &data);

    void startCapture();

    void stopCapture();

public slots:

    void littlebotCommand(const std::string &text);

    void showError(const QString &message);

    void updateWidgetsWithConnectionState(bool connected);

    void receiveDataStatus(const QVector<float> &data);

    void updateSetpoint();

private:
    /**
     * @brief Get the status of the Littlebot
     */
    // void getStatus();

    /**
     * @brief Send a command to the Littlebot
     */
    // void setCommand();

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

    float command_velocity_setpoint_left_{0.0f};

    float command_velocity_setpoint_right_{0.0f};

    std::vector<double> command_velocity_left_{0.0f};
    
    std::vector<double> command_velocity_right_{0.0f};

    std::vector<double> status_velocity_left_{0.0f};

    std::vector<double> status_velocity_right_{0.0f};

    std::vector<double> status_position_left_{0.0f};

    std::vector<double> status_position_right_{0.0f};

    std::vector<double> plot_index_;

    bool connected_{false};

    QwtPlotCurve *wheel_velocity_curve_{nullptr};

    QwtPlotCurve *setpoint_curve_{nullptr};

    static constexpr int kMaxPoints{100};

    float setpoint_{0.0f};

    std::shared_ptr<std::vector<double>> status_velocity_left_ptr_{nullptr};
};

}  // namespace littlebot_rqt_plugin