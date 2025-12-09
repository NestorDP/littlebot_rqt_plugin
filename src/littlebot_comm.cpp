#include "littlebot_rqt_plugin/littlebot_comm.hpp"

#include "littlebot_base/serial_port.hpp"

namespace littlebot_rqt_plugin
{
LittlebotComm::LittlebotComm(QObject *parent)
    : QObject(parent),
      timer_(new QTimer(this))
{
    serial_ = std::make_shared<littlebot_base::SerialPort>();
    
    connect(timer_, &QTimer::timeout, this,  
        [this]() { this->updateStatusDataFromHardware(false); });
}

void LittlebotComm::connectHardware(QString portName)
{
    try {
        littlebot_driver_ = std::make_shared<littlebot_base::LittlebotDriver>(serial_, portName.toStdString(), 115200);
    } catch (const std::exception &ex) {
        emit errorOccurred(QString::fromStdString(std::string("Connection failed: ") + ex.what()));
        return;
    }
    emit connectionStatus(true);
}

void LittlebotComm::disconnectHardware()
{
    try {
        if (littlebot_driver_) {
            serial_->close();
            littlebot_driver_.reset();
        }
    } catch (const std::exception &ex) {
        emit errorOccurred(QString::fromStdString(std::string("Disconnection failed: ") + ex.what()));
        return;
    }
    emit connectionStatus(false);
}

void LittlebotComm::receiveVelocitiesCommand(const QVector<float> &data)
{
    if (data.size() < 2) {
        emit errorOccurred("Insufficient velocity data received.");
        return;
    }

    command_velocities_["left_wheel"] = data[0];
    command_velocities_["right_wheel"] = data[1];
}

void LittlebotComm::startTimer()
{
    if (!timer_->isActive()) {
        timer_->start(kTimerInterval_ms);
    }
}

void LittlebotComm::stopTimer()
{
    if (timer_->isActive()) {
        timer_->stop();
    }
}

void LittlebotComm::updateStatusDataFromHardware(const bool debug)
{
    try {
        if (littlebot_driver_) {
            littlebot_driver_->sendData('S');
            if (littlebot_driver_->receiveData() == 'S') {
                status_velocities_ = littlebot_driver_->getStatusVelocities();
                status_positions_ = littlebot_driver_->getStatusPositions();

                QVector<float> data_status{
                    status_velocities_["left_wheel"],
                    status_velocities_["right_wheel"],
                    status_positions_["left_wheel"],
                    status_positions_["right_wheel"]
                };

                if (debug) {
                    QString message{"Message Test!"};
                    emit sendProtocolMessage(message);
                }
                emit sendDataStatus(data_status);
            } else {
                throw std::runtime_error("Failed to receive status data from hardware.");
            }
        } else {
            throw std::runtime_error("Littlebot driver not initialized.");
        }
    } catch (const std::exception &ex) {
        emit errorOccurred(QString::fromStdString(std::string("Error during status update: ") + ex.what()));
        this->stopTimer();
        return;
    }
}
}  // namespace littlebot_rqt_plugin
