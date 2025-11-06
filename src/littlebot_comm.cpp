#include "littlebot_rqt_plugin/littlebot_comm.hpp"

#include "littlebot_base/serial_port.hpp"

namespace littlebot_rqt_plugin
{
LittlebotComm::LittlebotComm(QObject *parent)
    : QObject(parent)
{
    serial_ = std::make_shared<littlebot_base::SerialPort>();
}


// void LittlebotComm::dataRequest() {
//     littlebot_driver_->requestData();
// }

void LittlebotComm::connectHardware(QString portName)
{
    try {
        littlebot_driver_ = std::make_shared<littlebot_base::LittlebotDriver>(serial_, portName.toStdString(), 115200);
    } catch (const std::exception &ex) {
        emit errorOccurred(QString::fromStdString(std::string("Connection failed: ") + ex.what()));
        return;
    }
}

// void LittlebotComm::dataRequest() {
//     QByteArray cmd = "GETDATA\n";
//     serial_->write(cmd);
// }

// void LittlebotComm::onDataAvailable() {
//     QByteArray response = serial_->readAll();

//     // Example: parse numeric values separated by spaces
//     QVector<double> data;
//     QList<QByteArray> parts = response.trimmed().split(' ');
//     for (const auto &p : parts)
//         data.append(p.toDouble());

//     emit dataReceived(data);
// }
}  // namespace littlebot_rqt_plugin