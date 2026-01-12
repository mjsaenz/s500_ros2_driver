#include "s500_ros2_driver/device/ping-device.hpp"

#include "s500_ros2_driver/utils/link/ping-port.hpp"
#include "s500_ros2_driver/utils/time/ping-time.hpp"

#include <cstdio>

namespace s500_ros2_driver {
namespace device {

bool PingDevice::initialize() { return request(s500_ros2_driver::message::CommonId::PROTOCOL_VERSION) && request(s500_ros2_driver::message::CommonId::DEVICE_INFORMATION); }

s500_ros2_driver::message::ping_message* PingDevice::read()
{
    uint8_t data;
    int result = _port.read(&data, 1);
    if (result != 0 && _parser.parseByte(data) == s500_ros2_driver::message::PingParser::State::NEW_MESSAGE) {
        return &_parser.rxMessage;
    }
    return nullptr;
}

s500_ros2_driver::message::ping_message PingDevice::message() { return _parser.rxMessage; }

s500_ros2_driver::message::ping_message* PingDevice::request(uint16_t id, int timeoutMs)
{
    _general_request.set_requested_id(id);
    writeMessage(_general_request);

    s500_ros2_driver::message::ping_message* reply = waitMessage(id, timeoutMs);
    if (reply && reply->message_id() == s500_ros2_driver::message::CommonId::NACK) {
        s500_ros2_driver::message::common_nack* m = static_cast<s500_ros2_driver::message::common_nack*>(reply);
        if (m->nacked_id() != s500_ros2_driver::message::CommonId::GENERAL_REQUEST) {
            return nullptr;
        }
    }

    return reply;
}

s500_ros2_driver::message::ping_message* PingDevice::waitMessage(uint16_t id, int timeoutMs)
{
    int timeStart = s500_ros2_driver::utils::time::PingTime::timeMs();
    while (s500_ros2_driver::utils::time::PingTime::timeMs() < timeStart + timeoutMs) {
        // TODO: make this totally nonblocking
        // this will block up to 0.1s for data to be available
        s500_ros2_driver::message::ping_message* message = read();

        if (!message) {
            continue;
        }

        _handleMessage(message);

        if (message->message_id() == id || message->message_id() == s500_ros2_driver::message::CommonId::NACK) {
            return message;
        }
        // Prevent cpu spinlock
        s500_ros2_driver::utils::time::PingTime::yield();
    }

    return nullptr;
}

int PingDevice::write(const uint8_t* data, int nBytes) { return _port.write(data, nBytes); }

void PingDevice::writeMessage(s500_ros2_driver::message::ping_message& message)
{
    message.updateChecksum();
    write(reinterpret_cast<uint8_t*>(message.msgData), message.msgDataLength());
}

void PingDevice::_handleMessage(const s500_ros2_driver::message::ping_message* message)
{
    device_id = message->source_device_id();

    switch (message->message_id()) {
    case s500_ros2_driver::message::CommonId::NACK:
        break;
    case s500_ros2_driver::message::CommonId::PROTOCOL_VERSION: {
        const s500_ros2_driver::message::common_protocol_version* message_protocol_version = static_cast<const s500_ros2_driver::message::common_protocol_version*>(message);
        protocol_version.version_major = message_protocol_version->version_major();
        protocol_version.version_minor = message_protocol_version->version_minor();
        protocol_version.version_patch = message_protocol_version->version_patch();
        break;
    }
    case s500_ros2_driver::message::CommonId::DEVICE_INFORMATION: {
        const s500_ros2_driver::message::common_device_information* message_device_information = static_cast<const s500_ros2_driver::message::common_device_information*>(message);
        device_information.device_type = message_device_information->device_type();
        device_information.device_revision = message_device_information->device_revision();
        device_information.firmware_version_major = message_device_information->firmware_version_major();
        device_information.firmware_version_minor = message_device_information->firmware_version_minor();
        device_information.firmware_version_patch = message_device_information->firmware_version_patch();
        break;
    }
    default:
        break;
    }
}

} // namespace device
} // namespace s500_ros2_driver
