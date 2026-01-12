#pragma once

#ifndef S500_ROS2_DRIVER_UTILS_LINK_PING_PARSER_HPP
#define S500_ROS2_DRIVER_UTILS_LINK_PING_PARSER_HPP

#include <stdint.h>

namespace s500_ros2_driver {
namespace utils {
namespace link {

// port interface used by PingDevice class
class PingPort {
public:
    virtual int read(uint8_t* buffer, int nBytes) = 0;
    virtual int write(const uint8_t* data, int nBytes) = 0;
};

} // namespace link
} // namespace utils
} // namespace s500_ros2_driver

#endif // S500_ROS2_DRIVER_UTILS_LINK_PING_PARSER_HPP
