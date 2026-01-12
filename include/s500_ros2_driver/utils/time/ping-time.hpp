#pragma once

#ifndef S500_ROS2_DRIVER_UTILS_TIME_PING_TIME_HPP
#define S500_ROS2_DRIVER_UTILS_TIME_PING_TIME_HPP

namespace s500_ros2_driver {
namespace utils {
namespace time {

/**
 * @brief Abstract namespace to allow usage between different implementations
 *
 */
namespace PingTime {
void microsecondDelay(unsigned int microseconds);
int timeMs();
void yield();
}

} // namespace time
} // namespace utils
} // namespace s500_ros2_driver

#endif // S500_ROS2_DRIVER_UTILS_TIME_PING_TIME_HPP
