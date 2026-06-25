#include "s500_ros2_driver/utils/time/ping-time.hpp"

#include <chrono>
#include <thread>

namespace s500_ros2_driver {
namespace utils {
namespace time {

int PingTime::timeMs()
{
    auto now = std::chrono::steady_clock::now();

    auto duration = now.time_since_epoch();
    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

    return durationMs.count();
}

void PingTime::microsecondDelay(unsigned int microseconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void PingTime::yield()
{
    // Avoid busy-spinning, especially under host load or when ROS sim time is active.
    // A short sleep gives other threads enough time to process I/O and callbacks.
    std::this_thread::sleep_for(std::chrono::microseconds(100));
}

} // namespace time
} // namespace utils
} // namespace s500_ros2_driver
