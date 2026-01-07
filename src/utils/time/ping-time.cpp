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

void PingTime::yield() { std::this_thread::yield(); }

} // namespace time
} // namespace utils
} // namespace s500_ros2_driver
