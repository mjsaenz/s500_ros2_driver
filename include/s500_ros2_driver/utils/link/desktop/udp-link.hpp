#pragma once

#ifndef S500_ROS2_DRIVER_UTILS_LINK_DESKTOP_UDP_LINK_HPP
#define S500_ROS2_DRIVER_UTILS_LINK_DESKTOP_UDP_LINK_HPP

#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "s500_ros2_driver/utils/link/desktop/abstract-link.hpp"

#include <iostream>

namespace s500_ros2_driver {
namespace utils {
namespace link {
namespace desktop {

class UdpLink : public AbstractLink {
public:
    UdpLink(const std::string& host, const std::string& port);
    ~UdpLink();

    /**
     * @brief Close link
     *
     */
    virtual void close() override final { _socket.close(); };

    /**
     * @brief Check if link is open
     *
     * @return true
     * @return false
     */
    virtual bool isOpen() override final { return _socket.is_open(); };

    /**
     * @brief Copy bytes from our internal buffer to `buffer` argument
     *
     * @param buffer Array that the data will be provided
     * @param nBytes Number of bytes that `buffer` can handle
     * @return int The real number of bytes that was transferred to `buffer`
     */
    virtual int read(uint8_t* buffer, int nBytes) override final;

    /**
     * @brief Send bytes to link
     *
     * @param data Array that contain data that will be transferred to link
     * @param nBytes Number of bytes that `data` contains
     * @return int The real number of bytes that was transferred,
     *  since this class uses async read and async write to be faster than light,
     *  it's not possible to confirm the number of bytes that was successfully transferred.
     */
    virtual int write(const uint8_t* data, int nBytes) override final;

    /**
     * @brief Write data from vector to link
     *
     * @param vector
     */
    void write(const std::vector<uint8_t>& vector);

private:
    UdpLink(const UdpLink&) = delete;
    const UdpLink& operator=(const UdpLink&) = delete;

    /**
     * @brief Start event loop process
     *
     */
    void runEventLoop();

    /**
     * @brief Connection handle callback
     *
     * @param error
     */
    void connectionHandle(boost::system::error_code const& error);

    /**
     * @brief Schedule an async read event
     *
     */
    void bindRead();

    /**
     * @brief Async event callback
     *
     * @param error Error code
     * @param bytesReceived Number of bytes received
     */
    void doRead(boost::system::error_code error, size_t bytesReceived);

    /**
     * @brief Deal with the async event loop
     *  run: Will control if the event loop will run or not
     *  future: Will check if the event loop was able to finish
     */
    struct {
        std::atomic<bool> run;
        boost::asio::io_context eventLoop;
        std::future<void> future;
    } _context;

    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;

    // Buffer used to async read
    std::vector<uint8_t> _rxBuffer;

    /**
     * @brief Buffer used to transfer data between async read and our public `read` function.
     *  This is necessary to avoid any problem of simultaneous `read` call while async read is received.
     */
    struct {
        std::vector<uint8_t> data;
        std::mutex mutex;
    } _linkBuffer;
};

} // namespace desktop
} // namespace link
} // namespace utils
} // namespace s500_ros2_driver

#endif //S500_ROS2_DRIVER_UTILS_LINK_DESKTOP_UDP_LINK_HPP
