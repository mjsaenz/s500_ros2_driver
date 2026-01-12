#include "s500_ros2_driver/utils/link/desktop/abstract-link.hpp"
#include "s500_ros2_driver/utils/link/desktop/serial-link.hpp"
#include "s500_ros2_driver/utils/link/desktop/udp-link.hpp"

// TODO: Remove this
#include <iostream>

#include <boost/format.hpp>
#include <boost/xpressive/xpressive.hpp>

namespace s500_ros2_driver {
namespace utils {
namespace link {
namespace desktop {

/**
 * @brief Accepts inputs with format described in `openUrl`
 *
 */
const char* AbstractLink::_urlStringRegex = R"((?P<type>udp|serial):(?P<host>.*):(?P<config>\d+))";

std::shared_ptr<AbstractLink> AbstractLink::openUrl(const std::string& url)
{
    if (url.empty()) {
        std::cerr << "Empty URL was provided when trying to open a link" << std::endl;
        return {};
    }

    const auto regex = boost::xpressive::sregex::compile(_urlStringRegex);
    boost::xpressive::smatch match;

    struct {
        std::string type;
        std::string host;
        std::string config;
    } urlStruct;

    if (!regex_search(url, match, regex)) {
        std::cerr << "Invalid URL provided, should be in format `type:host:config`" << std::endl;
        return {};
    }

    urlStruct.type = match["type"].str();
    urlStruct.host = match["host"].str();
    urlStruct.config = match["config"].str();

    if (urlStruct.type == "serial") {
        return std::make_shared<SerialLink>(urlStruct.host, std::stoi(urlStruct.config));
    } else if (urlStruct.type == "udp") {
        return std::make_shared<UdpLink>(urlStruct.host, urlStruct.config);
    }

    std::cerr << "Unknown link type provided" << std::endl;
    return {};
}

} // namespace desktop
} // namespace link
} // namespace utils
} // namespace s500_ros2_driver
