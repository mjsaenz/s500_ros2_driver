#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "s500_ros2_driver/device/ping-device-s500.hpp"
#include "s500_ros2_driver/message/ping-message-s500.hpp"
#include "s500_ros2_driver/utils/link/desktop/abstract-link.hpp"

#include "s500_ros2_driver/msg/S500Backscatter.hpp"
#include "std_msgs/msgs/header.hpp"

using namespace std::chrono_literals;

class S500Publisher : public rclcpp::Node
{
  public:
    S500Publisher() : Node("s500_publisher"), count_(0)
    {
      // connection parameters
      auto connection_type_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      connection_type_descriptor.description = "Which communication protocol to use to communcate with the sounder.";
      connection_type_descriptor.read_only = true;
      connection_type_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      connection_type_descriptor.additional_constraints = "Valid inputs are 'udp' or 'serial' only.";
      this->declare_parameter("connection_type", "udp", connection_type_descriptor); 
      
      auto udp_ip_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      udp_ip_descriptor.description = "IPv4 IP of the S500 sounder, to be used when connection_type is udp.";
      udp_ip_descriptor.read_only = true;
      udp_ip_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      this->declare_parameter("udp_ip", "192.168.0.20", udp_ip_descriptor);
      
      rcl_interfaces::msg::IntegerRange udp_port_range;
      udp_port_range.from_value = 49152;
      udp_port_range.to_value = 65535:
      udp_port_range.step = 1;
      
      auto udp_port_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      udp_port_descriptor.description = "UDP port of the S500 sounder, to be used when connection_type is udp.";
      udp_port_descriptor.read_only = true;
      udp_port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      udp_port_descriptor.integer_range.push_back(udp_port_range);
      this->declare_parameter("udp_port", 51200, udp_port_descriptor);
      
      auto serial_port_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      serial_port_descriptor.description = "Serial port of the S500 sounder, to be used when connection_type is serial.";
      serial_port_descriptor.read_only = true;
      serial_port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      this->declare_parameter("serial_port", "/dev/ttyUSB0", serial_port_descriptor);
      
      auto baud_rate_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      baud_rate_descriptor.description = "Serial baud rate of the S500 sounder, to be used when connection_type is serial.";
      baud_rate_descriptor.read_only = true;
      baud_rate_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      this->declare_parameter("baud_rate", 115200, baud_rate_descriptor);
      
      connection_type_ = this->get_parameter("connection_type").as_string();
      udp_ip_ = this->get_parameter("udp_ip").as_string();
      udp_port_ = static_cast<uint16_t>(this->get_parameter("udp_port").as_int());
      serial_port_ = this->get_parameter("serial_port").as_string();
      baud_rate_ = static_cast<uint32_t>(this->get_parameter("baud_rate").as_int());
      
      // ping parameters
      rcl_interfaces::msg::IntegerRange length_mm_range;
      length_mm_range.from_value = 0;
      length_mm_range.to_value = 142000: // 142m rough maximum range listed in Cerulean Sonar FAQ
      length_mm_range.step = 1;
      
      auto start_mm_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      start_mm_descriptor.description = "Start of ping range, i.e. distance from sounder head the first profile bin will correspond to, in mm. Typically set to 0.";
      start_mm_descriptor.read_only = true;
      start_mm_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      start_mm_descriptor.integer_range.push_back(length_mm_range);
      this->declare_parameter("start_mm", 0, start_mm_descriptor);
      
      auto length_mm_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      length_mm_descriptor.description = "Length of the returned file, in mm. End of range = start_mm + length_mm. Set to 0 for auto_range.";
      length_mm_descriptor.read_only = true;
      length_mm_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      length_mm_descriptor.integer_range.push_back(length_mm_range);
      this->declare_parameter("length_mm", 0, length_mm_descriptor);
      
      rcl_interfaces::msg::IntegerRange gain_index_range;
      gain_index_range.from_value = -1;
      gain_index_range.to_value = 13:
      gain_index_range.step = 1;
      
      auto gain_index_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      gain_index_descriptor.description = "Index of gain to be applied. Higher indices correspond to larger gains. Set to -1 for auto gain, otherwise 0-13 sets gain for manual gain.";
      gain_index_descriptor.read_only = true;
      gain_index_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      gain_index_descriptor.integer_range.push_back(gain_index_range);
      this->declare_parameter("gain_index", -1, gain_index_descriptor);
      
      rcl_interfaces::msg::IntegerRange ping_interval_ms_range;
      gain_index_range.from_value = -1;
      gain_index_range.to_value = 32767:
      gain_index_range.step = 1;
      
      auto ping_interval_ms_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      ping_interval_ms_descriptor.description = "Minimum interval between ping, in ms. Set to -1 to start a single ping.";
      ping_interval_ms_descriptor.read_only = true;
      ping_interval_ms_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      ping_interval_ms_descriptor.integer_range.push_back(ping_interval_ms_range);
      connection_type_descriptor.additional_constraints = "Ping Parameter field is defined as a 16-bit integer. Unknown what the true minimum or maximum ping intervals are.";
      this->declare_parameter("ping_interval_ms", 100, ping_interval_ms_descriptor);
      
      // pulse_len_usec: 0 for auto mode. Currently ignored and auto duration is always used.
      
      // report_id: ID of the packet type you would like in response. Options are distance2 (1224), profile6 (1308), or zero. Zero disables pinging. Node currently only supports profile6, as message is defined to report backscatter.
      
      // reserved: Set to 0.
      
      auto chirp_enable_description = rcl_interfaces::msg::ParameterDescriptor();
      chirp_enable_description.description = "Boolean to enable CHIRP. If false, the s500 will use a monotone ping instead.";
      gain_index_descriptor.read_only = true;
      gain_index_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOLEAN;
      this->declare_parameter("chirp_enable", true, chirp_enable_description);
      
      // decimation: Set to 0 for auto range resolution in chirp mode.
      
      
      start_mm_ = static_cast<uint32_t>(this->get_parameter("start_mm").as_int());
      length_mm_ = static_cast<uint32_t>(this->get_parameter("length_mm").as_int());
      gain_index_ = static_cast<int16_t>(this->get_parameter("gain_index").as_int());
      msec_per_ping_ = static_cast<int16_t>(this->get_parameter("ping_interval_ms").as_int());
      // pulse_len_usec: 0 for auto mode. Currently ignored and auto duration is always used.
      // report_id_ = S500Id::PROFILE6_T;
      // reserved_ = 0; 
      uint8_t chirp_ = (this->get_parameter("chirp_enable").as_bool() ? 1 : 0;;
      // decimation_ = 0;
      
      if (!init_device()){
        RCLCPP_ERROR(this->get_logger(), "Failed to init device. Node will be shut down.");
        rclcpp::shutdown();
        return;
      }
      
      publisher_ = this->create_publisher<s500_ros2_driver:msg:S500Backscatter>("S500_backscatter", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&S500Publisher::timer_callback, this));
      
      rclcpp::on_shutdown(std::bind(&S500Publisher::shutdown_callback, this));
    }
  private:
    bool init_device()
    {
      // device connection and initialization
      if (connection_type_ == "udp"){
        connection_string_ = "udp:" + udp_ip_ + ":" + std::to_string(udp_port_);
      } else if (connection_type == "serial") {
        connection_string_ = "serial:" + serial_port_ + ":" + std::to_string(baud_rate_);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Connection type %s not recognized. Valid connection types are 'udp' and 'serial'.", connection_type.c_str());
        return false;
      }
      
      auto port = AbstractLink::openUrl(connection_string_);
      if (!port) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open connection on %s. Check device connection.", connection_string_.c_str());
        return false;
      }
      
      device_ = S500(*port.get());
      if (!device_.initialize()){
        RCLCPP_ERROR(this->get_logger(), "Unable to establish communication link to S500 on %s. Check device connection.", connection_string_.c_str());
        return false;
      }
      
      
      if (!device_.set_ping_params(start_mm_, length_mm_, gain_index_, msec_per_ping_, pulse_len_usec_, report_id_, reserved_, chirp_, decimation_)){
        RCLCPP_ERROR(this->get_logger(), "Unable to set s500 ping parameters to specified values.");
        return false;
      }
      
      return true;
    }
  
    void timer_callback()
    {
      // call read() to attempt to parse profile6_t messages
      // timer and S500 interval should be the same
      // dont use wait_message() as it is blocking while waiting for read
    }
    
    void shutdown_callback()
    {
      RCLCPP_INFO(this->get_logger(). "Disabling ping on S500 on connection %s.", connection_string_.c_str());
      // setting report_id to 0 disables pinging.
      device_.set_ping_params(start_mm_, length_mm_, gain_index_, msec_per_ping_, pulse_len_usec_, 0, reserved_, chirp_, decimation_);
    }
    
    // connection parameters
    std::string connection_type_;
    std::string udp_ip_;
    uint16_t udp_port_;
    std::string serial_port_;
    uint32_t baud_rate_;
    std::string connection_string_;
    
    // ping parameters
    uint32_t start_mm_;
    uint32_t length_mm_;
    int16_t gain_index_;
    int16_t msec_per_ping_;
    uint16_t pulse_len_usec_ = 0; // pulse_len_usec: 0 for auto mode. Currently ignored and auto duration is always used.
    uint16_t report_id_ = S500Id::PROFILE6_T;
    uint16_t reserved_ = 0; 
    uint8_t chirp_;
    uint8_t decimation_ = 0; // decimation: Set to 0 for auto range resolution in chirp mode.
    
    S500 device_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<s500_ros2_driver::msg::S500Backscatter>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<S500Publisher>();
  rclpp::spin(node);
  rclpp::shutdown(); 
  return 0;
}
