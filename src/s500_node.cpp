#include <chrono>
#include <functional>
#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "s500_ros2_driver/device/ping-device-s500.hpp"
#include "s500_ros2_driver/message/ping-message-s500.hpp"
#include "s500_ros2_driver/utils/link/desktop/abstract-link.hpp"

#include "s500_ros2_driver/msg/s500_profile6_t.hpp"
#include "s500_ros2_driver/msg/s500_distance2.hpp"
#include "std_msgs/msg/header.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

using namespace std::chrono_literals;

class S500PublisherNode : public rclcpp::Node
{
  public:
    S500PublisherNode() : Node("s500_publisher_node")
    {
      // connection parameters: we set these once on initialization and do not change
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
      udp_port_range.from_value = 49152; // start of non-reserved port range
      udp_port_range.to_value = std::numeric_limits<uint16_t>::max();
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
      
      rcl_interfaces::msg::IntegerRange baud_rate_range;
      baud_rate_range.from_value = std::numeric_limits<uint32_t>::min();
      baud_rate_range.to_value = std::numeric_limits<uint32_t>::max();
      baud_rate_range.step = 1;
      
      auto baud_rate_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      baud_rate_descriptor.description = "Serial baud rate of the S500 sounder, to be used when connection_type is serial.";
      baud_rate_descriptor.read_only = true;
      baud_rate_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      baud_rate_descriptor.integer_range.push_back(baud_rate_range);
      this->declare_parameter("baud_rate", 115200, baud_rate_descriptor);
      
      this->connection_type_ = this->get_parameter("connection_type").as_string();
      this->udp_ip_ = this->get_parameter("udp_ip").as_string();
      this->udp_port_ = static_cast<uint16_t>(this->get_parameter("udp_port").as_int());
      this->serial_port_ = this->get_parameter("serial_port").as_string();
      this->baud_rate_ = static_cast<uint32_t>(this->get_parameter("baud_rate").as_int());
      
      
      // ping parameters: we set these once on initialization and do not change
      rcl_interfaces::msg::IntegerRange length_mm_range;
      length_mm_range.from_value = std::numeric_limits<uint32_t>::min();
      length_mm_range.to_value = 142000; // 142m rough maximum range listed in Cerulean Sonar FAQ
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
      gain_index_range.to_value = 13;
      gain_index_range.step = 1;
      
      auto gain_index_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      gain_index_descriptor.description = "Index of gain to be applied. Higher indices correspond to larger gains. Set to -1 for auto gain, otherwise 0-13 sets gain for manual gain.";
      gain_index_descriptor.read_only = true;
      gain_index_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      gain_index_descriptor.integer_range.push_back(gain_index_range);
      this->declare_parameter("gain_index", -1, gain_index_descriptor);
      
      rcl_interfaces::msg::IntegerRange ping_interval_ms_range;
      ping_interval_ms_range.from_value = -1;
      ping_interval_ms_range.to_value = std::numeric_limits<int16_t>::max();
      ping_interval_ms_range.step = 1;
      
      auto ping_interval_ms_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      ping_interval_ms_descriptor.description = "Minimum interval between ping, in ms. Set to -1 to start a single ping.";
      ping_interval_ms_descriptor.read_only = true;
      ping_interval_ms_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      ping_interval_ms_descriptor.integer_range.push_back(ping_interval_ms_range);
      ping_interval_ms_descriptor.additional_constraints = "Ping Parameter field is defined as a signed 16-bit integer. Unknown what the true minimum or maximum ping intervals are.";
      this->declare_parameter("ping_interval_ms", 1000, ping_interval_ms_descriptor);
      
      // pulse_len_usec: 0 for auto mode. Currently ignored and auto duration is always used.
      
      auto packet_type_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      packet_type_descriptor.description = "Name of the packet type you would like in response. 'distance2' returns simple and averaged range estimate. 'profile6_t' returns the entire backscatter, alongside a more verbose output.";
      packet_type_descriptor.read_only = true;
      packet_type_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      this->declare_parameter("packet_type", "profile6_t", packet_type_descriptor);
      
      // reserved: Set to 0.
      
      auto chirp_enable_description = rcl_interfaces::msg::ParameterDescriptor();
      chirp_enable_description.description = "Boolean to enable CHIRP. If false, the s500 will use a monotone ping instead.";
      gain_index_descriptor.read_only = true;
      gain_index_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      this->declare_parameter("chirp_enable", true, chirp_enable_description);
      
      // decimation: Set to 0 for auto range resolution in chirp mode.
      rcl_interfaces::msg::IntegerRange decimation_range;
      decimation_range.from_value = 0;
      decimation_range.to_value = std::numeric_limits<uint8_t>::max();
      decimation_range.step = 1;
      
      auto decimation_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      decimation_descriptor.description = "Integer factor to downsample sonar signal by. Set to 0 for auto range resolution in chirp mode.";
      decimation_descriptor.read_only = true;
      decimation_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      decimation_descriptor.integer_range.push_back(decimation_range);
      this->declare_parameter("decimation", 0, ping_interval_ms_descriptor);
      
      this->start_mm_ = static_cast<uint32_t>(this->get_parameter("start_mm").as_int());
      this->length_mm_ = static_cast<uint32_t>(this->get_parameter("length_mm").as_int());
      this->gain_index_ = static_cast<int16_t>(this->get_parameter("gain_index").as_int());
      this->msec_per_ping_ = static_cast<int16_t>(this->get_parameter("ping_interval_ms").as_int());
      // pulse_len_usec: 0 for auto mode. Currently ignored and auto duration is always used.
      this->packet_type_ = this->get_parameter("packet_type").as_string(); // set report_id_ in init() to isolate error-handling to single function
      // reserved_ = 0; 
      this->chirp_ = static_cast<uint8_t>((this->get_parameter("chirp_enable").as_bool()) ? 1 : 0);
      this->decimation_ = static_cast<uint8_t>(this->get_parameter("decimation").as_int());
      
      // speed of sound: on_parameter_event callback function allows dynamically changing the speed of sound through ROS parameter
      rcl_interfaces::msg::IntegerRange sos_mm_per_sec_range;
      sos_mm_per_sec_range.from_value = std::numeric_limits<uint32_t>::min();
      sos_mm_per_sec_range.to_value = std::numeric_limits<uint32_t>::max(); 
      sos_mm_per_sec_range.step = 1;
      
      auto sos_mm_per_sec_description = rcl_interfaces::msg::ParameterDescriptor();
      sos_mm_per_sec_description.description = "Speed of sound in the sounding medium, in mm/s. Used in distance calculations.";
      sos_mm_per_sec_description.read_only = false;
      sos_mm_per_sec_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      sos_mm_per_sec_description.integer_range.push_back(sos_mm_per_sec_range);
      this->declare_parameter("sos_mm_per_sec", 1500000, sos_mm_per_sec_description);
      
      this->sos_mm_per_sec_ = static_cast<uint32_t>(this->get_parameter("sos_mm_per_sec").as_int());
      
      // metadata parameter: we set this once on initialization and do not change
      auto frame_id_descriptor = rcl_interfaces::msg::ParameterDescriptor();
      frame_id_descriptor.description = "Frame_id to be published in backscatter message header. Can be used to distinguish between multiple devices in the same system.";
      frame_id_descriptor.read_only = true;
      frame_id_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      this->declare_parameter("frame_id", "Sounder_S500", frame_id_descriptor);
      
      this->frame_id_ = this->get_parameter("frame_id").as_string();
      
      if (!init_device()){
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize device. Node will be shut down.");
        rclcpp::shutdown();
        return;
      }
      
      if (report_id_ == s500_ros2_driver::message::S500Id::DISTANCE2){
        this->distance2_publisher_ = this->create_publisher<s500_ros2_driver::msg::S500Distance2>("s500/distance2", 10);
      } else { // report_id_ == s500_ros2_driver::message::S500Id::PROFILE6_T
        this->profile6_t_publisher_ = this->create_publisher<s500_ros2_driver::msg::S500Profile6T>("s500/profile6_t", 10);
      }
      
      RCLCPP_INFO(this->get_logger(), "Starting a dedicated sonar polling thread.");
      this->sonar_polling_thread_ = std::thread(&S500PublisherNode::sonar_poll_loop, this);
      
      rclcpp::on_shutdown(std::bind(&S500PublisherNode::on_shutdown_callback, this));
      
      this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
      
      auto sos_cb = [this](const rclcpp::Parameter & p) {
        uint32_t new_sos_mm_per_sec = static_cast<uint32_t>(p.as_int());
        
        if (!this->device_->set_speed_of_sound(new_sos_mm_per_sec, true)){
          RCLCPP_WARN(this->get_logger(), "Unable to set s500 speed of sound to specified value: %d mm/s. Continuing to log with original speed of sound: %d mm/s", new_sos_mm_per_sec, this->sos_mm_per_sec_);
        } else {
          this->sos_mm_per_sec_ = new_sos_mm_per_sec;
          RCLCPP_INFO(this->get_logger(), "Updated s500 speed of sound to %d mm/s.", this->sos_mm_per_sec_);
        }
      };
      
      this->sos_handler_ = this->param_subscriber_->add_parameter_callback("sos_mm_per_sec", sos_cb);
    }
  private:
    /**
     *  @brief Function to initialize connection to sonar device. Returns true if able to parse the connection string, open that connection, intitialize comms with the sounder, and set Speed of Sound and other ping params. Otherwise returns false with a ROS2 error message explaining where it failed.
     */
    bool init_device()
    {
      // device connection and initialization
      if (this->connection_type_ == "udp"){
        this->connection_string_ = "udp:" + this->udp_ip_ + ":" + std::to_string(this->udp_port_);
      } else if (this->connection_type_ == "serial") {
        this->connection_string_ = "serial:" + this->serial_port_ + ":" + std::to_string(this->baud_rate_);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Connection type %s not recognized. Valid connection types are 'udp' and 'serial'.", this->connection_type_.c_str());
        return false;
      }
      
      this->port_ = s500_ros2_driver::utils::link::desktop::AbstractLink::openUrl(connection_string_);
      if (!this->port_) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open connection on %s. Check device connection.", connection_string_.c_str());
        return false;
      }
      
      this->device_ = std::make_unique<s500_ros2_driver::device::S500>(*this->port_.get());
      if (!this->device_->initialize()){
        RCLCPP_ERROR(this->get_logger(), "Unable to establish communication link to S500 on %s. Check device connection.", connection_string_.c_str());
        return false;
      }
      
      if (!this->device_->set_speed_of_sound(sos_mm_per_sec_, true)){
        RCLCPP_ERROR(this->get_logger(), "Unable to set s500 speed of sound to specified value: %d mm/s.", sos_mm_per_sec_);
        return false;
      }
      
      if (this->packet_type_ == "distance2"){
        this->report_id_ = s500_ros2_driver::message::S500Id::DISTANCE2;
      } else if (this->packet_type_ == "profile6_t"){
        this->report_id_ = s500_ros2_driver::message::S500Id::PROFILE6_T;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Packet type %s not recognized.", packet_type_.c_str());
        return false;
      }
      
      if (!this->device_->set_ping_params(this->start_mm_, this->length_mm_, this->gain_index_, this->msec_per_ping_, this->pulse_len_usec_, this->report_id_, this->reserved_, this->chirp_, this->decimation_, true)){
        RCLCPP_ERROR(this->get_logger(), "Unable to set s500 ping parameters to specified values. Start: %d mm, Length: %d mm, Gain Index (-1 for auto gain): %d, Ping Interval: %d msec, Pulse Length (0 for auto mode): %d usec, report_id (1223 for distance2, 1308 for profile6_t, 0 for disable: %d, Chirp Mode: %d, Decimation (0 for auto range resolution in Chirp Mode): %d", this->start_mm_, this->length_mm_, this->gain_index_, this->msec_per_ping_, this->pulse_len_usec_, this->report_id_, this->chirp_, this->decimation_);
        return false;
      }
      
      RCLCPP_INFO(this->get_logger(), "Device initialized successfully on connection %s. Start: %d mm, Length: %d mm, Gain Index (-1 for auto gain): %d, Ping Interval: %d msec, Pulse Length (0 for auto mode): %d usec, report_id (1223 for distance2, 1308 for profile6_t, 0 for disable: %d, Chirp Mode: %d, Decimation (0 for auto range resolution in Chirp Mode): %d, Speed of Sound: %d mm/s", connection_string_.c_str(), this->start_mm_, this->length_mm_, this->gain_index_, this->msec_per_ping_, this->pulse_len_usec_, this->report_id_, this->chirp_, this->decimation_, this->sos_mm_per_sec_);
      RCLCPP_INFO(this->get_logger(), "Speed of Sound can be modified dynamically using the 'sos_mm_per_sec' parameter. All other Ping Parameters are set statically.");
      return true;
    }
  
    /**
     *  @brief Process loop to wait for and publish sonar messages of type report_id_. S500 waitMessage() function is blocking while reading data from S500, so process will run in a separate thread to prevent blocking main rclcpp process.
     */
    void sonar_poll_loop()
    {
      while (this->sonar_polling_thread_execute_ && rclcpp::ok()){
        
        rcl_time_point_value_t last_message_ts_msec = this->get_clock()->now().nanoseconds()/1e6;
        
        auto msg = this->device_->waitMessage(report_id_, msec_per_ping_); // blocking
        
        if (!this->sonar_polling_thread_execute_){
          break; // stop listening when shutdown is received
        }
        
        if (msg) {
          
          auto message_ts = this->get_clock()->now();
          last_message_ts_msec = message_ts.nanoseconds()/1000000;
          
          if (this->report_id_ == s500_ros2_driver::message::S500Id::DISTANCE2){
            auto& data = this->device_->distance2_data;
            auto distance2_msg = std::make_unique<s500_ros2_driver::msg::S500Distance2>();
            
            distance2_msg->header.stamp = message_ts;
            distance2_msg->header.frame_id = this->frame_id_;
            
            distance2_msg->ping_distance_mm = data.ping_distance_mm;
            distance2_msg->averaged_distance_mm = data.averaged_distance_mm;
            distance2_msg->ping_confidence = data.ping_confidence;
            distance2_msg->average_distance_confidence = data.average_distance_confidence;
            distance2_msg->timestamp_msec = data.timestamp;
            
            this->distance2_publisher_->publish(std::move(distance2_msg));
          } else { // this->report_id_ == s500_ros2_driver::message::S500Id::PROFILE6_T
            auto& data = this->device_->profile6_t_data;
            auto profile6_t_msg = std::make_unique<s500_ros2_driver::msg::S500Profile6T>();
            
            profile6_t_msg->header.stamp = message_ts;
            profile6_t_msg->header.frame_id = this->frame_id_;
            
            profile6_t_msg->ping_number = data.ping_number;
            profile6_t_msg->start_mm = data.start_mm;
            profile6_t_msg->length_mm = data.length_mm;
            profile6_t_msg->start_ping_hz = data.start_ping_hz;
            profile6_t_msg->end_ping_hz = data.end_ping_hz;
            profile6_t_msg->adc_sample_hz = data.adc_sample_hz;
            profile6_t_msg->timestamp_msec = data.timestamp_msec;
            profile6_t_msg->pulse_duration_sec = data.pulse_duration_sec;
            profile6_t_msg->analog_gain = data.analog_gain;
            profile6_t_msg->max_pwr_db = data.max_pwr_db;
            profile6_t_msg->min_pwr_db = data.min_pwr_db;
            profile6_t_msg->this_ping_depth_m = data.this_ping_depth_m;
            profile6_t_msg->smooth_depth_m = data.smooth_depth_m;
            profile6_t_msg->ping_depth_measurement_confidence = data.ping_depth_measurement_confidence;
            profile6_t_msg->gain_index = data.gain_index;
            profile6_t_msg->decimation = data.decimation;
            profile6_t_msg->smooth_depth_measurement_confidence = data.smoothed_depth_measurement_confidence;
            profile6_t_msg->num_results = data.num_results;
            
            if (data.pwr_results && data.num_results > 0){
              profile6_t_msg->pwr_results.assign(data.pwr_results, data.pwr_results + data.pwr_results_length);
            }
            
            this->profile6_t_publisher_->publish(std::move(profile6_t_msg));
          }
        } else {
          rcl_time_point_value_t time_since_last_message_msec = this->get_clock()->now().nanoseconds()/1000000 - last_message_ts_msec;
          if ( time_since_last_message_msec > static_cast<rcl_time_point_value_t>(msec_per_ping_)){
            // only warn if message was fully missed, not concerned with few ms variations in message time
            RCLCPP_WARN(this->get_logger(), "%ld ms since last %s message received. Expected new message within %d ms.", time_since_last_message_msec, packet_type_.c_str(), msec_per_ping_);
          }
        }
      }
      RCLCPP_INFO(this->get_logger(), "Sonar polling thread has ended.");
    }
    
    /**
     *  @brief ROS2 Shutdown Callback function to end sonar_poll_loop() hardware thread and disabling pings on the S500.
     */
    void on_shutdown_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Shutting down sonar polling thread.");
      this->sonar_polling_thread_execute_ = false;
      if (this->sonar_polling_thread_.joinable()){
        this->sonar_polling_thread_.join();
      }
      
      RCLCPP_INFO(this->get_logger(), "Disabling ping on S500 on connection %s.", this->connection_string_.c_str());
      // setting report_id to 0 disables pinging.
      this->device_->set_ping_params(this->start_mm_, this->length_mm_, this->gain_index_, this->msec_per_ping_, this->pulse_len_usec_, 0, this->reserved_, this->chirp_, this->decimation_, false);
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
    std::string packet_type_;
    uint16_t report_id_;
    uint16_t reserved_ = 0; 
    uint8_t chirp_;
    uint8_t decimation_ = 0; // decimation: Set to 0 for auto range resolution in chirp mode.
    
    // dynamic parameter
    uint32_t sos_mm_per_sec_;
    
    // metadata parameter
    std::string frame_id_;

    std::unique_ptr<s500_ros2_driver::device::S500> device_;
    std::shared_ptr<s500_ros2_driver::utils::link::PingPort> port_;
    
    rclcpp::Publisher<s500_ros2_driver::msg::S500Distance2>::SharedPtr distance2_publisher_;
    rclcpp::Publisher<s500_ros2_driver::msg::S500Profile6T>::SharedPtr profile6_t_publisher_;
    
    std::thread sonar_polling_thread_;
    std::atomic<bool> sonar_polling_thread_execute_{true};
    
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> sos_handler_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<S500PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown(); 
  return 0;
}
