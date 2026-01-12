#include "s500_ros2_driver/device/ping-device-s500.hpp"
#include "s500_ros2_driver/message/ping-message-common.hpp"
#include "s500_ros2_driver/message/ping-message-s500.hpp"

namespace s500_ros2_driver {
namespace device {

S500::~S500()
{
    if (profile6_t_data.pwr_results) {
        delete[] profile6_t_data.pwr_results;
    }
}

bool S500::initialize()
{
    return PingDevice::initialize();
}

void S500::_handleMessage(const s500_ros2_driver::message::ping_message* message)
{
    switch (message->message_id()) {
        case s500_ros2_driver::message::S500Id::ALTITUDE:
        {
            const s500_ros2_driver::message::s500_altitude* message_altitude = static_cast<const s500_ros2_driver::message::s500_altitude*>(message);
            altitude_data.altitude_mm = message_altitude->altitude_mm();
            altitude_data.quality = message_altitude->quality();
        }
        break;
        case s500_ros2_driver::message::S500Id::DISTANCE2:
        {
            const s500_ros2_driver::message::s500_distance2* message_distance2 = static_cast<const s500_ros2_driver::message::s500_distance2*>(message);
            distance2_data.ping_distance_mm = message_distance2->ping_distance_mm();
            distance2_data.averaged_distance_mm = message_distance2->averaged_distance_mm();
            distance2_data.reserved = message_distance2->reserved();
            distance2_data.ping_confidence = message_distance2->ping_confidence();
            distance2_data.average_distance_confidence = message_distance2->average_distance_confidence();
            distance2_data.timestamp = message_distance2->timestamp();
        }
        break;
        case s500_ros2_driver::message::S500Id::FW_VERSION:
        {
            const s500_ros2_driver::message::s500_fw_version* message_fw_version = static_cast<const s500_ros2_driver::message::s500_fw_version*>(message);
            fw_version_data.device_type = message_fw_version->device_type();
            fw_version_data.device_model = message_fw_version->device_model();
            fw_version_data.version_major = message_fw_version->version_major();
            fw_version_data.version_minor = message_fw_version->version_minor();
        }
        break;
        case s500_ros2_driver::message::S500Id::GAIN_INDEX:
        {
            const s500_ros2_driver::message::s500_gain_index* message_gain_index = static_cast<const s500_ros2_driver::message::s500_gain_index*>(message);
            gain_index_data.gain_index = message_gain_index->gain_index();
        }
        break;
        case s500_ros2_driver::message::S500Id::PING_RATE_MSEC:
        {
            const s500_ros2_driver::message::s500_ping_rate_msec* message_ping_rate_msec = static_cast<const s500_ros2_driver::message::s500_ping_rate_msec*>(message);
            ping_rate_msec_data.msec_per_ping = message_ping_rate_msec->msec_per_ping();
        }
        break;
        case s500_ros2_driver::message::S500Id::PROCESSOR_DEGC:
        {
            const s500_ros2_driver::message::s500_processor_degC* message_processor_degC = static_cast<const s500_ros2_driver::message::s500_processor_degC*>(message);
            processor_degC_data.centi_degC = message_processor_degC->centi_degC();
        }
        break;
        case s500_ros2_driver::message::S500Id::PROFILE6_T:
        {
            const s500_ros2_driver::message::s500_profile6_t* message_profile6_t = static_cast<const s500_ros2_driver::message::s500_profile6_t*>(message);
            profile6_t_data.ping_number = message_profile6_t->ping_number();
            profile6_t_data.start_mm = message_profile6_t->start_mm();
            profile6_t_data.length_mm = message_profile6_t->length_mm();
            profile6_t_data.start_ping_hz = message_profile6_t->start_ping_hz();
            profile6_t_data.end_ping_hz = message_profile6_t->end_ping_hz();
            profile6_t_data.adc_sample_hz = message_profile6_t->adc_sample_hz();
            profile6_t_data.timestamp_msec = message_profile6_t->timestamp_msec();
            profile6_t_data.spare2 = message_profile6_t->spare2();
            profile6_t_data.pulse_duration_sec = message_profile6_t->pulse_duration_sec();
            profile6_t_data.analog_gain = message_profile6_t->analog_gain();
            profile6_t_data.max_pwr_db = message_profile6_t->max_pwr_db();
            profile6_t_data.min_pwr_db = message_profile6_t->min_pwr_db();
            profile6_t_data.this_ping_depth_m = message_profile6_t->this_ping_depth_m();
            profile6_t_data.smooth_depth_m = message_profile6_t->smooth_depth_m();
            profile6_t_data.fspare2 = message_profile6_t->fspare2();
            profile6_t_data.ping_depth_measurement_confidence = message_profile6_t->ping_depth_measurement_confidence();
            profile6_t_data.gain_index = message_profile6_t->gain_index();
            profile6_t_data.decimation = message_profile6_t->decimation();
            profile6_t_data.smoothed_depth_measurement_confidence = message_profile6_t->smoothed_depth_measurement_confidence();
            profile6_t_data.num_results = message_profile6_t->num_results();
            if (message_profile6_t->pwr_results_length() > profile6_t_data.pwr_results_length) {
                if (profile6_t_data.pwr_results) {
                    delete[] profile6_t_data.pwr_results;
                }
                profile6_t_data.pwr_results = new uint16_t[message_profile6_t->pwr_results_length()];
            }

            // If pointer is invalid, make sure to abort, there is no more memory!
            if (profile6_t_data.pwr_results == nullptr) {
                profile6_t_data.pwr_results_length = -1;
                return;
            }

            profile6_t_data.pwr_results_length = message_profile6_t->pwr_results_length();
            memcpy(profile6_t_data.pwr_results, message_profile6_t->pwr_results(), message_profile6_t->pwr_results_length());
        }
        break;
        case s500_ros2_driver::message::S500Id::RANGE:
        {
            const s500_ros2_driver::message::s500_range* message_range = static_cast<const s500_ros2_driver::message::s500_range*>(message);
            range_data.start_mm = message_range->start_mm();
            range_data.length_mm = message_range->length_mm();
        }
        break;
        case s500_ros2_driver::message::S500Id::SPEED_OF_SOUND:
        {
            const s500_ros2_driver::message::s500_speed_of_sound* message_speed_of_sound = static_cast<const s500_ros2_driver::message::s500_speed_of_sound*>(message);
            speed_of_sound_data.sos_mm_per_sec = message_speed_of_sound->sos_mm_per_sec();
        }
        break;

        default:
            break;
    }

    PingDevice::_handleMessage(message);
}

bool S500::set_ping_params(uint32_t _start_mm, uint32_t _length_mm, int16_t _gain_index, int16_t _msec_per_ping, uint16_t _pulse_len_usec, uint16_t _report_id, uint16_t _reserved, uint8_t _chirp, uint8_t _decimation, bool verify)
{
    s500_ros2_driver::message::s500_set_ping_params message;
    message.set_start_mm(_start_mm);
    message.set_length_mm(_length_mm);
    message.set_gain_index(_gain_index);
    message.set_msec_per_ping(_msec_per_ping);
    message.set_pulse_len_usec(_pulse_len_usec);
    message.set_report_id(_report_id);
    message.set_reserved(_reserved);
    message.set_chirp(_chirp);
    message.set_decimation(_decimation);
    writeMessage(message);

    // Verification for set_ping_params is not implemented, as it requires
    // multiple get messages to be requested and checked (e.g., range, gain_index).
    // The device will reply with a command_ack message, which is handled by the base class.
    (void)verify; // To avoid unused parameter warning
    return true; // Assume success if writeMessage does not throw
}
bool S500::set_speed_of_sound(uint32_t _sos_mm_per_sec, bool verify)
{
    s500_ros2_driver::message::s500_set_speed_of_sound message;
    message.set_sos_mm_per_sec(_sos_mm_per_sec);
    writeMessage(message);

    // Check if we have a reply from the device
    if (!request(s500_ros2_driver::message::S500Id::SPEED_OF_SOUND)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (speed_of_sound_data.sos_mm_per_sec != _sos_mm_per_sec)) {
        return false;
    }
    return true;
}

} // namespace device
} // namespace s500_ros2_driver
