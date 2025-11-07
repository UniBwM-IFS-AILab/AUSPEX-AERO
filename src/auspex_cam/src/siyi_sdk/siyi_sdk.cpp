#include "siyi_sdk/siyi_sdk.hpp"

#include <vector>
#include <cstdint>  // for uint8_t, int32_t, etc.
#include <algorithm> // for std::clamp
#include <iostream>
#include <iomanip>
#include <cstring>      // for memset
#include <arpa/inet.h>  // for sockaddr_in, inet_pton
#include <sys/socket.h> // for socket, connect, send
#include <unistd.h>     // for close
#include <cmath>        // for floor

namespace {
    const uint16_t CRC16_LUT[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
    };
    enum CMD_ID : int {
        TCP_HEARTBEAT = 0x00, // not implemented; command only available with TCP
        GET_FIRMWARE_VERSION = 0x01,
        GET_HARDWARE_ID = 0x02,
        AUTO_FOCUS = 0x04,
        MANUAL_ZOOM_AUTO_FOCUS = 0x05,
        MANUAL_FOCUS = 0x06,
        SET_GIMBAL_ROTATION = 0x07,
        CENTER = 0x08, // only one-key center implemented; only available with TCP according to manual (lies)
        GET_GIMBAL_CONFIG = 0x0a,
        GET_FUNCTION_FEEDBACK = 0x0b,
        PHOTO_AND_RECORD = 0x0c,
        GET_GIMBAL_ATTITUDE = 0x0d,
        SET_CONTROL_ANGLE = 0x0e,
        ABSOLUTE_ZOOM_AUTO_FOCUS = 0x0f,
        GET_IMAGE_MODE = 0x10,
        SET_IMAGE_MODE = 0x11,
        GET_TEMP_POINT = 0x12,
        GET_TEMP_RANGE_IN_BOX = 0x13,
        GET_TEMP_RANGE = 0x14,
        GET_LRF_DIST = 0x15,
        GET_MAX_ZOOM_VALUE = 0x16,
        GET_LRF_LAT_LON = 0x17,
        GET_ZOOM_VALUE = 0x18,
        GET_PRESENT_WORKING_MODE = 0x19,
        GET_THERMAL_COLOR = 0x1a,
        SET_THERMAL_COLOR = 0x1b,
        GET_CODEC_SPECS = 0x20, // not implemented; command maybe only available with TCP (unsure)
        SET_CODEC_SPECS = 0x21, // not implemented; command maybe only available with TCP (unsure)
        SET_AIRCRAFT_ATTITUDE = 0x22,
        SET_RC_CHANNEL_DATA = 0x23, // not implemented; unused
        ENABLE_FC_DATA_STREAM = 0x24, // not implemented; unused
        GET_DATA_STREAM = 0x25,
        GET_GIMBAL_ANGLES = 0x26,
        GET_GIMBAL_CONTROL_DATA = 0x27, // not implemented; only works with ardupilot
        GET_GIMBAL_WEAK_CONTROL_DATA = 0x28, // not implemented; only works with ardupilot
        SET_GIMBAL_WEAK_CONTROL_DATA = 0x29, // not implemented; only works with ardupilot
        GET_MOTOR_VOLTAGE_DATA = 0x2a, // not implemented; only works with ardupilot
        SET_UTC_TIME = 0x30,
        GET_LRF_STATUS = 0x31,
        SET_LRF_STATUS = 0x32,
        GET_THERMAL_IMAGE_MODE = 0x33,
        SET_THERMAL_IMAGE_MODE = 0x34,
        GET_THERMAL_RAW_ONCE = 0x35,
        GET_THERMAL_GAIN = 0x37,
        SET_THERMAL_GAIN = 0x38,
        GET_THERMAL_CORRECT_PARAMS = 0x39,
        SET_THERMAL_CORRECT_PARAMS = 0x3a,
        GET_THERMAL_CORRECT = 0x3b,
        SET_THERMAL_CORRECT = 0x3c,
        SET_GPS_DATA = 0x3e,
        FORMAT_SD_CARD = 0x48,
        GET_PICTURE_NAME_TYPE = 0x49, // not implemented
        SET_PICTURE_NAME_TYPE = 0x4a, // not implemented
        GET_AI_MODE_STATUS = 0x4d, // not implemented; only works with SIYI AI-module
        GET_AI_TRACKING_STREAM_STATUS = 0x4e, // not implemented; only works with SIYI AI-module
        UPDATE_THERMAL_SHUTTER = 0x4f,
        AI_TRACKING_STREAM = 0x50, // not implemented; only works with SIYI AI-module
        SET_AI_TRACKING_STREAM_STATUS = 0x51, // not implemented; only works with SIYI AI-module
        GET_GIMBAL_WEAK_CONTROL = 0x70, // not implemented
        SET_GIMBAL_WEAK_CONTROL = 0x71, // not implemented
        SOFT_REBOOT = 0x80
    };
    const std::unordered_map<int, size_t> CMD_RESPONSE_LENGTHS = {
        {CMD_ID::GET_FIRMWARE_VERSION, 12},
        {CMD_ID::GET_HARDWARE_ID, 12},
        {CMD_ID::AUTO_FOCUS, 1},
        {CMD_ID::MANUAL_ZOOM_AUTO_FOCUS, 2},
        {CMD_ID::MANUAL_FOCUS, 1},
        {CMD_ID::SET_GIMBAL_ROTATION, 1},
        {CMD_ID::CENTER, 1},
        {CMD_ID::GET_GIMBAL_CONFIG, 8},
        {CMD_ID::GET_FUNCTION_FEEDBACK, 1},
        {CMD_ID::GET_GIMBAL_ATTITUDE, 12},
        {CMD_ID::SET_CONTROL_ANGLE, 6},
        {CMD_ID::ABSOLUTE_ZOOM_AUTO_FOCUS, 1},
        {CMD_ID::GET_IMAGE_MODE, 1},
        {CMD_ID::SET_IMAGE_MODE, 1},
        {CMD_ID::GET_TEMP_POINT, 6},
        {CMD_ID::GET_TEMP_RANGE_IN_BOX, 20},
        {CMD_ID::GET_TEMP_RANGE, 12},
        {CMD_ID::GET_LRF_DIST, 2},
        {CMD_ID::GET_MAX_ZOOM_VALUE, 2},
        {CMD_ID::GET_LRF_LAT_LON, 8},
        {CMD_ID::GET_ZOOM_VALUE, 2},
        {CMD_ID::GET_PRESENT_WORKING_MODE, 1},
        {CMD_ID::GET_THERMAL_COLOR, 1},
        {CMD_ID::SET_THERMAL_COLOR, 1},
        {CMD_ID::GET_CODEC_SPECS, 9},
        {CMD_ID::SET_CODEC_SPECS, 2},
        {CMD_ID::GET_DATA_STREAM, 1},
        {CMD_ID::GET_LRF_STATUS, 1},
        {CMD_ID::SET_LRF_STATUS, 1},
        {CMD_ID::SET_THERMAL_IMAGE_MODE, 1},
        {CMD_ID::GET_THERMAL_RAW_ONCE, 1},
        {CMD_ID::GET_THERMAL_GAIN, 1},
        {CMD_ID::SET_THERMAL_GAIN, 1},
        {CMD_ID::GET_THERMAL_CORRECT_PARAMS, 10},
        {CMD_ID::SET_THERMAL_CORRECT_PARAMS, 1},
        {CMD_ID::GET_THERMAL_CORRECT, 1},
        {CMD_ID::SET_THERMAL_CORRECT, 1},
        {CMD_ID::GET_GIMBAL_ANGLES, 6},
        {CMD_ID::SET_UTC_TIME, 1},
        {CMD_ID::GET_PICTURE_NAME_TYPE, 2},
        {CMD_ID::SET_PICTURE_NAME_TYPE, 1},
        {CMD_ID::SOFT_REBOOT, 2}
    };
}


// Constructor
SiyiSDK::SiyiSDK() : running_(true)
{
    init_socket();
    recv_thread_ = std::thread([this] { this->receiveLoop(); });
    heartbeat_thread_ = std::thread([this]{ this->heartbeatLoop(); });
}

// Destructor
SiyiSDK::~SiyiSDK()
{
    running_ = false;
    close_socket();
    if (recv_thread_.joinable()) recv_thread_.join();
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
}

// Initialize Socket
bool SiyiSDK::init_socket()
{
    // Check if already open
    if (sock_fd_ >= 0) return true;

    // Create TCP socket
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        perror("socket");
        return false;
    }

    // Setup local address
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_port = 0;
    inet_pton(AF_INET, local_ip_.c_str(), &local_addr_.sin_addr);

    if (bind(sock_fd_.load(), (sockaddr*)&local_addr_, sizeof(local_addr_)) < 0) {
        perror("bind");
        return false;
    }

    // Convert Cam IP Address
    memset(&camera_addr_, 0, sizeof(camera_addr_));
    camera_addr_.sin_family = AF_INET;
    camera_addr_.sin_port = htons(camera_port_);
    if (inet_pton(AF_INET, camera_ip_.c_str(), &camera_addr_.sin_addr) <= 0) {
        perror("inet_pton camera_ip");
        return false;
    }

    // Connect to camera (TCP handshake)
    if (connect(sock_fd_, (sockaddr*)&camera_addr_, sizeof(camera_addr_)) < 0) {
        perror("connect");
        return false;
    }

    // Add timeout
    struct timeval tv;
    tv.tv_sec = 1;  // 1 second
    tv.tv_usec = 0;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("setsockopt failed");
    }
    
    this->connected_ = true;
    return true;
}

void SiyiSDK::close_socket()
{
    int fd = sock_fd_.exchange(-1);
    if (fd >= 0) {
        shutdown(fd, SHUT_RDWR);
        close(fd);
    }
    this->connected_ = false;
}

bool SiyiSDK::reconnect_blocking()
{
    fail_all_pending("Reconnecting");
    close_socket();

    // Exponential backoff: 200ms -> 400ms ... capped at 5s
    using namespace std::chrono;
    auto delay = 200ms;
    const auto max_delay = 5000ms;

    while (running_) {
        if (init_socket()) {
            return true;
        }
        std::this_thread::sleep_for(delay);
        delay = std::min(delay * 2, max_delay);
    }
    return false;
}

// This function ONLY SENDS the specified message
bool SiyiSDK::send_only(int ctrl, int cmd_id, const std::vector<uint8_t>& data)
{
    if (!connected_ || sock_fd_ < 0) return false;  // Socket not open

    auto msg = encode_msg(ctrl, cmd_id, data);
    
    return send_all(msg);
}

// This function sends the specified message and returns the response data
std::optional<std::vector<uint8_t>> SiyiSDK::send_receive(int ctrl, int cmd_id, const std::vector<uint8_t>& data, int retries, int timeout_ms)
{
    if (!connected_ || sock_fd_ < 0) return std::nullopt;  // Socket not open
    if (retries == -1) retries = this->default_send_receive_retries_;
    if (timeout_ms == -1) timeout_ms = this->default_recv_timeout_ms_;

    auto msg = encode_msg(ctrl, cmd_id, data);
    if (cmd_id == 0x0c) cmd_id = 0x0b; // Special case

    // Retry loop
    for (int attempt = 0; attempt < retries; attempt++) {
        // Create and add promise
        std::promise<std::vector<uint8_t>> prom;
        auto fut = prom.get_future();
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_[cmd_id] = std::move(prom);
        }

        // Send message
        auto succ = send_all(msg);
        if (!succ)
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_.erase(cmd_id);
            continue;
        }

        // Wait for answer
        if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
            try {
                return fut.get(); // may throw if fail_all_pending() set an exception
            } catch (const std::exception& e) {
                {
                    std::lock_guard<std::mutex> lock(pending_mutex_);
                    pending_.erase(cmd_id);
                }
                continue;
            }
        } else {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_.erase(cmd_id);
        }
    }
    return std::nullopt;
}

void SiyiSDK::receiveLoop()
{
    std::vector<uint8_t> streamBuf;
    uint8_t tempBuf[1024];
    
    while (this->running_)
    {
        int recv_bytes = recv(sock_fd_, tempBuf, sizeof(tempBuf), 0);
        if (recv_bytes == 0) {
            // Peer performed an orderly shutdown
            fail_all_pending("Connection closed by peer");
            if (!running_) break;
            if (!reconnect_blocking()) break;
            streamBuf.clear();
            continue;
        }
        if (recv_bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue; // timeout, loop again
            }
            // Hard error (ECONNRESET, ENOTCONN, etc.)
            fail_all_pending(std::string("Socket error: ") + strerror(errno));
            if (!running_) break;
            if (!reconnect_blocking()) break;
            streamBuf.clear();
            continue;
        }
        
        // Append new bytes into stream buffer
        streamBuf.insert(streamBuf.end(), tempBuf, tempBuf + recv_bytes);
        
        // Try to parse complete messages
        while (streamBuf.size() >= 10) {  // minimum message length
            // Look for sync header
            if (streamBuf[0] != 0x55 || streamBuf[1] != 0x66) {
                // Drop until we find a header
                streamBuf.erase(streamBuf.begin());
                continue;
            }
            
            // Check buffer length again
            size_t data_len = streamBuf[3] | (streamBuf[4] << 8);
            size_t total_len = 8 + data_len;
            if (streamBuf.size() < total_len) break; // Not enough data yet, wait for more

            // Extract one full message
            std::vector<uint8_t> msg(streamBuf.begin(), streamBuf.begin() + total_len);
            streamBuf.erase(streamBuf.begin(), streamBuf.begin() + total_len);

            // Error checking
            int cmd_id = static_cast<int>(msg[7]);
            auto it = CMD_RESPONSE_LENGTHS.find(cmd_id);
            if (it == CMD_RESPONSE_LENGTHS.end() || data_len != it->second) continue; // unknown command length or malformed message
            if (data_len != CMD_RESPONSE_LENGTHS.at(cmd_id)) continue; // Wrong message length
            
            // Extract data
            std::vector<uint8_t> data(msg.begin() + 8, msg.begin() + 8 + data_len);

            // Fulfill promise
            {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                auto it = pending_.find(cmd_id);
                if (it != pending_.end()) {
                    it->second.set_value(data);
                    pending_.erase(it);
                }
            }

            // Update cache
            if (cmd_id == CMD_ID::GET_TEMP_POINT ||
                cmd_id == CMD_ID::GET_TEMP_RANGE_IN_BOX ||
                cmd_id == CMD_ID::GET_TEMP_RANGE ||
                cmd_id == CMD_ID::GET_LRF_DIST ||
                cmd_id == CMD_ID::GET_GIMBAL_ATTITUDE) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                cache_[cmd_id] = data;
            }
        }
    }
}

void SiyiSDK::heartbeatLoop()
{
    using namespace std::chrono;

    while (running_) {
        // Sleep first so we don’t hammer during reconnect storms
        std::this_thread::sleep_for(milliseconds(heartbeat_interval_ms_));

        if (!running_) break;

        if (connected_ && sock_fd_ >= 0) {
            send_only(0, CMD_ID::TCP_HEARTBEAT, {});
        }
    }
}

// Sends the entire message vector. Returns true only if all bytes are send. Returns false if there is an error.
bool SiyiSDK::send_all(const std::vector<uint8_t>& buf)
{
    size_t off = 0;
    const size_t len = buf.size();
    if (len == 0) return true;

    while (off < len) {
        ssize_t n = ::send(sock_fd_, buf.data() + off, static_cast<int>(len - off), MSG_NOSIGNAL);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            // Hard error (EPIPE/ENOTCONN/ECONNRESET, etc.)
            return false;
        }
        // n == 0 is unexpected for send(); treat as fatal
        return false;
    }
    return true;
}

// Fails all pending promises so they don't wait.
void SiyiSDK::fail_all_pending(const std::string& reason)
{
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto &kv : pending_) {
        try {
            kv.second.set_exception(
                std::make_exception_ptr(std::runtime_error(reason))
            );
        } catch (...) {
            // If promise already satisfied, ignore
        }
    }
    pending_.clear();
}

//////////////
// Messages //
//////////////

// 0x00: TCP Heartbeat
bool SiyiSDK::send_tcp_heartbeat()
{
    return this->send_only(0, CMD_ID::TCP_HEARTBEAT, {});
}

// 0x01: Request Gimbal Camera Firmware Version
std::optional<std::tuple<std::string, std::string, std::string>> SiyiSDK::get_firmware_version()
{
    auto result = this->send_receive(1, CMD_ID::GET_FIRMWARE_VERSION, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    auto version_to_string = [](const uint8_t* bytes) -> std::string {
        std::ostringstream oss;
        oss << static_cast<int>(bytes[2]) << "." 
            << static_cast<int>(bytes[1]) << "." 
            << static_cast<int>(bytes[0]);
        return oss.str();
    };
    std::string camera_ver = version_to_string(&msg[0]);
    std::string gimbal_ver = version_to_string(&msg[4]);
    std::string zoom_ver = version_to_string(&msg[8]);
    return std::tuple(camera_ver, gimbal_ver, zoom_ver);
}

// 0x02: Request Gimbal Camera Hardware ID
std::optional<std::string> SiyiSDK::get_hardware_id()
{
    auto result = this->send_receive(1, CMD_ID::GET_HARDWARE_ID, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return std::string(reinterpret_cast<const char*>(msg.data()), 12);
}

// 0x04: Auto Focus
bool SiyiSDK::set_auto_focus(int x, int y)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(0),
        static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF)
    };

    auto result = this->send_receive(1, CMD_ID::AUTO_FOCUS, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x05: Manual Zoom Auto Focus
std::optional<float> SiyiSDK::manual_zoom_in()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_ZOOM_AUTO_FOCUS, {1});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float value = bytes_to_int(msg, 0, 2, false) / 10.0f;
    return value;
}
std::optional<float> SiyiSDK::manual_zoom_out()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_ZOOM_AUTO_FOCUS, {static_cast<uint8_t>(-1)});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float value = bytes_to_int(msg, 0, 2, false) / 10.0f;
    return value;
}
std::optional<float> SiyiSDK::manual_zoom_stop()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_ZOOM_AUTO_FOCUS, {0});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float value = bytes_to_int(msg, 0, 2, false) / 10.0f;
    return value;
}

// 0x06: Manual Focus
bool SiyiSDK::manual_focus_far()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_FOCUS, {1});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
bool SiyiSDK::manual_focus_near()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_FOCUS, {static_cast<uint8_t>(-1)});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
bool SiyiSDK::manual_focus_stop()
{
    auto result = this->send_receive(1, CMD_ID::MANUAL_FOCUS, {0});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x07: Gimbal Rotation
bool SiyiSDK::set_gimbal_rotation_speed(int yaw_rate, int pitch_rate)
{
    yaw_rate = std::clamp(yaw_rate, -100, 100);
    pitch_rate = std::clamp(pitch_rate, -100, 100);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(yaw_rate),
        static_cast<uint8_t>(pitch_rate)
    };
    
    auto result = this->send_receive(1, CMD_ID::SET_GIMBAL_ROTATION, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x08: Center
bool SiyiSDK::center()
{
    auto result = this->send_receive(1, CMD_ID::CENTER, {1});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x0a: Request Gimbal Configuration Information
std::optional<std::tuple<int, int, int, int, int>> SiyiSDK::get_camera_config()
{
    auto result = this->send_receive(1, CMD_ID::GET_GIMBAL_CONFIG, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return std::make_tuple(msg[3], msg[4], msg[5], msg[6], msg[7]);
}

// 0x0c: Photo and Record
bool SiyiSDK::take_picture()
{
    auto result = this->send_receive(0, CMD_ID::PHOTO_AND_RECORD, {0}, 1, 200);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x00);
}
bool SiyiSDK::start_stop_recording()
{
    auto result = this->send_receive(0, CMD_ID::PHOTO_AND_RECORD, {2}, 1, 200);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x00);
}
bool SiyiSDK::set_gimbal_mode_lock()
{
    auto result = this->send_only(0, CMD_ID::PHOTO_AND_RECORD, {3});
    return result;
}
bool SiyiSDK::set_gimbal_mode_follow()
{
    auto result = this->send_only(0, CMD_ID::PHOTO_AND_RECORD, {4});
    return result;
}
bool SiyiSDK::set_gimbal_mode_fpv()
{
    auto result = this->send_only(0, CMD_ID::PHOTO_AND_RECORD, {5});
    return result;
}
bool SiyiSDK::set_gimbal_mode_tilt_downward()
{
    auto result = this->send_only(0, CMD_ID::PHOTO_AND_RECORD, {9});
    return result;
}
bool SiyiSDK::set_gimbal_mode_linked_zoom()
{
    auto result = this->send_only(0, CMD_ID::PHOTO_AND_RECORD, {10});
    return result;
}

// 0x0d: Request Gimbal Attitude
std::optional<std::tuple<float, float, float, float, float, float>> SiyiSDK::get_gimbal_attitude()
{
    auto result = this->send_receive(1, CMD_ID::GET_GIMBAL_ATTITUDE, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float yaw = bytes_to_int(msg, 0, 2) / 10.0f;
    float pitch = bytes_to_int(msg, 2, 2) / 10.0f;
    float roll = bytes_to_int(msg, 4, 2) / 10.0f;
    float yaw_vel = bytes_to_int(msg, 6, 2) / 10.0f;
    float pitch_vel = bytes_to_int(msg, 8, 2) / 10.0f;
    float roll_vel = bytes_to_int(msg, 10, 2) / 10.0f;
    return std::make_tuple(yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel);
}

// 0x0e: Send Control Angle to Gimbal
std::optional<std::tuple<float, float, float>> SiyiSDK::set_control_angle(float yaw, float pitch)
{
    int16_t _yaw = (int16_t)(yaw * 10.0f);
    int16_t _pitch = (int16_t)(pitch * 10.0f);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(_yaw & 0xFF), static_cast<uint8_t>((_yaw >> 8) & 0xFF),
        static_cast<uint8_t>(_pitch & 0xFF), static_cast<uint8_t>((_pitch >> 8) & 0xFF)
    };
    
    auto result = this->send_receive(1, CMD_ID::SET_CONTROL_ANGLE, data);
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    yaw = bytes_to_int(msg, 0, 2) / 10.0f;
    pitch = bytes_to_int(msg, 2, 2) / 10.0f;
    float roll = bytes_to_int(msg, 4, 2) / 10.0f;
    return std::make_tuple(yaw, pitch, roll);
}

// 0x0f: Absolute Zoom and Auto Focus
bool SiyiSDK::set_zoom(float zoom)
{
    uint8_t integer_part = static_cast<uint8_t>(std::floor(zoom));
    uint8_t fractional_part = static_cast<uint8_t>(std::floor((zoom - integer_part) * 10));
    std::vector<uint8_t> data = {integer_part, fractional_part};
    
    auto result = this->send_receive(1, CMD_ID::ABSOLUTE_ZOOM_AUTO_FOCUS, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x10: Request Gimbal Camera Image Mode
std::optional<int> SiyiSDK::get_image_mode()
{
    auto result = this->send_receive(1, CMD_ID::GET_IMAGE_MODE, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return msg[0];
}

// 0x11: Send Image Mode to Gimbal Camera
std::optional<int> SiyiSDK::set_image_mode(int mode)
{
    auto result = this->send_receive(1, CMD_ID::SET_IMAGE_MODE, {static_cast<uint8_t>(mode)});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return msg[0];
}

// 0x12: Request the Temperature of a Point
std::optional<float> SiyiSDK::get_temp_point(int x, int y)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF),
        1
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_TEMP_POINT, data, -1, 300);
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float temp = bytes_to_int(msg, 0, 2, false) / 100.0f;
    return temp;
}
bool SiyiSDK::start_temp_point_caching(int x, int y)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF),
        2
    };
    
    return this->send_only(1, CMD_ID::GET_TEMP_POINT, data);
}
bool SiyiSDK::stop_temp_point_caching(int x, int y)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF),
        0
    };
    
    return this->send_only(1, CMD_ID::GET_TEMP_POINT, data);
}
std::optional<std::tuple<float, int, int>> SiyiSDK::get_cached_temp_point()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(CMD_ID::GET_TEMP_POINT);
    if (it != cache_.end() && !it->second.empty()) {
        auto msg = it->second;
        float temp = bytes_to_int(msg, 0, 2, false) / 100.0f;
        int x = bytes_to_int(msg, 2, 2, false);
        int y = bytes_to_int(msg, 4, 2, false);
        return std::make_tuple(temp, x, y);
    }
    return std::nullopt;
}

// 0x13: Request the Max / Min Temperature in a Selected Box
std::optional<std::tuple<float, int, int, float, int, int>> SiyiSDK::get_temp_range_box(int x0, int y0, int x1, int y1)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x0 & 0xFF), static_cast<uint8_t>((x0 >> 8) & 0xFF),
        static_cast<uint8_t>(y0 & 0xFF), static_cast<uint8_t>((y0 >> 8) & 0xFF),
        static_cast<uint8_t>(x1 & 0xFF), static_cast<uint8_t>((x1 >> 8) & 0xFF),
        static_cast<uint8_t>(y1 & 0xFF), static_cast<uint8_t>((y1 >> 8) & 0xFF),
        1
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_TEMP_RANGE_IN_BOX, data, -1, 300);
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float temp_max = bytes_to_int(msg, 8, 2, false) / 100.0f;
    float temp_min = bytes_to_int(msg, 10, 2, false) / 100.0f;
    float temp_max_x = bytes_to_int(msg, 12, 2, false);
    float temp_max_y = bytes_to_int(msg, 14, 2, false);
    float temp_min_x = bytes_to_int(msg, 16, 2, false);
    float temp_min_y = bytes_to_int(msg, 18, 2, false);
    return std::make_tuple(temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y);
}
bool SiyiSDK::start_temp_range_box_caching(int x0, int y0, int x1, int y1)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x0 & 0xFF), static_cast<uint8_t>((x0 >> 8) & 0xFF),
        static_cast<uint8_t>(y0 & 0xFF), static_cast<uint8_t>((y0 >> 8) & 0xFF),
        static_cast<uint8_t>(x1 & 0xFF), static_cast<uint8_t>((x1 >> 8) & 0xFF),
        static_cast<uint8_t>(y1 & 0xFF), static_cast<uint8_t>((y1 >> 8) & 0xFF),
        2
    };
    
    return this->send_only(1, CMD_ID::GET_TEMP_RANGE_IN_BOX, data);
}
bool SiyiSDK::stop_temp_range_box_caching(int x0, int y0, int x1, int y1)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(x0 & 0xFF), static_cast<uint8_t>((x0 >> 8) & 0xFF),
        static_cast<uint8_t>(y0 & 0xFF), static_cast<uint8_t>((y0 >> 8) & 0xFF),
        static_cast<uint8_t>(x1 & 0xFF), static_cast<uint8_t>((x1 >> 8) & 0xFF),
        static_cast<uint8_t>(y1 & 0xFF), static_cast<uint8_t>((y1 >> 8) & 0xFF),
        0
    };
    
    return this->send_only(1, CMD_ID::GET_TEMP_RANGE_IN_BOX, data);
}
std::optional<std::tuple<int, int, int, int, float, int, int, float, int, int>> SiyiSDK::get_cached_temp_range_box()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(CMD_ID::GET_TEMP_RANGE_IN_BOX);
    if (it != cache_.end() && !it->second.empty()) {
        auto msg = it->second;
        int x0 = bytes_to_int(msg, 0, 2, false);
        int y0 = bytes_to_int(msg, 2, 2, false);
        int x1 = bytes_to_int(msg, 4, 2, false);
        int y1 = bytes_to_int(msg, 6, 2, false);
        float temp_max = bytes_to_int(msg, 8, 2, false) / 100.0f;
        float temp_min = bytes_to_int(msg, 10, 2, false) / 100.0f;
        float temp_max_x = bytes_to_int(msg, 12, 2, false);
        float temp_max_y = bytes_to_int(msg, 14, 2, false);
        float temp_min_x = bytes_to_int(msg, 16, 2, false);
        float temp_min_y = bytes_to_int(msg, 18, 2, false);
        return std::make_tuple(x0, y0, x1, y1, temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y);
    }
    return std::nullopt;
}

// 0x14: Request the Max / Min Temperature in the full image
std::optional<std::tuple<float, int, int, float, int, int>> SiyiSDK::get_temp_range_global()
{
    auto result = this->send_receive(1, CMD_ID::GET_TEMP_RANGE, {1}, -1, 300);
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float temp_max = bytes_to_int(msg, 0, 2, false) / 100.0f;
    float temp_min = bytes_to_int(msg, 2, 2, false) / 100.0f;
    int temp_max_x = bytes_to_int(msg, 4, 2, false);
    int temp_max_y = bytes_to_int(msg, 6, 2, false);
    int temp_min_x = bytes_to_int(msg, 8, 2, false);
    int temp_min_y = bytes_to_int(msg, 10, 2, false);
    return std::make_tuple(temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y);
}
bool SiyiSDK::start_temp_range_global_caching()
{
    return this->send_only(1, CMD_ID::GET_TEMP_RANGE, {2});
}
bool SiyiSDK::stop_temp_range_global_caching()
{
    return this->send_only(1, CMD_ID::GET_TEMP_RANGE, {0});
}
std::optional<std::tuple<float, int, int, float, int, int>> SiyiSDK::get_cached_temp_range_global()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(CMD_ID::GET_TEMP_RANGE);
    if (it != cache_.end() && !it->second.empty()) {
        auto msg = it->second;
        float temp_max = bytes_to_int(msg, 0, 2, false) / 100.0f;
        float temp_min = bytes_to_int(msg, 2, 2, false) / 100.0f;
        int temp_max_x = bytes_to_int(msg, 4, 2, false);
        int temp_max_y = bytes_to_int(msg, 6, 2, false);
        int temp_min_x = bytes_to_int(msg, 8, 2, false);
        int temp_min_y = bytes_to_int(msg, 10, 2, false);
        return std::make_tuple(temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y);
    }
    return std::nullopt;
}

// 0x15: Request Range Value from the Laser Rangefinder
std::optional<float> SiyiSDK::get_lrf_dist()
{
    auto result = this->send_receive(1, CMD_ID::GET_LRF_DIST, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float range = bytes_to_int(msg, 0, 2, false) / 10.0f;
    return range;
}

// 0x16: Request the Max Zoom Value in Present
std::optional<float> SiyiSDK::get_max_zoom_value()
{
    auto result = this->send_receive(1, CMD_ID::GET_MAX_ZOOM_VALUE, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    int integer_part = bytes_to_int(msg, 0, 1, false);
    int fractional_part = bytes_to_int(msg, 1, 1, false);
    float max_zoom = integer_part + (fractional_part / 10.0f);
    return max_zoom;
}

// 0x17: Request the Latitude and Longitude of the Laser Rangefinder’s Target
std::optional<std::tuple<float, float>> SiyiSDK::get_lrf_lat_lon()
{
    auto result = this->send_receive(1, CMD_ID::GET_LRF_LAT_LON, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    int lon = bytes_to_int(msg, 0, 4) / 10000000.0f;
    int lat = bytes_to_int(msg, 4, 4) / 10000000.0f;
    return std::make_tuple(lat, lon);
}

// 0x18: Request the Zoom Value in Present
std::optional<float> SiyiSDK::get_current_zoom_value()
{
    auto result = this->send_receive(1, CMD_ID::GET_ZOOM_VALUE, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    int integer_part = bytes_to_int(msg, 0, 1, false);
    int fractional_part = bytes_to_int(msg, 0, 1, false);
    float zoom = integer_part + (fractional_part / 10.0f);
    return zoom;
}

// 0x19: Request Gimbal Camera’s Present Working Mode
std::optional<int> SiyiSDK::get_present_working_mode()
{
    auto result = this->send_receive(1, CMD_ID::GET_PRESENT_WORKING_MODE, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return msg[0];
}

// 0x1a: Request the Thermal Color Palette
std::optional<int> SiyiSDK::get_thermal_color()
{
    auto result = this->send_receive(1, CMD_ID::GET_THERMAL_COLOR, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return msg[0];
}

// 0x1b: Send a Thermal Color to Gimbal Camera
std::optional<int> SiyiSDK::set_thermal_color(int color)
{
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_COLOR, {static_cast<uint8_t>(color)});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return msg[0];
}

// command might not exist: // 0x20: Request Gimbal Camera Codec Specs
// std::optional<std::tuple<int, int, int, int, int>> SiyiSDK::get_stream_codec_specs(int stream)
// {
//     auto result = this->send_receive(1, CMD_ID::GET_CODEC_SPECS, {static_cast<uint8_t>(stream)});
//     if (!result) return std::nullopt;
//     auto& msg = *result;
    
//     int enc = bytes_to_int(msg, 1, 1, false);
//     int res_w = bytes_to_int(msg, 2, 2, false);
//     int res_h = bytes_to_int(msg, 4, 2, false);
//     int bitrate = bytes_to_int(msg, 6, 2, false);
//     int framerate = bytes_to_int(msg, 8, 1, false);
    
//     return std::make_tuple(enc, res_w, res_h, bitrate, framerate);
// }

// command might not exist: 0x21: Request Gimbal Camera Codec Specs
// bool SiyiSDK::set_stream_codec_specs(int stream, int enc, int res_w, int res_h, int bitrate)
// {
//     std::vector<uint8_t> data = {
//         static_cast<uint8_t>(stream & 0xFF),
//         static_cast<uint8_t>(enc & 0xFF),
//         static_cast<uint8_t>(res_w & 0xFF), static_cast<uint8_t>((res_w >> 8) & 0xFF),
//         static_cast<uint8_t>(res_h & 0xFF), static_cast<uint8_t>((res_h >> 8) & 0xFF),
//         static_cast<uint8_t>(bitrate & 0xFF), static_cast<uint8_t>((bitrate >> 8) & 0xFF),
//         0
//     };
    
//     auto result = this->send_receive(1, CMD_ID::SET_CODEC_SPECS, data);
//     if (!result) return false;
//     auto& msg = *result;
    
//     int succ = bytes_to_int(msg, 1, 1, false);
//     return succ;
// }

// 0x22: Send Aircraft Attitude Data to Gimbal
bool SiyiSDK::set_aircraft_attitude(int time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(time_boot_ms & 0xFF), static_cast<uint8_t>((time_boot_ms >> 8) & 0xFF), static_cast<uint8_t>((time_boot_ms >> 16) & 0xFF), static_cast<uint8_t>((time_boot_ms >> 24) & 0xFF),
    };
    append_float_to_bytes(data, roll);
    append_float_to_bytes(data, pitch);
    append_float_to_bytes(data, yaw);
    append_float_to_bytes(data, rollspeed);
    append_float_to_bytes(data, pitchspeed);
    append_float_to_bytes(data, yawspeed);
    
    auto result = this->send_only(1, CMD_ID::SET_AIRCRAFT_ATTITUDE, data);
    return result;
}

// 0x25: Request Gimbal to Send Data Stream
bool SiyiSDK::start_gimbal_attitude_caching(int frequency)
{
    std::vector<uint8_t> data = {
        1,
        static_cast<uint8_t>(frequency)
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_DATA_STREAM, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
bool SiyiSDK::stop_gimbal_attitude_caching()
{
    std::vector<uint8_t> data = {
        1,
        0
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_DATA_STREAM, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
std::optional<std::tuple<float, float, float, float, float, float>> SiyiSDK::get_cached_gimbal_attitude()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(CMD_ID::GET_GIMBAL_ATTITUDE);
    if (it != cache_.end() && !it->second.empty()) {
        auto msg = it->second;
        float yaw = bytes_to_int(msg, 0, 2) / 10.0f;
        float pitch = bytes_to_int(msg, 2, 2) / 10.0f;
        float roll = bytes_to_int(msg, 4, 2) / 10.0f;
        float yaw_vel = bytes_to_int(msg, 6, 2) / 10.0f;
        float pitch_vel = bytes_to_int(msg, 8, 2) / 10.0f;
        float roll_vel = bytes_to_int(msg, 10, 2) / 10.0f;
        return std::make_tuple(yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel);
    }
    return std::nullopt;
}
bool SiyiSDK::start_lrf_dist_caching()
{
    std::vector<uint8_t> data = {
        2,
        3
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_DATA_STREAM, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x02);
}
bool SiyiSDK::stop_lrf_dist_caching()
{
    std::vector<uint8_t> data = {
        2,
        0
    };
    
    auto result = this->send_receive(1, CMD_ID::GET_DATA_STREAM, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x02);
}
std::optional<float> SiyiSDK::get_cached_lrf_dist()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(CMD_ID::GET_LRF_DIST);
    if (it != cache_.end() && !it->second.empty()) {
        auto msg = it->second;
        float range = bytes_to_int(msg, 0, 2, false) / 10.0f;
        return range;
    }
    return std::nullopt;
}

// 0x26: Request Gimbal Magnetic Encoder Angle Data
std::optional<std::tuple<float, float, float>> SiyiSDK::get_gimbal_raw_angles()
{
    auto result = this->send_receive(1, CMD_ID::GET_GIMBAL_ANGLES, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float yaw = bytes_to_int(msg, 0, 2) / 10.0f;
    float pitch = bytes_to_int(msg, 2, 2) / 10.0f;
    float roll = bytes_to_int(msg, 4, 2) / 10.0f;
    
    return std::make_tuple(yaw, pitch, roll);
}

// 0x30: Set UTC Time
bool SiyiSDK::set_utc_time(uint64_t timestamp)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(timestamp & 0xFF),
        static_cast<uint8_t>((timestamp >> 8) & 0xFF),
        static_cast<uint8_t>((timestamp >> 16) & 0xFF),
        static_cast<uint8_t>((timestamp >> 24) & 0xFF),
        static_cast<uint8_t>((timestamp >> 32) & 0xFF),
        static_cast<uint8_t>((timestamp >> 40) & 0xFF),
        static_cast<uint8_t>((timestamp >> 48) & 0xFF),
        static_cast<uint8_t>((timestamp >> 56) & 0xFF)
    };
    
    auto result = this->send_receive(1, CMD_ID::SET_UTC_TIME, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x31: Request Laser Rangefinder Status
bool SiyiSDK::get_lrf_status()
{
    auto result = this->send_receive(1, CMD_ID::GET_LRF_STATUS, {});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x32: Request Laser Rangefinder Status
bool SiyiSDK::enable_lrf()
{
    auto result = this->send_receive(1, CMD_ID::SET_LRF_STATUS, {1});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
bool SiyiSDK::disable_lrf()
{
    auto result = this->send_receive(1, CMD_ID::SET_LRF_STATUS, {0});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x34: Send Thermal RAW Data Command to Gimbal
// no idea what it even does ¯\_(ツ)_/¯

// 0x35: Request Temperature from the Thermal Imaging Camera Once
// again, no idea what it even does ¯\_(ツ)_/¯

// 0x37: Request Thermal Gain
std::optional<int> SiyiSDK::get_thermal_gain()
{
    auto result = this->send_receive(1, CMD_ID::GET_THERMAL_GAIN, {});
    if (!result) return false;
    auto& msg = *result;
    
    return bytes_to_int(msg, 0, 1, false);
}

// 0x38: Send Thermal Gain to Gimbal
bool SiyiSDK::set_low_thermal_gain()
{
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_GAIN, {0}, -1, 500);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x00);
}
bool SiyiSDK::set_high_thermal_gain()
{
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_GAIN, {1});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x39: Request Thermal Imaging Environmental Correction Parameters
std::optional<std::tuple<float, float, float, float, float>> SiyiSDK::get_thermal_correction_params()
{
    auto result = this->send_receive(1, CMD_ID::GET_THERMAL_CORRECT_PARAMS, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    float distance = bytes_to_int(msg, 0, 2, false) / 100.0f;
    float emission_rate = bytes_to_int(msg, 2, 2, false) / 100.0f;
    float humidity = bytes_to_int(msg, 4, 2, false) / 100.0f;
    float temp_atmo = bytes_to_int(msg, 6, 2, false) / 100.0f;
    float temp_reflect = bytes_to_int(msg, 8, 2, false) / 100.0f;
    
    return std::make_tuple(distance, emission_rate, humidity, temp_atmo, temp_reflect);
}

// 0x3a: Set Thermal Imaging Environmental Correction Parameters
bool SiyiSDK::set_thermal_correction_params(float distance, float emission_rate, float humidity, float temp_atmo, float temp_reflect)
{
    int16_t _distance = (int16_t)(distance * 100.0f);
    int16_t _emission_rate = (int16_t)(emission_rate * 100.0f);
    int16_t _humidity = (int16_t)(humidity * 100.0f);
    int16_t _temp_atmo = (int16_t)(temp_atmo * 100.0f);
    int16_t _temp_reflect = (int16_t)(temp_reflect * 100.0f);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(_distance & 0xFF), static_cast<uint8_t>((_distance >> 8) & 0xFF),
        static_cast<uint8_t>(_emission_rate & 0xFF), static_cast<uint8_t>((_emission_rate >> 8) & 0xFF),
        static_cast<uint8_t>(_humidity & 0xFF), static_cast<uint8_t>((_humidity >> 8) & 0xFF),
        static_cast<uint8_t>(_temp_atmo & 0xFF), static_cast<uint8_t>((_temp_atmo >> 8) & 0xFF),
        static_cast<uint8_t>(_temp_reflect & 0xFF), static_cast<uint8_t>((_temp_reflect >> 8) & 0xFF),
    };
    
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_CORRECT_PARAMS, data);
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x3B: Request Thermal Imaging Environmental Correction Switch
std::optional<bool> SiyiSDK::get_thermal_correction_status()
{
    auto result = this->send_receive(1, CMD_ID::GET_THERMAL_CORRECT, {});
    if (!result) return std::nullopt;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x3C: Set Thermal Imaging Environmental Correction Switch
bool SiyiSDK::enable_thermal_correction()
{
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_CORRECT, {1});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}
bool SiyiSDK::disable_thermal_correction()
{
    auto result = this->send_receive(1, CMD_ID::SET_THERMAL_CORRECT, {0});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x00);
}

// 0x3E: Send Raw GPS Data to Gimbal
bool SiyiSDK::set_raw_gps_data(int time_boot_ms, int lat, int lon, int alt, int alt_ellipsoid, int vn, int ve, int vd)
{
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(time_boot_ms & 0xFF), static_cast<uint8_t>((time_boot_ms >> 8) & 0xFF), static_cast<uint8_t>((time_boot_ms >> 16) & 0xFF), static_cast<uint8_t>((time_boot_ms >> 24) & 0xFF),
        static_cast<uint8_t>(lat & 0xFF), static_cast<uint8_t>((lat >> 8) & 0xFF), static_cast<uint8_t>((lat >> 16) & 0xFF), static_cast<uint8_t>((lat >> 24) & 0xFF),
        static_cast<uint8_t>(lon & 0xFF), static_cast<uint8_t>((lon >> 8) & 0xFF), static_cast<uint8_t>((lon >> 16) & 0xFF), static_cast<uint8_t>((lon >> 24) & 0xFF),
        static_cast<uint8_t>(alt & 0xFF), static_cast<uint8_t>((alt >> 8) & 0xFF), static_cast<uint8_t>((alt >> 16) & 0xFF), static_cast<uint8_t>((alt >> 24) & 0xFF),
        static_cast<uint8_t>(alt_ellipsoid & 0xFF), static_cast<uint8_t>((alt_ellipsoid >> 8) & 0xFF), static_cast<uint8_t>((alt_ellipsoid >> 16) & 0xFF), static_cast<uint8_t>((alt_ellipsoid >> 24) & 0xFF),
        static_cast<uint8_t>(vn & 0xFF), static_cast<uint8_t>((vn >> 8) & 0xFF), static_cast<uint8_t>((vn >> 16) & 0xFF), static_cast<uint8_t>((vn >> 24) & 0xFF),
        static_cast<uint8_t>(ve & 0xFF), static_cast<uint8_t>((ve >> 8) & 0xFF), static_cast<uint8_t>((ve >> 16) & 0xFF), static_cast<uint8_t>((ve >> 24) & 0xFF),
        static_cast<uint8_t>(vd & 0xFF), static_cast<uint8_t>((vd >> 8) & 0xFF), static_cast<uint8_t>((vd >> 16) & 0xFF), static_cast<uint8_t>((vd >> 24) & 0xFF),
    };
    
    auto result = this->send_only(1, CMD_ID::SET_THERMAL_CORRECT, data);
    return result;
}

// 0x48: Format SD Card
bool SiyiSDK::format_sd_card()
{
    auto result = this->send_receive(1, CMD_ID::FORMAT_SD_CARD, {});
    if (!result) return false;
    auto& msg = *result;
    
    return (msg[0] == 0x01);
}

// 0x4f: Manually Update Thermal Imaging Shutter (not working)
// bool SiyiSDK::reset_thermal_shutter()
// {
//     auto result = this->send_receive(1, CMD_ID::UPDATE_THERMAL_SHUTTER, {});
//     if (!result) return false;
//     auto& msg = *result;
    
//     return (msg[0] == 0x01);
// }

// 0x80: Gimbal Camera Soft Reboot (not working)
// bool SiyiSDK::soft_reboot()
// {
//     std::vector<uint8_t> data = {
//         1,
//         1
//     };
    
//     auto result = this->send_receive(1, CMD_ID::SOFT_REBOOT, data);
//     if (!result) return false;
//     auto& msg = *result;
    
//     return (msg[0] == 0x01 && msg[1] == 0x01);
// }

////////////////////////////////////
// CRC16 and message en-/decoding //
////////////////////////////////////

std::vector<uint8_t> SiyiSDK::encode_msg(int ctrl, int cmd_id, const std::vector<uint8_t>& data)
{
    int data_len = static_cast<int>(data.size());
    std::vector<uint8_t> msg(10 + data_len, 0);  // initialize with zeros
    
    // Starting mark
    msg[0] = 0x55;
    msg[1] = 0x66;
    
    // Control mark
    msg[2] = static_cast<uint8_t>(ctrl);
    
    // Data length
    msg[3] = data_len & 0xFF;         // Low-byte first
    msg[4] = (data_len >> 8) & 0xFF;  // High-byte
    
    // Sequence: can and will be ignored (left to 0x0000)
    
    // Command ID
    msg[7] = static_cast<uint8_t>(cmd_id);
    
    // Data
    std::copy(data.begin(), data.end(), msg.begin() + 8);
    
    // CRC16
    append_crc16(msg);
    
    return msg;
}

void SiyiSDK::append_crc16(std::vector<uint8_t>& msg)
{
    uint16_t crc = 0x0;
    
    for (size_t i = 0; i < msg.size() - 2; i++)  // exclude last 2 bytes (for CRC)
    {
        uint8_t byte = msg[i];
        crc = ((crc << 8) & 0xff00) ^ CRC16_LUT[((crc >> 8) & 0xff) ^ byte];
    }
    
    // Append CRC to the last two bytes
    msg[msg.size() - 2] = crc & 0xff;         // low byte
    msg[msg.size() - 1] = (crc >> 8) & 0xff;  // high byte
}

int SiyiSDK::bytes_to_int(const std::vector<uint8_t>& bytes, size_t offset, size_t length, bool is_signed)
{
    int value = 0;
    for (size_t i = 0; i < length; ++i) {
        value |= static_cast<int>(bytes[offset + i]) << (8 * i); // little-endian
    }
    if (is_signed) {
        // If the highest bit of the field is set, sign-extend
        int shift = (sizeof(int) - length) * 8;
        value = (value << shift) >> shift;
    }
    return value;
}

void SiyiSDK::append_float_to_bytes(std::vector<uint8_t>& buf, float value) {
    uint8_t bytes[sizeof(float)];
    std::memcpy(bytes, &value, sizeof(float));

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes, bytes + sizeof(float));
#endif

    buf.insert(buf.end(), bytes, bytes + sizeof(float));
}