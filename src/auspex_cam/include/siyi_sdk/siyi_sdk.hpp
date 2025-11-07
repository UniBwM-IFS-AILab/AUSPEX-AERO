#ifndef SIYI_SDK
#define SIYI_SDK

#include <vector>       // for std::vector
#include <string>       // for std::string
#include <cstdint>      // for uint16_t
#include <netinet/in.h> // for struct sockaddr_in (POSIX sockets)
#include <mutex>        // for locks
#include <unordered_map>
#include <future>
#include <atomic>
#include <thread>
#include <optional>

class SiyiSDK
{
public:
    SiyiSDK();
    ~SiyiSDK();
    
    // Camera
    bool set_auto_focus(int x, int y);
    std::optional<float> manual_zoom_in();   // you probably shouldn't use manual zoom and use set_zoom instead
    std::optional<float> manual_zoom_out();  // you probably shouldn't use manual zoom and use set_zoom instead
    std::optional<float> manual_zoom_stop(); // you probably shouldn't use manual zoom and use set_zoom instead
    bool manual_focus_far();  // you probably shouldn't use manual focus and use set_auto_focus instead
    bool manual_focus_near(); // you probably shouldn't use manual focus and use set_auto_focus instead
    bool manual_focus_stop(); // you probably shouldn't use manual focus and use set_auto_focus instead
    std::optional<std::tuple<int, int, int, int, int>> get_camera_config(); // returns {record_sta, gimbal_motion_mode, gimbal_mouting_dir, hdmi_or_cvbs, zoom_linkage} (refer to manual)
    bool take_picture();
    bool start_stop_recording();
    bool set_zoom(float zoom);
    std::optional<int> get_image_mode(); // returns mode (refer to manual)
    std::optional<int> set_image_mode(int mode); // returns mode (refer to manual)
    std::optional<float> get_max_zoom_value(); // returns max_zoom
    std::optional<float> get_current_zoom_value(); // returns zoom
    // command might not exist: std::optional<std::tuple<int, int, int, int, int>> get_stream_codec_specs(int stream); // returns {encoding, res_w, res_h, bitrate, framerate} (refer to manual)
    // command might not exist: bool set_stream_codec_specs(int stream, int enc, int res_w, int res_h, int bitrate);
    
    // Thermal
    std::optional<float> get_temp_point(int x, int y); // WARNING: blocks for long time (~500ms)! returns temperature
    bool start_temp_point_caching(int x, int y);
    bool stop_temp_point_caching(int x, int y);
    std::optional<std::tuple<float, int, int>> get_cached_temp_point(); // returns {temperature, x, y}
    std::optional<std::tuple<float, int, int, float, int, int>> get_temp_range_box(int x0, int y0, int x1, int y1); // WARNING: blocks for long time (~500ms)! returns {temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y}
    bool start_temp_range_box_caching(int x0, int y0, int x1, int y1);
    bool stop_temp_range_box_caching(int x0, int y0, int x1, int y1);
    std::optional<std::tuple<int, int, int, int, float, int, int, float, int, int>> get_cached_temp_range_box(); // returns {x0, y0, x1, y1, temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y}
    std::optional<std::tuple<float, int, int, float, int, int>> get_temp_range_global(); // WARNING: blocks for long time (~500ms)! returns {temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y}
    bool start_temp_range_global_caching();
    bool stop_temp_range_global_caching();
    std::optional<std::tuple<float, int, int, float, int, int>> get_cached_temp_range_global(); // returns {temp_max, temp_max_x, temp_max_y, temp_min, temp_min_x, temp_min_y}
    std::optional<int> get_thermal_color(); // returns color_palette (refer to manual)
    std::optional<int> set_thermal_color(int color); // returns color_palette (refer to manual)
    std::optional<int> get_thermal_gain(); // returns sensitivity (0: low gain, 1: high gain)
    bool set_low_thermal_gain(); // WARNING: blocks for long time (~1000ms)!
    bool set_high_thermal_gain(); // WARNING: blocks for long time (~1000ms)!
    std::optional<std::tuple<float, float, float, float, float>> get_thermal_correction_params(); // returns {distance, emission_rate, humidity, temp_atmo, temp_reflect}
    bool set_thermal_correction_params(float distance, float emission_rate, float humidity, float temp_atmo, float temp_reflect);
    std::optional<bool> get_thermal_correction_status();
    bool enable_thermal_correction();
    bool disable_thermal_correction();
    // bool reset_thermal_shutter(); // WARNING: not working for some reason!
    
    // Gimbal
    bool set_gimbal_rotation_speed(int yaw_rate, int pitch_rate); // yaw+ = right, pitch+ = up; both: [-100,+100]
    bool center();
    bool set_gimbal_mode_lock();
    bool set_gimbal_mode_follow();
    bool set_gimbal_mode_fpv();
    bool set_gimbal_mode_tilt_downward();
    bool set_gimbal_mode_linked_zoom();
    std::optional<std::tuple<float, float, float, float, float, float>> get_gimbal_attitude(); // returns {yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel}
    std::optional<std::tuple<float, float, float>> set_control_angle(float yaw, float pitch); // returns {yaw, pitch, roll}
    bool start_gimbal_attitude_caching(int frequency); // frequency (1: 2Hz, 2: 4Hz, 3: 5Hz, 4: 10Hz, 5: 20Hz, 6: 50Hz, 7: 100Hz)
    bool stop_gimbal_attitude_caching();
    std::optional<std::tuple<float, float, float, float, float, float>> get_cached_gimbal_attitude(); // returns {yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel}
    std::optional<int> get_present_working_mode(); // returns mode (0: lock, 1: follow, 2: fpv)
    std::optional<std::tuple<float, float, float>> get_gimbal_raw_angles(); // returns: yaw, pitch, roll
    
    // Laser Range Finder
    std::optional<float> get_lrf_dist(); // returns distance (in meters)
    std::optional<std::tuple<float, float>> get_lrf_lat_lon(); // WARNING: only works if the camera gets attitude from FC! returns {latitude, longitude}
    bool get_lrf_status();
    bool enable_lrf();
    bool disable_lrf();
    bool start_lrf_dist_caching();
    bool stop_lrf_dist_caching();
    std::optional<float> get_cached_lrf_dist(); // returns distance (in meters)
    
    // Other
    std::optional<std::tuple<std::string, std::string, std::string>> get_firmware_version(); // returns versions: {camera, gimbal, zoom}
    std::optional<std::string> get_hardware_id();
    bool set_aircraft_attitude(int time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
    bool set_utc_time(uint64_t timestamp);
    bool set_raw_gps_data(int time_boot_ms, int lat, int lon, int alt, int alt_ellipsoid, int vn, int ve, int vd);
    bool format_sd_card();
    // bool soft_reboot(); // WARNING: not working for some reason!
    
private:
    // Socket stuff
    const std::string camera_ip_ = "192.168.144.25";
    const uint16_t camera_port_ = 37260;
    struct sockaddr_in camera_addr_;
    socklen_t addrLen_;
    const std::string local_ip_ = "192.168.144.13";
    struct sockaddr_in local_addr_;
    std::atomic<int> sock_fd_ = -1;
    int default_send_receive_retries_ = 3;
    int default_recv_timeout_ms_ = 100;
    int heartbeat_interval_ms_ = 500;
    
    // Threading stuff
    std::atomic<bool> running_;
    std::thread recv_thread_;
    std::thread heartbeat_thread_;
    std::unordered_map<int, std::promise<std::vector<uint8_t>>> pending_;
    std::mutex pending_mutex_;
    std::unordered_map<int, std::vector<uint8_t>> cache_;
    std::mutex cache_mutex_;
    std::atomic<bool> connected_{false};
    
    
    bool send_tcp_heartbeat();
    void heartbeatLoop();
    void receiveLoop();
    std::vector<uint8_t> encode_msg(int ctrl, int cmd_id, const std::vector<uint8_t>& data);
    int bytes_to_int(const std::vector<uint8_t>& bytes, size_t offset, size_t length, bool is_signed = true);
    void append_float_to_bytes(std::vector<uint8_t>& buf, float value);
    std::optional<std::vector<uint8_t>> send_receive(int ctrl, int cmd_id, const std::vector<uint8_t>& data, int retries = -1, int timeout_ms = -1);
    bool send_only(int ctrl, int cmd_id, const std::vector<uint8_t>& data);
    void append_crc16(std::vector<uint8_t>& msg);
    void fail_all_pending(const std::string& reason);
    bool init_socket();
    void close_socket();
    bool reconnect_blocking();
    bool send_all(const std::vector<uint8_t>& buf);
};

#endif  // SIYI_SDK