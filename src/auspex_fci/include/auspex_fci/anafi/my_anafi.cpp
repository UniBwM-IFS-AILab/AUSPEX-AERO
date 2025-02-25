#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/qos.hpp>

#include "anafi_ros_interfaces/msg/move_to_command.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

#include <GeographicLib/Geodesic.hpp>
#include <vector>


class AnafiBridgeNode : public rclcpp::Node
{
public:
    AnafiBridgeNode()
        : Node("anafi_offboard"), _last_logged_altitude(-1.0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("drone_prefix", "anafi");
        this->_drone_prefix = this->get_parameter("drone_prefix").as_string();
        this->_altitude = 0.0;
        this->_position = std::make_pair(0.0, 0.0);


        // Publishers
        pub_move_to_ = this->create_publisher<anafi_ros_interfaces::msg::MoveToCommand>("anafi/drone/moveto", 10);
        // Subscribers
        sub_altitude_ = this->create_subscription<std_msgs::msg::Float32>(
            "/anafi/drone/altitude", rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&AnafiBridgeNode::altitude_changed_callback, this, std::placeholders::_1));

        sub_position_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/anafi/drone/position", rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&AnafiBridgeNode::position_changed_callback, this, std::placeholders::_1));

    
        // Service clients
        arm_drone_client_ = this->create_client<std_srvs::srv::SetBool>("/anafi/drone/arm");
        takeoff_client_ = this->create_client<std_srvs::srv::Trigger>("/anafi/drone/takeoff");

        landing_client_ = this->create_client<std_srvs::srv::Trigger>("anafi/drone/land");

        RCLCPP_INFO(this->get_logger(), "Node initialized and waiting for services...");
        

        target_latitude_ = 45.1351;
        target_longitude_ = 110.5820;
        target_altitude_ = 100.0;

        RCLCPP_INFO(this->get_logger(), "AnafiBridgeNode initialized.");

    }

    const double earth_radius_km = 6371.0;
    
    //For calculating the distance between the current position and target location
    double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
        // Convert latitude and longitude from degrees to radians
        lat1 = lat1 * M_PI / 180.0;
        lon1 = lon1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;
        lon2 = lon2 * M_PI / 180.0;

        // Haversine formula
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;

        double a = std::pow(std::sin(dlat / 2), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        // Distance in meters
        return earth_radius_km * c * 1000; // Convert km to meters
    }

    //To move to a given coordinate location
    void moveTo_callback(double latitude, double longitude, double altitude)
    {
        auto move_to_msg = anafi_ros_interfaces::msg::MoveToCommand();
        move_to_msg.latitude = latitude;
        move_to_msg.longitude = longitude;
        move_to_msg.altitude = altitude;      
        move_to_msg.orientation_mode = 1;      
        
        double distance = haversine_distance(latitude,longitude,current_latitude_,current_longitude_);
        move_to_msg.heading = 0.0;             

        pub_move_to_->publish(move_to_msg);

        RCLCPP_INFO(this->get_logger(), "Current distance calculated:%f", distance );
		
  
        while(distance>1)
        {

            rclcpp::spin_some(shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Current distance calculated:%f", distance );
            distance = haversine_distance(latitude,longitude,current_latitude_,current_longitude_);
                 std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Control frequency

        }
        
        RCLCPP_INFO(this->get_logger(), "MoveTo command sent successfully. Target position: Lat: %f, Lon: %f, Alt: %f", latitude, longitude, altitude);

    }



private:
    std::string _drone_prefix;
    double _altitude;
    double _last_logged_altitude;
    std::pair<double, double> _position;

    // Declaring target location variables
    double target_latitude_;
    double target_longitude_;
    double target_altitude_;

    double current_latitude_;
    double current_longitude_;    

    rclcpp::Publisher<anafi_ros_interfaces::msg::MoveToCommand>::SharedPtr pub_move_to_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_altitude_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_position_;
    rclcpp::Subscription<anafi_ros_interfaces::msg::MoveToCommand>::SharedPtr sub_move_to_;
    
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_drone_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr landing_client_;
    rclcpp::TimerBase::SharedPtr move_timer_;

    // Callbacks
    void altitude_changed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        _altitude = msg->data;

        // Only log the altitude if it changes significantly
        if (std::abs(_altitude - _last_logged_altitude) > 0.1) {
            //RCLCPP_INFO(this->get_logger(), "Altitude updated to: %f", _altitude);
            _last_logged_altitude = _altitude;
        }
    }

    void position_changed_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        current_latitude_ = msg->point.x;
        current_longitude_ = msg->point.y;
        //RCLCPP_INFO(this->get_logger(), "Position changed to: [%f, %f]",current_latitude_, current_longitude_);
        
    }

    public:
    // Takeoff Action Execution
    
    void execute_takeoff_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "Starting takeoff sequence...");

        // Arm the drone
        while (!arm_drone_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /anafi/drone/arm service...");
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;  // Arm the drone
        auto future = arm_drone_client_->async_send_request(request);

        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Drone armed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone, but continuing to takeoff...");
        }

        // Takeoff
        while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /anafi/drone/takeoff service...");
        }

        auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future_takeoff = takeoff_client_->async_send_request(trigger_request);

        // Monitor altitude during takeoff
        RCLCPP_INFO(this->get_logger(), "Waiting for the drone to reach 0.9m altitude...");
        while (_altitude < 0.9) {
            //JUST CHANGED NOW
            rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_takeoff);
        }

        // At this point, assume the drone is hovering
        RCLCPP_INFO(this->get_logger(), "Drone is hovering, proceeding with MoveTo command...");

        RCLCPP_INFO(this->get_logger(), "Proceeding with moveTo command...");


        moveTo_callback(48.8807,2.369687,5);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveTo_callback(48.8808,2.369687,5); //does not work to call second point,

        RCLCPP_INFO(this->get_logger(), "Landing the drone...");
        auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto landing = landing_client_->async_send_request(trigger_request);

        rclcpp::spin_until_future_complete(this->get_node_base_interface(), landing);

        RCLCPP_INFO(this->get_logger(), "Drone landed successfully.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnafiBridgeNode>();

    auto move_to_msg = std::make_shared<anafi_ros_interfaces::msg::MoveToCommand>();    

    // node->moveTo_callback(move_to_msg);
    node->execute_takeoff_sequence();



    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
