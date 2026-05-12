# This module defines the FlightManager class, which manages and supervises
# the GNC (Guidance, Navigation, and Control) system of the vehicle.

from dataclasses import dataclass
import math
import yaml
import threading
from typing import Any, Callable, cast
import sys
import logging

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from auspex_aero_msgs.action import ChangeAltitude, CircleLocation, CloseServo, Fly2D, Fly3D, FlyAboveGroundLevel, FlyStep3D, Hover, Idle, Land, OpenServo, Takeoff, Turn
from auspex_msgs.msg import PlatformState, BatteryState
from auspex_msgs.srv import GetOrigin, SetOrigin
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint

from ._types import VectorNED, PositionWGS84, FlightLimits, CheckpointTolerances, ControlSetpoint
from .fc_interface import FCInterface, FCControlPort, FCVehiclePort, FCType
from .guidance import BasicFlyTo
from .navigation import Navigation

#############################################################
# Constants                                                 #
#############################################################

# Publishing rates
PLATFORM_STATE_PUBLISH_RATE_HZ = 10.0

# Polling rates
WAIT_FOR_POLLING_RATE_HZ = 20.0
GUIDANCE_POLLING_RATE_HZ = 20.0

# Goal completion wait parameters
WAIT_FOR_ARMED_TIMEOUT_S = 5.0
WAIT_FOR_SETPOINT_CONTROL_MODE_TIMEOUT_S = 5.0
WAIT_FOR_TAKEOFF_TIMEOUT_S = 120.0
WAIT_FOR_LANDING_TIMEOUT_S = None
WAIT_FOR_FLY_3D_TIMEOUT_S = None
WAIT_DURATION_SERVO_S = 1.0


#############################################################
# Private data structures                                   #
#############################################################

@dataclass(frozen=True)
class ActionResult:
    success: bool = False
    cancelled: bool = False
    result: Any = None


#############################################################
# FlightManager Class                                       #
#############################################################

class FlightManager(Node):
    def __init__(self):
        super().__init__('flight_manager')
        self.declare_parameter('platform_config_path', "")
        self.declare_parameter('platform_id', "")
        self.declare_parameter('platform_number', 0)
        
        self.get_logger().info("Starting FlightManager...")
        
        # Action mutex
        self._action_mutex = threading.Lock()
        self._action_active = False
        self._action_allow_cancel = True
        
        # Setup
        self._read_config()
        self._setup_messages()
        self._setup_services()
        self._setup_action_servers()
        self._setup_gnc_modules()
        
        self.get_logger().info("FlightManager ready.")
    
    def shutdown(self) -> None:
        self.get_logger().info("Stopping FlightManager...")
        self._flight_controller.stop()
        self.get_logger().info("FlightManager stopped.")
    
    def _read_config(self):
        self._platform_id = self.get_parameter('platform_id').get_parameter_value().string_value
        platform_config_path = self.get_parameter('platform_config_path').get_parameter_value().string_value
        if not platform_config_path:
            raise RuntimeError("platform_config_path parameter not set!")
        with open(platform_config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # IP Adress
        self._platform_ip = config["network"]["vpn"]["ip"] if "vpn" in config["network"] else "127.0.0.1"
        self._team_id = config["general"]["team_id"] if "team_id" in config["general"] else "unknown_team"
        
        # FCType and address
        fctype_str = str(config["general"]["flight_controller"])
        match fctype_str.lower():
            case "px4":
                self._fc_type = FCType.PX4
                self._fc_address = config["general"]["flight_controller_address"]
            case "px4_simulated":
                self._fc_type = FCType.PX4
                self._fc_address = config["general"]["flight_controller_address"]
                self.port_offset = self.get_parameter('platform_number').get_parameter_value().integer_value
                if self.port_offset != 0:
                    base, port_str = self._fc_address.rsplit(':', 1)
                    self._fc_address = f"{base}:{int(port_str) + self.port_offset}"
            case "ardupilot":
                self._fc_type = FCType.ARDUPILOT
                self._fc_address = config["general"]["flight_controller_address"]
            case _:
                raise RuntimeError(f"Unsupported flight controller type in config: {fctype_str}")
        
        # Checkpoint tolerances
        self.checkpoint_tolerances = CheckpointTolerances(
            config["flight_parameters"]["checkpoint_tolerances"]["position_horizontal_m"],
            config["flight_parameters"]["checkpoint_tolerances"]["position_vertical_m"],
            config["flight_parameters"]["checkpoint_tolerances"]["velocity_total_m_s"],
            config["flight_parameters"]["checkpoint_tolerances"]["yaw_deg"]
        )
        
        # FC Limits
        self.fc_limits = FlightLimits(
            config["flight_parameters"]["flight_controller_limits"]["velocity_horizontal_m_s"],
            config["flight_parameters"]["flight_controller_limits"]["velocity_vertical_m_s"],
            config["flight_parameters"]["flight_controller_limits"]["acceleration_horizontal_m_s2"],
            config["flight_parameters"]["flight_controller_limits"]["acceleration_vertical_m_s2"],
            config["flight_parameters"]["flight_controller_limits"]["rate_yaw_deg_s"],
            config["flight_parameters"]["flight_controller_limits"]["acceleration_yaw_deg_s2"]
        )
    
    def _setup_messages(self) -> None:
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        ) # Only send the latest message, don't try again.
        self._platform_state_publisher = self.create_publisher(
            PlatformState,
            "/platform_state",
            qos_profile
        )
        self._platform_state_timer = self.create_timer(1.0 / PLATFORM_STATE_PUBLISH_RATE_HZ, self._publish_platform_state)
    
    def _setup_services(self) -> None:
        self._get_origin_service = self.create_service(
            GetOrigin,
            f"{self._platform_id}/get_origin",
            self._handle_get_origin
        )
        self._set_origin_service = self.create_service(
            SetOrigin,
            f"{self._platform_id}/set_origin",
            self._handle_set_origin
        )
    
    def _setup_action_servers(self) -> None:
        # Takeoff & Land
        self._takeoff_action_server = ActionServer(
            node = self,
            action_type = Takeoff,
            action_name = f"{self._platform_id}/fm/takeoff",
            execute_callback = self._execute_takeoff,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._land_action_server = ActionServer(
            node = self,
            action_type = Land,
            action_name = f"{self._platform_id}/fm/land",
            execute_callback = self._execute_land,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        # Fly
        self._turn_action_server = ActionServer(
            node = self,
            action_type = Turn,
            action_name = f"{self._platform_id}/fm/turn",
            execute_callback = self._execute_turn,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._change_altitude_action_server = ActionServer(
            node = self,
            action_type = ChangeAltitude,
            action_name = f"{self._platform_id}/fm/change_altitude",
            execute_callback = self._execute_change_altitude,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._hover_action_server = ActionServer(
            node = self,
            action_type = Hover,
            action_name = f"{self._platform_id}/fm/hover",
            execute_callback = self._execute_hover,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._idle_action_server = ActionServer(
            node = self,
            action_type = Idle,
            action_name = f"{self._platform_id}/fm/idle",
            execute_callback = self._execute_idle,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._circle_location_action_server = ActionServer(
            node = self,
            action_type = CircleLocation,
            action_name = f"{self._platform_id}/fm/circle_location",
            execute_callback = self._execute_circle_location,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._fly_2d_action_server = ActionServer(
            node = self,
            action_type = Fly2D,
            action_name = f"{self._platform_id}/fm/fly_2d",
            execute_callback = self._execute_fly_2d,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._fly_3d_action_server = ActionServer(
            node = self,
            action_type = Fly3D,
            action_name = f"{self._platform_id}/fm/fly_3d",
            execute_callback = self._execute_fly_3d,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._fly_agl_action_server = ActionServer(
            node = self,
            action_type = FlyAboveGroundLevel,
            action_name = f"{self._platform_id}/fm/fly_above_ground_level",
            execute_callback = self._execute_fly_agl,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._fly_step_3d_action_server = ActionServer(
            node = self,
            action_type = FlyStep3D,
            action_name = f"{self._platform_id}/fm/fly_step_3d",
            execute_callback = self._execute_fly_step_3d,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        # Servo
        self._open_servo_action_server = ActionServer(
            node = self,
            action_type = OpenServo,
            action_name = f"{self._platform_id}/fm/open_servo",
            execute_callback = self._execute_open_servo,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
        self._close_servo_action_server = ActionServer(
            node = self,
            action_type = CloseServo,
            action_name = f"{self._platform_id}/fm/close_servo",
            execute_callback = self._execute_close_servo,
            goal_callback = self._goal_callback,
            cancel_callback = self._cancel_callback
        )
    
    def _setup_gnc_modules(self) -> None:
        # Start interfaces
        self._flight_controller = FCInterface(self._fc_address, self._fc_type, self.port_offset)
        self._flight_controller.start()
        self._ctrl: FCControlPort = cast(FCControlPort, self._flight_controller)
        self._vhcl: FCVehiclePort = cast(FCVehiclePort, self._flight_controller)
        self._nav = Navigation(self._flight_controller)
        
        # Wait for GNSS origin location and set it in the navigation system
        try:
            self._nav.wait_for_gnss_origin_location()
        except Exception as e:
            self.get_logger().error(f"Failed to get GNSS origin location. FlightManager cannot operate without it.")
            raise
        self._nav.set_best_effort_origin()
    
    def _wait_for_gnss_origin_location(self) -> PositionWGS84:
        self.get_logger().info("Waiting for GNSS origin location...")
        rate = self.create_rate(WAIT_FOR_POLLING_RATE_HZ)
        while rclpy.ok():
            try:
                pos_ned = self._nav.get_position_ned()
                pos_wgs84 = self._nav.convert_ned_to_wgs84(pos_ned)
                self.get_logger().info(f"GNSS origin location obtained: {pos_wgs84}")
                return pos_wgs84
            except Exception as e:
                self.get_logger().warning(f"Error obtaining GNSS origin location: {e}")
            rate.sleep()
        raise RuntimeError("Failed to obtain GNSS origin location: ROS shutdown.")
    
    #############################################################
    # Messages                                                  #
    #############################################################
    
    def _publish_platform_state(self) -> None:
        msg = PlatformState()
        # ---- Header ----
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # ---- Platform info ----
        msg.platform_id = self._platform_id
        msg.platform_ip = self._platform_ip
        msg.team_id = self._team_id

        # ---- GPS position ----
        msg.platform_gps_position = GeoPoint()
        pos = self._nav.get_position_wgs84()
        msg.platform_gps_position.latitude = pos.latitude
        msg.platform_gps_position.longitude = pos.longitude
        msg.platform_gps_position.altitude = pos.altitude

        # ---- Pose ----
        msg.platform_pose = Pose()
        pos = self._nav.get_position_ned()
        msg.platform_pose.position.x = pos.north
        msg.platform_pose.position.y = pos.east
        msg.platform_pose.position.z = pos.down
        quat = self._nav.get_quaternion()
        msg.platform_pose.orientation.x = quat.x
        msg.platform_pose.orientation.y = quat.y
        msg.platform_pose.orientation.z = quat.z
        msg.platform_pose.orientation.w = quat.w
        yaw, pitch, roll = self._nav.get_euler_angles()
        msg.yaw_deg = yaw
        msg.pitch_deg = pitch
        msg.roll_deg = roll

        # ---- Platform status ----
        msg.platform_status = "AIRBORNE" if self._vhcl.is_in_air() else "LANDED"
        
        # ---- Battery state ----
        msg.battery_state = BatteryState()
        bat = self._vhcl.get_battery_state()
        msg.battery_state.voltage = bat.voltage_v
        msg.battery_state.current = bat.current_battery_a if not math.isnan(bat.current_battery_a) else -1.0
        msg.battery_state.charge = -1.0 # Not available from MAVSDK
        msg.battery_state.capacity = -1.0 # Not available from MAVSDK
        msg.battery_state.design_capacity = -1.0 # Not available from MAVSDK
        msg.battery_state.percentage = bat.remaining_percent / 100.0

        self._platform_state_publisher.publish(msg)
    
    #############################################################
    # Services                                                  #
    #############################################################
    
    def _handle_get_origin(self, request: GetOrigin.Request, response: GetOrigin.Response) -> GetOrigin.Response:
        origin = self._nav.get_origin()
        if origin is None:
            response.success = False
        else:
            response.success = True
            response.origin.latitude = origin.latitude
            response.origin.longitude = origin.longitude
            response.origin.altitude = origin.altitude
        return response
    
    def _handle_set_origin(self, request: SetOrigin.Request, response: SetOrigin.Response) -> SetOrigin.Response:
        print("[WARNING] SetOrigin service does nothing, because it is probably not needed anymore.")
        response.success = True
        return response
    
    #############################################################
    # Goal and cancel callback functions (actions)              #
    #############################################################
    
    # Accept and cancel callbacks
    def _goal_callback(self, goal_request) -> GoalResponse:
        with self._action_mutex:
            if self._action_active:
                return GoalResponse.REJECT
            else:
                self._action_active = True
                return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self._action_mutex:
            if self._action_allow_cancel:
                return CancelResponse.ACCEPT
            else:
                return CancelResponse.REJECT
    
    #############################################################
    # Actions                                                   #
    #############################################################
    
    # === Takeoff ===============================================

    def _execute_takeoff(self, goal_handle) -> Takeoff.Result:
        # Parse parameters
        takeoff_altitude_m = goal_handle.request.height_agl_m
        
        # Execute
        result = self._execute_action(
            action_name='takeoff',
            action_func=self._takeoff,
            action_func_kwargs={
                'takeoff_altitude_m': takeoff_altitude_m
            },
            publish_feedback_cb=None,
            goal_handle=goal_handle,
            allow_cancel=False,
            precon_in_air=False,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return Takeoff.Result(success=result.success)
    
    def _takeoff(self, takeoff_altitude_m: float) -> ActionResult:
        # Save position for later
        takeoff_position = VectorNED(
            self._nav.get_position_ned().north,
            self._nav.get_position_ned().east,
            self._nav.get_position_ned().down - takeoff_altitude_m
        )
        
        # Send takeoff command
        try:
            self._ctrl.takeoff(2.5)
        except Exception as e:
            self.get_logger().warning(f"Action \'takeoff\' failed: Exception during takeoff command: {e}")
            return ActionResult(success=False)
        
        # wait until takeoff complete
        def goal_condition() -> bool:
            position = self._nav.get_position_ned()
            return position.down <= -(2.5 - self.checkpoint_tolerances.position_vertical_m)
        if not self._wait_for(goal_condition, WAIT_FOR_TAKEOFF_TIMEOUT_S):
            self.get_logger().warning("Action \'takeoff\' failed: Timeout waiting for target altitude.")
            return ActionResult(success=False)
        
        # Fly to target altitude
        if not self._enter_offboard_mode():
            self.get_logger().warning(f"Action \'takeoff\' failed: Vehicle could not enter offboard mode.")
            return ActionResult(success=False)
        try:
            roll, pitch, yaw = self._nav.get_euler_angles()
            return self._fly_3d(takeoff_position, self.fc_limits.velocity_vertical_m_s, 0.0, yaw, None, lambda: False)
        finally:
            if not self._exit_offboard_mode():
                self.get_logger().warning(f"Action \'takeoff\' cleanup failed: Vehicle could not exit offboard mode.")
    
    
    # === Land ==================================================
    
    def _execute_land(self, goal_handle) -> Land.Result:
        # Execute
        result = self._execute_action(
            action_name='land',
            action_func=self._land,
            action_func_kwargs={},
            publish_feedback_cb=None,
            goal_handle=goal_handle,
            allow_cancel=False,
            precon_in_air=True,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=True
        )
        return Land.Result(success=result.success)
    
    def _land(self) -> ActionResult:
        # send command
        try:
            self._ctrl.land()
        except Exception as e:
            self.get_logger().warning(f"Action \'land\' failed: Exception during land command: {e}")
            return ActionResult(success=False)
        
        # wait until on ground
        def goal_condition() -> bool:
            in_air = self._vhcl.is_in_air()
            return in_air is False
        if not self._wait_for(goal_condition, WAIT_FOR_LANDING_TIMEOUT_S):
            self.get_logger().warning("Action \'land\' failed: Timeout waiting for vehicle to land.")
            return ActionResult(success=False)
        
        return ActionResult(success=True)
    
    
    # === Turn ==================================================
    
    def _execute_turn(self, goal_handle) -> Turn.Result:
        # Parse parameters
        yaw_deg = goal_handle.request.yaw_deg
        
        # Execute
        result = self._execute_action(
            action_name='turn',
            action_func=self._turn,
            action_func_kwargs={
                'yaw_deg': yaw_deg
            },
            publish_feedback_cb=None,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=True,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
    
        return Turn.Result(success=result.success)
    
    def _turn(self, yaw_deg: float, is_canceled_cb: Callable[[], bool]) -> ActionResult:
        position = self._nav.get_position_ned()
        setpoint = ControlSetpoint(
            position=position,
            yaw_deg=yaw_deg
        )
        self._ctrl.set_target_setpoint_ned(setpoint)
        
        rate = self.create_rate(GUIDANCE_POLLING_RATE_HZ)
        
        while rclpy.ok():
            if is_canceled_cb():
                self.get_logger().info("Action \'turn\' was cancelled.")
                return ActionResult(success=False, cancelled=True)
            
            _, _, current_yaw_deg = self._nav.get_euler_angles()
            if abs(yaw_deg % 360 - current_yaw_deg % 360) < self.checkpoint_tolerances.yaw_deg:
                return ActionResult(success=True)
            
            rate.sleep()
        
        return ActionResult(success=False)
    
    
    # === Change Altitude =======================================
    
    def _execute_change_altitude(self, goal_handle) -> ChangeAltitude.Result:
        # Parse parameters
        rel_alt_m = goal_handle.request.rel_alt_m
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(Fly3D.Feedback(progress_percent=progress_percent))
        
        # Execute
        result = self._execute_action(
            action_name='change_altitude',
            action_func=self._change_altitude,
            action_func_kwargs={
                'rel_alt_m': rel_alt_m
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=True,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
    
        return ChangeAltitude.Result(success=result.success)
    
    def _change_altitude(self, rel_alt_m: float, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        position = VectorNED(
            self._nav.get_position_ned().north,
            self._nav.get_position_ned().east,
            self._nav.get_position_ned().down - rel_alt_m
        )
        roll, pitch, yaw = self._nav.get_euler_angles()
        return self._fly_3d(position, self.fc_limits.velocity_vertical_m_s, 0.0, yaw, publish_feedback_cb, is_canceled_cb)
    
    
    # === Hover =================================================
    
    def _execute_hover(self, goal_handle) -> Hover.Result:
        # Parse parameters
        duration_s = goal_handle.request.duration_s
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(Hover.Feedback(progress_percent=progress_percent))
        
        # Execute
        result = self._execute_action(
            action_name='hover',
            action_func=self._hover,
            action_func_kwargs={
                'duration_s': duration_s
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return Hover.Result(success=result.success)
    
    def _hover(self, duration_s: float, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        # Send hover command
        try:
            self._ctrl.hold()
        except Exception as e:
            self.get_logger().warning(f"Action \'hover\' failed: Exception during hold command: {e}")
            return ActionResult(success=False)
        
        # wait until hover is complete
        rate = self.create_rate(GUIDANCE_POLLING_RATE_HZ)
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if is_canceled_cb():
                self.get_logger().info("Action \'hover\' was cancelled.")
                return ActionResult(success=False, cancelled=True)
            
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed_time >= duration_s:
                return ActionResult(success=True)
            
            publish_feedback_cb(100 * elapsed_time / duration_s)
            
            rate.sleep()
        
        return ActionResult(success=False)
    
    
    # === Idle ==================================================
    
    def _execute_idle(self, goal_handle) -> Idle.Result:
        # Parse parameters
        duration_s = goal_handle.request.duration_s
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(Idle.Feedback(progress_percent=progress_percent))
        
        # Execute
        result = self._execute_action(
            action_name='idle',
            action_func=self._idle,
            action_func_kwargs={
                'duration_s': duration_s
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return Idle.Result(success=result.success)
    
    def _idle(self, duration_s: float, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        # Same as hover
        return self._hover(duration_s, publish_feedback_cb, is_canceled_cb)
    
    
    # === Circle Location =======================================
    
    def _execute_circle_location(self, goal_handle) -> CircleLocation.Result:
        # Parse parameters
        position = PositionWGS84(latitude=goal_handle.request.target_lat_deg,
                                 longitude=goal_handle.request.target_lon_deg,
                                 altitude=goal_handle.request.target_alt_amsl_m)
        radius_m = goal_handle.request.radius_m
        speed_m_s = goal_handle.request.speed_m_s
        duration_s = goal_handle.request.duration_s
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(CircleLocation.Feedback(progress_percent=progress_percent))
        
        # Execute
        result = self._execute_action(
            action_name='circle_location',
            action_func=self._circle_location,
            action_func_kwargs={
                'position': position,
                'radius_m': radius_m,
                'speed_m_s': speed_m_s,
                'duration_s': duration_s
                },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return CircleLocation.Result(success=result.success)
    
    def _circle_location(self, position: PositionWGS84, radius_m: float, speed_m_s: float, duration_s: int, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        # Send orbit command
        try:
            self._ctrl.orbit(
                radius_m=radius_m,
                velocity_m_s=speed_m_s,
                center_position=position
            )
        except Exception as e:
            self.get_logger().warning(f"Action \'circle_location\' failed: Exception during circle command: {e}")
            return ActionResult(success=False)
        
        # wait until orbit is complete
        rate = self.create_rate(GUIDANCE_POLLING_RATE_HZ)
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if is_canceled_cb():
                self.get_logger().info("Action \'circle_location\' was cancelled.")
                return ActionResult(success=False, cancelled=True)
            
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed_time >= duration_s:
                return ActionResult(success=True)
            
            publish_feedback_cb(100 * elapsed_time / duration_s)
            
            rate.sleep()
        
        return ActionResult(success=False)
    
    
    # === Fly 2D ================================================
    
    def _execute_fly_2d(self, goal_handle) -> Fly2D.Result:
        # Parse parameters
        position = PositionWGS84(latitude=goal_handle.request.target_lat_deg,
                                 longitude=goal_handle.request.target_lon_deg,
                                 altitude=0.0)
        position = self._nav.convert_wgs84_to_ned(position)
        speed_m_s = goal_handle.request.speed_m_s
        heading_offset_deg = goal_handle.request.heading_offset_deg
        goal_yaw_deg = goal_handle.request.goal_yaw_deg
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(Fly2D.Feedback(progress_percent=progress_percent))
        
        result = self._execute_action(
            action_name='fly_2d',
            action_func=self._fly_2d,
            action_func_kwargs={
                'position': position,
                'speed_m_s': speed_m_s,
                'heading_offset_deg': heading_offset_deg,
                'goal_yaw_deg': goal_yaw_deg
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=True,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
    
        return Fly2D.Result(success=result.success)
    
    def _fly_2d(self, position: VectorNED, speed_m_s: float, heading_offset_deg: float, goal_yaw_deg: float, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        position = VectorNED(position.north, position.east, self._nav.get_position_ned().down)
        return self._fly_3d(position, speed_m_s, heading_offset_deg, goal_yaw_deg, publish_feedback_cb, is_canceled_cb)
    
    
    # === Fly 3D ================================================
    
    def _execute_fly_3d(self, goal_handle) -> Fly3D.Result:
        # Parse parameters
        position = PositionWGS84(latitude=goal_handle.request.target_lat_deg,
                                 longitude=goal_handle.request.target_lon_deg,
                                 altitude=goal_handle.request.target_alt_amsl_m)
        position = self._nav.convert_wgs84_to_ned(position)
        speed_m_s = goal_handle.request.speed_m_s
        heading_offset_deg = goal_handle.request.heading_offset_deg
        goal_yaw_deg = goal_handle.request.goal_yaw_deg
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(Fly3D.Feedback(progress_percent=progress_percent))
        
        result = self._execute_action(
            action_name='fly_3d',
            action_func=self._fly_3d,
            action_func_kwargs={
                'position': position,
                'speed_m_s': speed_m_s,
                'heading_offset_deg': heading_offset_deg,
                'goal_yaw_deg': goal_yaw_deg
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=True,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
    
        return Fly3D.Result(success=result.success)
    
    def _fly_3d(self, position: VectorNED, speed_m_s: float, heading_offset_deg: float, goal_yaw_deg: float, publish_feedback_cb: Callable[[float], None] | None, is_canceled_cb: Callable[[], bool]) -> ActionResult:
        guidance = BasicFlyTo(
            current_position=self._nav.get_position_ned(),
            current_velocity=self._nav.get_velocity_ned(),
            target_position=position,
            guidance_settings=self.checkpoint_tolerances,
            look_ahead_distance_m=10.0
        )
        
        rate = self.create_rate(GUIDANCE_POLLING_RATE_HZ)
        
        while rclpy.ok():
            setpoint = guidance.update(self._nav.get_position_ned(), self._nav.get_velocity_ned())
            
            if publish_feedback_cb:
                publish_feedback_cb(guidance.progress_percent())
            
            if is_canceled_cb():
                self.get_logger().info("Action \'fly_3d\' was cancelled.")
                return ActionResult(success=False, cancelled=True)
            
            if guidance.is_finished():
                return ActionResult(success=True)
            
            self._ctrl.set_target_setpoint_ned(setpoint)
            
            rate.sleep()
        
        return ActionResult(success=False)
    
    
    # === Fly Above Ground Level ================================
    
    def _execute_fly_agl(self, goal_handle):
        raise NotImplementedError("Action \'fly_agl\' is not implemented yet.")
    
    
    # === Fly Step 3D ===========================================
    
    def _execute_fly_step_3d(self, goal_handle):
        # Parse parameters
        position = PositionWGS84(latitude=goal_handle.request.target_lat_deg,
                                 longitude=goal_handle.request.target_lon_deg,
                                 altitude=goal_handle.request.target_alt_amsl_m)
        position = self._nav.convert_wgs84_to_ned(position)
        speed_m_s = goal_handle.request.speed_m_s
        heading_offset_deg = goal_handle.request.heading_offset_deg
        goal_yaw_deg = goal_handle.request.goal_yaw_deg
        
        def publish_feedback(progress_percent: float):
            with self._action_mutex:
                goal_handle.publish_feedback(FlyStep3D.Feedback(progress_percent=progress_percent))
        
        # Execute
        result = self._execute_action(
            action_name='fly_step_3d',
            action_func=self._fly_step_3d,
            action_func_kwargs={
                'position': position,
                'speed_m_s': speed_m_s,
                'heading_offset_deg': heading_offset_deg,
                'goal_yaw_deg': goal_yaw_deg
            },
            publish_feedback_cb=publish_feedback,
            goal_handle=goal_handle,
            allow_cancel=True,
            precon_in_air=True,
            offboard_mode=True,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
    
        return FlyStep3D.Result(success=result.success)
    
    def _fly_step_3d(self, position: VectorNED, speed_m_s: float, heading_offset_deg: float, goal_yaw_deg: float, publish_feedback_cb: Callable[[float], None], is_canceled_cb: Callable[[], bool]) -> ActionResult:
        diff = position - self._nav.get_position_ned()
        if diff.down > 0:
            self.get_logger().info("Target altitude is below current altitude.")
            # Fly horizontally first
            pub_feedback = lambda percent: publish_feedback_cb(percent / 2)
            result = self._fly_2d(position, speed_m_s, heading_offset_deg, goal_yaw_deg, pub_feedback, is_canceled_cb)
            if not result.success:
                return result
            # Fly vertically
            pub_feedback = lambda percent: publish_feedback_cb(50.0 + percent / 2)
            return self._fly_3d(position, speed_m_s, heading_offset_deg, goal_yaw_deg, pub_feedback, is_canceled_cb)
        else:
            self.get_logger().info("Target altitude is above current altitude.")
            # Fly vertically first
            pub_feedback = lambda percent: publish_feedback_cb(percent / 2)
            result = self._change_altitude(-diff.down, pub_feedback, is_canceled_cb)
            if not result.success:
                return result
            # Fly horizontally
            pub_feedback = lambda percent: publish_feedback_cb(50.0 + percent / 2)
            return self._fly_3d(position, speed_m_s, heading_offset_deg, goal_yaw_deg, pub_feedback, is_canceled_cb)
    
    
    # === Open Servo ============================================
    
    def _execute_open_servo(self, goal_handle):
        # Parse parameters
        servo_id = goal_handle.request.channel
        
        # Execute
        result = self._execute_action(
            action_name='open_servo',
            action_func=self._open_servo,
            action_func_kwargs={
                'servo_id': servo_id
            },
            publish_feedback_cb=None,
            goal_handle=goal_handle,
            allow_cancel=False,
            precon_in_air=None,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return OpenServo.Result(success=result.success)
    
    def _open_servo(self, servo_id: int) -> ActionResult:
        # Send command
        try:
            self._ctrl.set_actuator_value(servo_id, 1)
        except Exception as e:
            self.get_logger().warning(f"Action \'open_servo\' failed: Exception during set_actuator_value command: {e}")
            return ActionResult(success=False)
        
        self.get_clock().sleep_for(Duration(nanoseconds=int(WAIT_DURATION_SERVO_S * 1e9)))
        return ActionResult(success=True)
    
    
    # === Close Servo ===========================================
    
    def _execute_close_servo(self, goal_handle):
        # Parse parameters
        servo_id = goal_handle.request.channel
        
        # Execute
        result = self._execute_action(
            action_name='close_servo',
            action_func=self._close_servo,
            action_func_kwargs={
                'servo_id': servo_id
            },
            publish_feedback_cb=None,
            goal_handle=goal_handle,
            allow_cancel=False,
            precon_in_air=None,
            offboard_mode=False,
            flight_limits=self.fc_limits,
            disarm_after=False
        )
        return CloseServo.Result(success=result.success)
    
    def _close_servo(self, servo_id: int) -> ActionResult:
        # Send command
        try:
            self._ctrl.set_actuator_value(servo_id, 0)
        except Exception as e:
            self.get_logger().warning(f"Action \'close_servo\' failed: Exception during set_actuator_value command: {e}")
            return ActionResult(success=False)
        
        self.get_clock().sleep_for(Duration(nanoseconds=int(WAIT_DURATION_SERVO_S * 1e9)))
        return ActionResult(success=True)
    
    
    #############################################################
    # Helper Methods                                            #
    #############################################################
    
    def _execute_action(self,
                        action_name: str,
                        action_func: Callable[..., ActionResult],
                        action_func_kwargs: dict,
                        publish_feedback_cb: Callable[[Any], None] | None,
                        goal_handle,
                        allow_cancel: bool,
                        precon_in_air: bool | None,
                        offboard_mode: bool,
                        flight_limits: FlightLimits,
                        disarm_after: bool) -> ActionResult:
        # Action bookkeeping
        with self._action_mutex:
            # self._action_active = True # Set in goal callback
            self._action_allow_cancel = allow_cancel
        self.get_logger().info(f"Action \'{action_name}\' starting...")
        
        # Initialize flight session
        entered_offboard = False
        result = ActionResult(success=False)
        try:
            # Prerequisites (fail early)
            # In air
            if precon_in_air is not None:
                in_air = self._vhcl.is_in_air()
                if precon_in_air and (not in_air):
                    self.get_logger().warning(f"Action \'{action_name}\' prerequisite failed: Vehicle is not in air.")
                    return result
                if (not precon_in_air) and in_air:
                    self.get_logger().warning(f"Action \'{action_name}\' prerequisite failed: Vehicle is in air.")
                    return result
            # Arming
            is_armed = self._vhcl.is_armed()
            if not is_armed:
                if not self._arm_vehicle():
                    self.get_logger().warning(f"Action \'{action_name}\' prerequisite failed: Vehicle could not be armed.")
                    return result
            # Flight limits
            if not self._set_flight_limits(flight_limits):
                self.get_logger().warning(f"Action \'{action_name}\' prerequisite failed: Flight limits could not be set.")
                return result
            # Offboard mode
            if offboard_mode:
                if not self._enter_offboard_mode():
                    self.get_logger().warning(f"Action \'{action_name}\' prerequisite failed: Vehicle could not enter offboard mode.")
                    return result
                else:
                    entered_offboard = True
            
            # Execute the core action
            if publish_feedback_cb is not None:
                action_func_kwargs["publish_feedback_cb"] = publish_feedback_cb
            if allow_cancel:
                action_func_kwargs["is_canceled_cb"] = lambda: goal_handle.is_cancel_requested
            result = action_func(**action_func_kwargs)
            
        except Exception as e:
            # Log exceptions but continue
            self.get_logger().warning(f"Action \'{action_name}\' failed with exception: {e}")
            return result
        
        finally:
            # Cleanup flight session (fail silently, hope for the best)
            # Offboard mode
            if entered_offboard:
                if not self._exit_offboard_mode():
                    self.get_logger().warning(f"Action \'{action_name}\' cleanup failed: Vehicle could not exit offboard mode.")
            if self._vhcl.is_in_air():
                self._ctrl.hold()  # stop the vehicle
            # Disarming
            if disarm_after and result.success:
                if not self._disarm_vehicle():
                    self.get_logger().warning(f"Action \'{action_name}\' cleanup failed: Vehicle could not be disarmed.")
            
            # Cleanup action bookkeeping
            with self._action_mutex:
                if result.success:
                    goal_handle.succeed()
                else:
                    if result.cancelled:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                
                goal_handle = None
                self._action_active = False
                self._action_allow_cancel = True
            self.get_logger().info(f"Action \'{action_name}\' finished.")
        
        return result
    
    def _wait_for(self, condition_func, timeout_s: float | None, polling_rate_hz: float = WAIT_FOR_POLLING_RATE_HZ, cancel_callback=None) -> bool:
        rate = self.create_rate(polling_rate_hz)
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if cancel_callback is not None and cancel_callback():
                return False
            
            if condition_func():
                return True
            
            if timeout_s is not None:
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed_time >= timeout_s:
                    return False
            
            rate.sleep()
        
        return False
    
    # common mini-actions
    def _arm_vehicle(self) -> bool:
        if self._vhcl.is_armed():
            return True
        try:
            self._vhcl.arm()
        except Exception as e:
            self.get_logger().warning(f"Arming failed: {e}")
            return False
        return self._wait_for(self._vhcl.is_armed, timeout_s=WAIT_FOR_ARMED_TIMEOUT_S)
    
    def _disarm_vehicle(self) -> bool:
        if not self._vhcl.is_armed():
            return True
        try:
            self._vhcl.disarm()
        except Exception as e:
            self.get_logger().warning(f"Disarming failed: {e}")
            return False
        return self._wait_for(lambda: not self._vhcl.is_armed(), timeout_s=WAIT_FOR_ARMED_TIMEOUT_S)
    
    def _enter_offboard_mode(self) -> bool:
        try:
            self._ctrl.enter_setpoint_control_mode()
        except Exception as e:
            self.get_logger().warning(f"Entering offboard mode failed: {e}")
            return False
        return self._wait_for(lambda: self._ctrl.is_in_setpoint_control_mode(), timeout_s=WAIT_FOR_SETPOINT_CONTROL_MODE_TIMEOUT_S)
        return True
    
    def _exit_offboard_mode(self) -> bool:
        try:
            self._ctrl.exit_setpoint_control_mode()
        except Exception as e:
            self.get_logger().warning(f"Exiting offboard mode failed: {e}")
            return False
        return self._wait_for(lambda: not self._ctrl.is_in_setpoint_control_mode(), timeout_s=WAIT_FOR_SETPOINT_CONTROL_MODE_TIMEOUT_S)
        return True
    
    def _set_flight_limits(self, flight_limits: FlightLimits) -> bool:
        try:
            self._ctrl.set_flight_limits(flight_limits)
        except Exception as e:
            self.get_logger().warning(f"Setting flight limits failed: {e}")
            return False
        return True

def main(args=None):
    # Fix default python logging configuration (ROS2 messes with it and causes warnings to not show up)
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] [%(name)s]: %(message)s",
        stream=sys.stdout,
        force=True,
    )
    logging.getLogger("mavsdk_server").setLevel(logging.ERROR) # Reduce mavsdk spam
    
    rclpy.init(args=args)
    node = FlightManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()