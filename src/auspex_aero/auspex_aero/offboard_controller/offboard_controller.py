# This module defines the OffboardController class.
# It receives lists of tasks from AUSPEX-EXEC, buffers them and executes them one-by-one.
# It distributes tasks to the appropriate modules (e.g. flight manager, camera controller, etc.) and combines feedback.

from typing import Any, Dict, Tuple
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from auspex_utils.action_conversion import *
from auspex_msgs.action import ExecuteSequence
from auspex_msgs.msg import PlatformCapabilities
from auspex_aero_msgs.action import ChangeAltitude, CircleLocation, CloseServo, Fly2D, Fly3D, FlyAboveGroundLevel, FlyStep3D, Hover, Idle, Land, OpenServo, Takeoff, Turn
from auspex_aero_msgs.srv import PauseExecution, ResumeExecution

from ._utils import *

#############################################################
# Constants                                                 #
#############################################################

MAIN_LOOP_RATE_HZ = 2.0
CAPABILITIES_PUBLISH_RATE_HZ = 0.2


#############################################################
# Offboard Controller Class                                 #
#############################################################

class OffboardController(Node):
    def __init__(self) -> None:
        super().__init__('offboard_controller')
        self.declare_parameter('platform_config_path', "")
        self.declare_parameter('platform_id', "")
        
        self.get_logger().info("Starting OffboardController...")
        
        # State variables
        self._execution: ExecutionState | None = None
        self._fm_action: ChildAction | None = None
        
        # Setup
        self._read_config()
        self._setup_action_server()
        self._setup_pause_resume_services()
        self._setup_action_clients()
        self._start_capabilities_publisher()
        self._start_main_loop()
        
        self.get_logger().info("OffboardController ready.")
    
    def shutdown(self):
        self.get_logger().info("Shutting down OffboardController...")
        self._stop_event.set()
        self.get_logger().info("OffboardController shutdown complete.")
    
    def _read_config(self):
        self.platform_id = self.get_parameter('platform_id').get_parameter_value().string_value
        platform_config_path = self.get_parameter('platform_config_path').get_parameter_value().string_value
        if not platform_config_path:
            raise RuntimeError("platform_config_path parameter not set!")
        with open(platform_config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.model = config.get("general", {}).get("model", "unknown")
        self.max_flight_duration_s = config.get("flight_parameters", {}).get("general", {}).get("max_flight_duration_s", 0)
        self.max_flight_height_m = config.get("flight_parameters", {}).get("general", {}).get("max_flight_height_m", 0)
        self.max_velocity_m_s = config.get("flight_parameters", {}).get("flight_controller_limits", {}).get("velocity_horizontal_m_s", 0)
    
    def _setup_action_server(self):
        self.server_cbg = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            node = self,
            action_type = ExecuteSequence,
            action_name = f"{self.platform_id}/action_sequence",
            goal_callback = self._goal_callback,
            handle_accepted_callback = self._accepted_callback,
            execute_callback = self._execute_callback,
            cancel_callback = self._cancel_callback,
            callback_group=self.server_cbg
        )
        self.get_logger().info(f"Action server '{self.platform_id}/action_sequence' is ready.")
    
    def _setup_pause_resume_services(self):
        self.pause_resume_cbg = MutuallyExclusiveCallbackGroup()
        self._pause_service = self.create_service(
            srv_type = PauseExecution,
            srv_name = f"{self.platform_id}/action_sequence/pause",
            callback = self._pause,
            callback_group=self.pause_resume_cbg
        )
        self._resume_service = self.create_service(
            srv_type = ResumeExecution,
            srv_name = f"{self.platform_id}/action_sequence/resume",
            callback = self._resume,
            callback_group=self.pause_resume_cbg
        )
        self.get_logger().info(f"Services '{self.platform_id}/action_sequence/pause' and '{self.platform_id}/action_sequence/resume' are ready.")
    
    def _setup_action_clients(self):
        self.client_cbg = ReentrantCallbackGroup()
        self._action_clients: Dict[str, ActionClient] = {}
        self._setup_flight_manager_action_clients()
        self._setup_sensor_action_clients()
    
    def _setup_flight_manager_action_clients(self):
        self._action_clients['change_altitude'] = ActionClient(self, ChangeAltitude, f"{self.platform_id}/fm/change_altitude", callback_group=self.client_cbg)
        self._action_clients['circle_location'] = ActionClient(self, CircleLocation, f"{self.platform_id}/fm/circle_location", callback_group=self.client_cbg)
        self._action_clients['close_servo'] = ActionClient(self, CloseServo, f"{self.platform_id}/fm/close_servo", callback_group=self.client_cbg)
        self._action_clients['fly_2d'] = ActionClient(self, Fly2D, f"{self.platform_id}/fm/fly_2d", callback_group=self.client_cbg)
        self._action_clients['fly_3d'] = ActionClient(self, Fly3D, f"{self.platform_id}/fm/fly_3d", callback_group=self.client_cbg)
        self._action_clients['fly_agl'] = ActionClient(self, FlyAboveGroundLevel, f"{self.platform_id}/fm/fly_agl", callback_group=self.client_cbg)
        self._action_clients['fly_step_3d'] = ActionClient(self, FlyStep3D, f"{self.platform_id}/fm/fly_step_3d", callback_group=self.client_cbg)
        self._action_clients['hover'] = ActionClient(self, Hover, f"{self.platform_id}/fm/hover", callback_group=self.client_cbg)
        self._action_clients['idle'] = ActionClient(self, Idle, f"{self.platform_id}/fm/idle", callback_group=self.client_cbg)
        self._action_clients['land'] = ActionClient(self, Land, f"{self.platform_id}/fm/land", callback_group=self.client_cbg)
        self._action_clients['open_servo'] = ActionClient(self, OpenServo, f"{self.platform_id}/fm/open_servo", callback_group=self.client_cbg)
        self._action_clients['takeoff'] = ActionClient(self, Takeoff, f"{self.platform_id}/fm/takeoff", callback_group=self.client_cbg)
        self._action_clients['turn'] = ActionClient(self, Turn, f"{self.platform_id}/fm/turn", callback_group=self.client_cbg)
        
        self.get_logger().info(f"Action clients for flight manager are ready.")
    
    def _setup_sensor_action_clients(self):
        # TODO
        pass
    
    def _start_capabilities_publisher(self):
        self.cap_pub = self.create_publisher(
            PlatformCapabilities,
            f"platform_capabilities",
            10
        )
        def _publish_cap_callabck():
            msg = PlatformCapabilities()
            msg.platform_id = self.platform_id
            msg.model_info = self.model
            msg.platform_class = "uav"
            msg.max_flight_duration = self.max_flight_duration_s
            msg.max_flight_height = self.max_flight_height_m
            msg.max_velocity = self.max_velocity_m_s
            msg.turning_radius = 0.0
            
            self.cap_pub.publish(msg)
        
        self.cap_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.cap_timer = self.create_timer(
            1.0 / CAPABILITIES_PUBLISH_RATE_HZ,
            _publish_cap_callabck,
            callback_group=self.cap_timer_cbg
        )
    
    def _start_main_loop(self):
        self._stop_event = threading.Event()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()
        self._main_timer = self.create_timer(
            1.0 / MAIN_LOOP_RATE_HZ,
            self._main_loop,
            callback_group=self.timer_cbg
        )

    #############################################################
    # Action server and pause/resume callbacks                  #
    #############################################################
    
    def _goal_callback(self, goal_request):
        if self._execution is not None:
            self.get_logger().warn("Received new goal while another execution is in progress. Rejecting new goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def _accepted_callback(self, goal_handle):
        sequence = ActionSequence(goal_handle.request.actions)
        self.get_logger().info(f"New action sequence accepted: {sequence}")
        self._execution = ExecutionState(sequence)
        goal_handle.execute()
    
    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request for action sequence.")
        assert self._execution is not None
        
        self._execution.cancel()
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        assert self._execution is not None
        self._execution.start()
        
        while rclpy.ok() and not self._stop_event.is_set():
            if self._execution.status == ExecutionStatus.FINISHED:
                break
        
        result_type, result = self._execution.result
        self._execution = None
        
        match result_type:
            case ExecutionFinishType.SUCCESS:
                goal_handle.succeed()
                return ExecuteSequence.Result() # TODO: include some error info in the result message
            case ExecutionFinishType.CANCELLED:
                goal_handle.canceled()
                return ExecuteSequence.Result() # TODO: include some error info in the result message
            case ExecutionFinishType.FAILED:
                goal_handle.abort()
                return ExecuteSequence.Result() # TODO: include some error info in the result message
    
    def _pause(self, request, response):
        if self._execution is None:
            self.get_logger().warn("Received pause request but no execution is in progress.")
            response.success = False
            return response
        if self._execution.status != ExecutionStatus.EXECUTING:
            self.get_logger().warn("Received pause request but execution is not currently executing.")
            response.success = False
            return response
        self.get_logger().info(f"Pausing action sequence: {self._execution.sequence}")
        self._execution.pause()
        response.success = True
        return response
    
    def _resume(self, request, response):
        if self._execution is None:
            self.get_logger().warn("Received resume request but no execution is in progress.")
            response.success = False
            return response
        if self._execution.status != ExecutionStatus.PAUSED:
            self.get_logger().warn("Received resume request but execution is not currently paused.")
            response.success = False
            return response
        self.get_logger().info(f"Resuming action sequence: {self._execution.sequence}")
        self._execution.resume()
        response.success = True
        return response
    
    #############################################################
    # Action execution                                          #
    #############################################################
    
    def _main_loop(self):
        if self._execution is None:
            return
        
        match self._execution.status:
            case ExecutionStatus.IDLE:
                return
            case ExecutionStatus.EXECUTING:
                self._main_executing()
            case ExecutionStatus.PAUSED:
                self._main_paused()
            case ExecutionStatus.CANCELLING:
                self._main_cancelling()
            case ExecutionStatus.FINISHED:
                return
    
    def _main_executing(self):
        assert self._execution is not None
        
        if self._fm_action is None:
            if self._execution.sequence.is_finished:
                self.get_logger().info("Action sequence finished successfully.")
                self._execution.mark_successful()
                return
            current_action = self._execution.sequence.current_action
            client, goal_msg = self._get_client_and_goal(current_action)
            self.get_logger().info(f"Sending new action to flight manager: {current_action}")
            self._fm_action = ChildAction(client, goal_msg)
            self._fm_action.start()
            return
        
        match self._fm_action.state:
            case ActionExecutionStatus.FINISHED:
                self._action_finished()
            case _:
                return
    
    def _main_paused(self):
        assert self._execution is not None
        
        if self._fm_action is None:
            return
        
        match self._fm_action.state:
            case ActionExecutionStatus.FINISHED:
                self._action_finished()
            case ActionExecutionStatus.EXECUTING:
                self._fm_action.cancel()
            case _:
                return # Either to early to cancel, or already cancelled
    
    def _main_cancelling(self):
        assert self._execution is not None
        
        if self._fm_action is None:
            self._execution.mark_cancelled()
            return
        
        match self._fm_action.state:
            case ActionExecutionStatus.FINISHED:
                self._action_finished()
            case ActionExecutionStatus.EXECUTING:
                self._fm_action.cancel()
            case _:
                return # Either to early to cancel, or already cancelled
    
    def _action_finished(self):
        assert self._fm_action is not None
        assert self._execution is not None
        
        match self._fm_action.finish_type:
            case ActionFinishType.SUCCESS:
                self.get_logger().info(f"Action finished successfully: {self._execution.sequence.current_action}")
                self._execution.sequence.action_completed()
                self._fm_action = None
                self._main_loop() # immediately go into main loop again
            case ActionFinishType.CANCELED:
                self.get_logger().info(f"Action has cancelled: {self._execution.sequence.current_action}")
                self._fm_action = None
                self._main_loop() # immediately go into main loop again
            case ActionFinishType.FAILED:
                self.get_logger().error(f"Action failed: {self._execution.sequence.current_action}")
                self._fm_action = None
                self._execution.mark_failed()
                return
    
    #############################################################
    # Helper Functions                                          #
    #############################################################
    
    def _get_client_and_goal(self, action: BaseAction) -> Tuple[ActionClient, Any]:
        match action:
            case TakeOffAction():
                goal_msg = Takeoff.Goal(
                    height_agl_m = action.height_m_agl
                )
                return self._action_clients['takeoff'], goal_msg
            case LandAction():
                goal_msg = Land.Goal()
                return self._action_clients['land'], goal_msg
            case Fly2DAction():
                goal_msg = Fly2D.Goal(
                    target_lat_deg = action.to_lat,
                    target_lon_deg = action.to_lon,
                    speed_m_s = action.speed_m_s,
                    heading_offset_deg = action.heading_offset_deg,
                    goal_yaw_deg = action.goal_yaw_deg
                )
                return self._action_clients['fly_2d'], goal_msg
            case Fly3DAction():
                goal_msg = Fly3D.Goal(
                    target_lat_deg = action.to_lat,
                    target_lon_deg = action.to_lon,
                    target_alt_amsl_m = action.to_alt_m_amsl,
                    speed_m_s = action.speed_m_s,
                    heading_offset_deg = action.heading_offset_deg,
                    goal_yaw_deg = action.goal_yaw_deg
                )
                return self._action_clients['fly_3d'], goal_msg
            case FlyStep3DAction():
                goal_msg = FlyStep3D.Goal(
                    target_lat_deg = action.to_lat,
                    target_lon_deg = action.to_lon,
                    target_alt_amsl_m = action.to_alt_m_amsl,
                    speed_m_s = action.speed_m_s,
                    heading_offset_deg = action.heading_offset_deg,
                    goal_yaw_deg = action.goal_yaw_deg
                )
                return self._action_clients['fly_step_3d'], goal_msg
            case FlyAtDistance2GroundAction():
                goal_msg = FlyAboveGroundLevel.Goal(
                    target_lat_deg = action.to_lat,
                    target_lon_deg = action.to_lon,
                    distance_agl_m = action.distance_m,
                    speed_m_s = action.speed_m_s,
                    heading_offset_deg = action.heading_offset_deg,
                    goal_yaw_deg = action.goal_yaw_deg
                )
                return self._action_clients['fly_agl'], goal_msg
            case OpenServoAction():
                goal_msg = OpenServo.Goal(
                    channel = action.channel
                )
                return self._action_clients['open_servo'], goal_msg
            case CloseServoAction():
                goal_msg = CloseServo.Goal(
                    channel = action.channel
                )
                return self._action_clients['close_servo'], goal_msg
            case HoverAction():
                goal_msg = Hover.Goal(
                    duration_s = int(action.duration_ms / 1000.0)
                )
                return self._action_clients['hover'], goal_msg
            case IdleAction():
                goal_msg = Idle.Goal(
                    duration_s = int(action.duration_ms / 1000.0)
                )
                return self._action_clients['idle'], goal_msg
            case CircleGPSAction():
                goal_msg = CircleLocation.Goal(
                    target_lat_deg = action.lat,
                    target_lon_deg = action.lon,
                    target_alt_amsl_m = action.alt_m_amsl,
                    radius_m = action.radius_m,
                    speed_m_s = action.speed_m_s,
                    duration_s = int(action.duration_ms / 1000.0)
                )
                return self._action_clients['circle_location'], goal_msg
            case AscendAction():
                goal_msg = ChangeAltitude.Goal(
                    rel_alt_m = action.alt_m_amsl
                )
                return self._action_clients['change_altitude'], goal_msg
            case DescendAction():
                goal_msg = ChangeAltitude.Goal(
                    rel_alt_m = -action.alt_m_amsl
                )
                return self._action_clients['change_altitude'], goal_msg
            case TurnAction():
                goal_msg = Turn.Goal(
                    yaw_deg = action.yaw_deg,
                )
                return self._action_clients['turn'], goal_msg
            case StartDetectionAction():
                raise NotImplementedError("StartDetection action not implemented yet")
            case StopDetectionAction():
                raise NotImplementedError("StopDetection action not implemented yet")
            case CaptureImageAction():
                raise NotImplementedError("CaptureImage action not implemented yet")
            case SetWiFiOnAction():
                raise NotImplementedError("SetWiFiOn action not implemented yet")
            case SetWiFiOffAction():
                raise NotImplementedError("SetWiFiOff action not implemented yet")
            case _:
                raise ValueError(f"Unsupported action type: {type(action).__name__}")

#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()