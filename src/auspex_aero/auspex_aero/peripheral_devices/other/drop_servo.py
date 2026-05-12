

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from auspex_aero_msgs.action import CloseServo, OpenServo, DropPayload, ResetPayload

from ..device import PeripheralDevice

class DropServo(PeripheralDevice):
    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info("Starting DropServo...")
        
        first_key = next(iter(self.config))
        self.channel = self.config[first_key]["channel"]
        self._setup_sensor_action_clients()
        self._setup_action_server()
        
        self.get_logger().info("DropServo ready.")
    
    def _setup_sensor_action_clients(self):
        self._open_client = ActionClient(
            self,
            OpenServo,
            f"{self.platform_id}/fm/open_servo"
        )
        self._close_client = ActionClient(
            self,
            CloseServo,
            f"{self.platform_id}/fm/close_servo"
        )
    
    def _setup_action_server(self):
        self._drop_server = ActionServer(
            self,
            DropPayload,
            f"{self.platform_id}/{self.package_id}/drop_payload",
            self._execute_drop
        )
        self._reset_server = ActionServer(
            self,
            ResetPayload,
            f"{self.platform_id}/{self.package_id}/reset_payload",
            self._execute_reset
        )
    
    def _execute_drop(self, goal_handle):
        self.get_logger().info(f"Dropping payload...")
        
        self._open_client.wait_for_server()
        open_goal = OpenServo.Goal(channel=self.channel)
        success = self._open_client.send_goal(open_goal)
        
        if success:
            self.get_logger().info(f"Payload dropped.")
            goal_handle.succeed()
            return DropPayload.Result(success=True)
        else:
            self.get_logger().error(f"Failed to drop payload.")
            goal_handle.abort()
            return DropPayload.Result(success=False)
    
    def _execute_reset(self, goal_handle):
        self.get_logger().info(f"Resetting payload...")
        
        self._close_client.wait_for_server()
        close_goal = CloseServo.Goal(channel=self.channel)
        success = self._close_client.send_goal(close_goal)
        
        if success:
            self.get_logger().info(f"Payload reset.")
            goal_handle.succeed()
            return ResetPayload.Result(success=True)
        else:
            self.get_logger().error(f"Failed to reset payload.")
            goal_handle.abort()
            return ResetPayload.Result(success=False)

    
#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = DropServo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()