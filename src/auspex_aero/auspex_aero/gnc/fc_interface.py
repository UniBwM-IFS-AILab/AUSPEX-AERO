# MavsdkInterface: A threaded, synchronous wrapper around mavsdk
# This combines navigation and control in the GNC system, since both happen onboard the FC.

import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import Protocol
import threading
import time
import typing
import logging
import copy
from concurrent.futures import Future, TimeoutError

import mavsdk
from mavsdk.action import OrbitYawBehavior
from mavsdk.offboard import PositionGlobalYaw, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw, AccelerationNed
from mavsdk.telemetry import FlightMode, GpsInfo, RawGps, Battery

from ._types import FlightLimits, ControlSetpoint, PositionWGS84, VectorNED, Quaternion, AltitudeType


#############################################################
# Constants                                                 #
#############################################################

# Timeouts
CONNECT_TIMEOUT_S = 30.0
COMMAND_TIMEOUT_S = 10.0
STOP_THREAD_TIMEOUT_S = 2.0
TELEMETRY_WAIT_TIMEOUT_S = 2.0
TELEMETRY_WAIT_RATE_HZ = 10.0

# Telemetry rates
TELEMETRY_RATE_FLIGHT_STATUS_HZ = 10.0
TELEMETRY_RATE_BATTERY_HZ = 1.0
TELEMETRY_RATE_POSE_HZ = 50.0

# Misc
ORBIT_YAW_BEHAVIOR = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER


#############################################################
# Private data structures                                   #
#############################################################

class FCType(Enum):
    PX4 = 0
    ARDUPILOT = 1

@dataclass
class VehicleState:
    armed: bool | None = None
    in_air: bool | None = None
    health_ok: bool | None = None
    flight_mode: FlightMode | None = None
    battery_state: Battery | None = None
    position: VectorNED | None = None
    velocity: VectorNED | None = None
    quaternion: Quaternion | None = None
    gps_info: GpsInfo | None = None
    raw_gps: RawGps | None = None
    gps_global_origin: PositionWGS84 | None = None


#############################################################
# Protocols (Split Interfaces)                              #
#############################################################

class FCVehiclePort(Protocol):
    # Arm / Disarm / Kill
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def kill(self) -> None: ...

    # Telemetry / state queries (note: FCNavigationPort already contains position/velocity/quaternion)
    def is_armed(self) -> bool: ...
    def is_in_air(self) -> bool: ...
    def is_health_ok(self) -> bool: ...
    def get_battery_state(self) -> Battery: ...
    def get_flight_mode(self) -> FlightMode: ...
    def get_raw_telemetry(self) -> VehicleState: ...

class FCNavigationPort(Protocol):
    def get_position(self) -> VectorNED: ...
    def get_velocity(self) -> VectorNED: ...
    def get_quaternion(self) -> Quaternion: ...
    def get_gps_info_blocking(self, timeout_s: float) -> GpsInfo: ...
    def get_raw_gps_position_blocking(self, timeout_s: float) -> PositionWGS84: ...
    def get_gps_global_origin(self) -> PositionWGS84: ...

class FCControlPort(Protocol):
    # Higher-level commands
    def takeoff(self, altitude_m: float) -> None: ...
    def land(self) -> None: ...
    def hold(self) -> None: ...
    def goto_location(self, position: PositionWGS84, yaw_deg: float) -> None: ...
    def orbit(self, radius_m: float, velocity_m_s: float, center_position: PositionWGS84 | None = None) -> None: ...
    def return_to_launch(self) -> None: ...

    # Offboard / setpoint control mode
    def enter_setpoint_control_mode(self) -> None: ...
    def exit_setpoint_control_mode(self) -> None: ...
    def is_in_setpoint_control_mode(self) -> bool: ...

    # Offboard setpoint setters (position/velocity/acc/combined)
    def set_target_setpoint_ned(self, setpoint: ControlSetpoint) -> None: ...
    
    def set_target_position_wgs(self, position: PositionWGS84, yaw_deg: float, altitude_type) -> None: ...
    def set_target_position_ned(self, position: VectorNED, yaw_deg: float) -> None: ...
    def set_target_velocity_ned(self, velocity: VectorNED, yaw_deg: float) -> None: ...
    def set_target_position_velocity_ned(self, position: VectorNED, velocity: VectorNED, yaw_deg: float) -> None: ...
    def set_target_position_velocity_acceleration_ned(self, position: VectorNED, velocity: VectorNED, acceleration: VectorNED, yaw_deg: float) -> None: ...

    # Other control commands
    def set_actuator_value(self, actuator_number: int, value: float) -> None: ...
    def set_flight_limits(self, flight_limits: FlightLimits) -> None: ...


#############################################################
# FCInterface Class                                         #
#############################################################

class FCInterface(FCVehiclePort, FCControlPort, FCNavigationPort):
    """A threaded, synchronous wrapper around mavsdk.
    
    Typical usage:
        fc = FCInterface("serial:///dev/ttyAMA0:921600")
        fc.start()
        ...
        fc.arm()
        fc.takeoff()
        ...
        fc.stop()
    """
    
    def __init__(self, system_address: str, fc_type: FCType, fci_number: int) -> None:
        """
        :param str system_address: The flight controllers system address.
        :param FCType fc_type: The type of flight controller.
        :param int fci_number: The number for the flight controller interface.
        """
        self._system_address = system_address
        self._fc_type = fc_type
        self._fci_number = fci_number
        self._logger = logging.getLogger("fc_interface")
        
        # asyncio loop and thread
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_ready_evt = threading.Event()
        self._thread: threading.Thread | None = None
        
        # MAVSDK system
        self._system: mavsdk.System | None = None
        
        # Telemetry
        self._telemetry_lock = threading.Lock()
        self._telemetry = VehicleState()
        self._telemetry_tasks: list[asyncio.Task] = []
    
    #############################################################
    # Lifecycle Methods                                         #
    #############################################################
    
    def start(self) -> None:
        """
        Start the background event loop and connect to the flight controller.

        :raises RuntimeError: If connection to the flight controller fails or times out.
        """
        
        self._logger.info("Starting FCInterface...")
        
        # Start the event loop in a separate thread
        if self._thread is not None:
            self._logger.warning("FCInterface was already started.")
            return  # Already started
        self._loop_ready_evt.clear()
        self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._thread.start()
        
        # Wait until the event loop is running
        self._loop_ready_evt.wait()
        
        # Connect to the flight controller
        self._logger.info(f"Connecting to flight controller at {self._system_address}...")
        assert self._loop is not None
        future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        try:
            future.result(timeout=CONNECT_TIMEOUT_S)
        except TimeoutError:
            self._logger.error(f"Failed to connect to flight controller within {CONNECT_TIMEOUT_S} seconds.")
            self.stop()
            raise RuntimeError("Failed to connect to the flight controller within the timeout period.")
        except Exception as e:
            self._logger.error("Failed to connect to flight controller.")
            self.stop()
            raise RuntimeError("Failed to connect to the flight controller.") from e
        self._logger.info("Connected to flight controller.")
    
    def stop(self) -> None:
        """
        Stop the background event loop and clean up.
        """
        
        self._logger.info("Stopping FCInterface...")
        if self._loop is not None:
            for task in self._telemetry_tasks:
                self._loop.call_soon_threadsafe(task.cancel)
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(STOP_THREAD_TIMEOUT_S)
        self._thread = None
        self._system = None
        self._telemetry_tasks = []
        self._logger.info("FCInterface stopped.")
    
    def _run_event_loop(self) -> None:
        # Create and run the event loop
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop_ready_evt.set()
        try:
            self._loop.run_forever()
        finally:
            # cancel pending tasks and close
            pending = asyncio.all_tasks(self._loop)
            for t in pending:
                t.cancel()
            try:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self._loop.close()
            self._loop = None
    
    async def _connect(self) -> None:
        self._system = mavsdk.System(port=50051 + self._fci_number)
        assert self._system is not None
        await self._system.connect(system_address=self._system_address)
        
        async for state in self._system.core.connection_state():
            if state.is_connected:
                break
        
        await self._start_telemetry_subscriptions()
    
    async def _start_telemetry_subscriptions(self) -> None:
        self._telemetry_tasks = [
            asyncio.create_task(self._telemetry_subscribe_armed()),
            asyncio.create_task(self._telemetry_subscribe_in_air()),
            asyncio.create_task(self._telemetry_subscribe_health()),
            asyncio.create_task(self._telemetry_subscribe_flight_mode()),
            asyncio.create_task(self._telemetry_subscribe_battery()),
            asyncio.create_task(self._telemetry_subscribe_pos_vel()),
            asyncio.create_task(self._telemetry_subscribe_quaternion()),
            asyncio.create_task(self._telemetry_subscribe_gps_info()),
            asyncio.create_task(self._telemetry_subscribe_raw_gps())
        ]
    
    #############################################################
    # Public API (commands)                                     #
    #############################################################
    
    # Arm, disarm, kill
    def arm(self) -> None:
        """Arm the vehicle.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.arm_async())
    
    def arm_async(self) -> Future:
        return self._schedule(self._arm())
    
    def disarm(self) -> None:
        """Disarm the vehicle.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.disarm_async())
    
    def disarm_async(self) -> Future:
        return self._schedule(self._disarm())
    
    def kill(self) -> None:
        """Kill the vehicle (stops motors).
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.kill_async())
    
    def kill_async(self) -> Future:
        return self._schedule(self._kill())
    
    # Takeoff, land
    def takeoff(self, altitude_m: float) -> None:
        """Take off the vehicle.
        
        :param float altitude_m: Takeoff altitude (in meters).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.takeoff_async(altitude_m))
    
    def takeoff_async(self, altitude_m: float) -> Future:
        return self._schedule(self._takeoff(altitude_m))
    
    def land(self) -> None:
        """Land the vehicle.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.land_async())
    
    def land_async(self) -> Future:
        return self._schedule(self._land())
    
    # Simple flight
    def orbit(self, radius_m: float, velocity_m_s: float, center_position: PositionWGS84 | None = None) -> None:
        """Orbit around a point.
        
        :param float radius_m: Orbit radius (in meters).
        :param float velocity_m_s: Tangential velocity (in meters per second).
        :param PositionWGS84 | None center_position: Center position of the orbit (in degrees/meters, WGS84 frame). If None, the current position is used.
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        
        self._wait_for_command(self.orbit_async(radius_m, velocity_m_s, center_position))
    
    def orbit_async(self, radius_m: float, velocity_m_s: float, center_position: PositionWGS84 | None = None) -> Future:
        return self._schedule(self._orbit(radius_m, velocity_m_s, center_position))
    
    def goto_location(self, position: PositionWGS84, yaw_deg: float) -> None:
        """
        Fly to a specific location (simple action).
        
        :param PositionWGS84 position: Target position (in degrees/meters, WGS84 frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.goto_location_async(position, yaw_deg))
    
    def goto_location_async(self, position: PositionWGS84, yaw_deg: float) -> Future:
        return self._schedule(self._goto_location(position, yaw_deg))
    
    def hold(self) -> None:
        """
        Hold the current position.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.hold_async())
    
    def hold_async(self) -> Future:
        return self._schedule(self._hold())
    
    def return_to_launch(self) -> None:
        """
        Return to the launch position.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.return_to_launch_async())
    
    def return_to_launch_async(self) -> Future:
        return self._schedule(self._return_to_launch())
    
    # Offboard flight
    def enter_setpoint_control_mode(self) -> None:
        """
        Enter the setpoint control mode.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.enter_setpoint_control_mode_async())
    
    def enter_setpoint_control_mode_async(self) -> Future:
        return self._schedule(self._enter_offboard_mode())
    
    def exit_setpoint_control_mode(self) -> None:
        """
        Exit the setpoint control mode.
        
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.exit_setpoint_control_mode_async())
    
    def exit_setpoint_control_mode_async(self) -> Future:
        return self._schedule(self._exit_offboard_mode())
    
    def set_target_position_wgs(self, position: PositionWGS84, yaw_deg: float, altitude_type: AltitudeType) -> None:
        """
        Set the target position (offboard control mode).
        
        :param PositionWGS84 position: Target position (in degrees/meters, WGS84 frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :param AltitudeType altitude_type: Altitude type (REL_HOME, AMSL or AGL).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_target_position_wgs_async(position, yaw_deg, altitude_type))
    
    def set_target_position_wgs_async(self, position: PositionWGS84, yaw_deg: float, altitude_type: AltitudeType) -> Future:
        return self._schedule(self._set_target_position_wgs(position, yaw_deg, altitude_type))
    
    def set_target_position_ned(self, position: VectorNED, yaw_deg: float) -> None:
        """
        Set the target position (offboard control mode).
        
        :param VectorNED position: Target position (in meters, NED frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_target_position_ned_async(position, yaw_deg))
    
    def set_target_position_ned_async(self, position: VectorNED, yaw_deg: float) -> Future:
        return self._schedule(self._set_target_position_ned(position, yaw_deg))
    
    def set_target_velocity_ned(self, velocity: VectorNED, yaw_deg: float) -> None:
        """
        Set the target velocity (offboard control mode).
        
        :param VectorNED velocity: Target velocity (in meters per second, NED frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_target_velocity_ned_async(velocity, yaw_deg))
    
    def set_target_velocity_ned_async(self, velocity: VectorNED, yaw_deg: float) -> Future:
        return self._schedule(self._set_target_velocity_ned(velocity, yaw_deg))
    
    def set_target_position_velocity_ned(self, position: VectorNED, velocity: VectorNED, yaw_deg: float) -> None:
        """
        Set the target position and velocity (offboard control mode).
        
        :param VectorNED position: Target position (in meters, NED frame).
        :param VectorNED velocity: Target velocity (in meters per second, NED frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_target_position_velocity_ned_async(position, velocity, yaw_deg))
    
    def set_target_position_velocity_ned_async(self, position: VectorNED, velocity: VectorNED, yaw_deg: float) -> Future:
        return self._schedule(self._set_target_position_velocity_ned(position, velocity, yaw_deg))
    
    def set_target_position_velocity_acceleration_ned(self, position: VectorNED, velocity: VectorNED, acceleration: VectorNED, yaw_deg: float) -> None:
        """
        Set the target position, velocity and acceleration (offboard control mode).
        
        :param VectorNED position: Target position (in meters, NED frame).
        :param VectorNED velocity: Target velocity (in meters per second, NED frame).
        :param VectorNED acceleration: Target acceleration (in meters per second squared, NED frame).
        :param float yaw_deg: Yaw angle (in degrees, NED frame, 0 is North, positive is clockwise).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_target_position_velocity_acceleration_ned_async(position, velocity, acceleration, yaw_deg))
    
    def set_target_position_velocity_acceleration_ned_async(self, position: VectorNED, velocity: VectorNED, acceleration: VectorNED, yaw_deg: float) -> Future:
        return self._schedule(self._set_target_position_velocity_acceleration_ned(position, velocity, acceleration, yaw_deg))
    
    def set_target_setpoint_ned(self, setpoint: ControlSetpoint) -> None:
        """
        Set the target setpoint (offboard control mode).
        
        :param ControlSetpoint setpoint: Target setpoint (contains position, velocity, acceleration).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        :raises ValueError: If the setpoint is invalid (e.g., contains None values for required fields).
        """
        self._wait_for_command(self.set_target_setpoint_ned_async(setpoint))
    
    def set_target_setpoint_ned_async(self, setpoint: ControlSetpoint) -> Future:
        if setpoint.yaw_deg is None:
            raise ValueError("Yaw must always be set in the ControlSetpoint.")
        if setpoint.acceleration is not None:
            if setpoint.position is None or setpoint.velocity is None:
                raise ValueError("If acceleration is set, position and velocity must also be set.")
            else:
                # acc + vel + pos
                return self.set_target_position_velocity_acceleration_ned_async(setpoint.position, setpoint.velocity, setpoint.acceleration, setpoint.yaw_deg)
        else:
            if setpoint.velocity is not None:
                if setpoint.position is None:
                    # vel
                    return self.set_target_velocity_ned_async(setpoint.velocity, setpoint.yaw_deg)
                else:
                    # vel + pos
                    return self.set_target_position_velocity_ned_async(setpoint.position, setpoint.velocity, setpoint.yaw_deg)
            else:
                if setpoint.position is not None:
                    # pos
                    return self.set_target_position_ned_async(setpoint.position, setpoint.yaw_deg)
                else:
                    raise ValueError("No valid setpoint provided.")
    
    # Other
    def set_actuator_value(self, actuator_number: int, value: float) -> None:
        """
        Set the value of a specific actuator.
        
        :param int actuator_number: Index of the actuator (starts with 1).
        :param float value: Value to set the actuator to (0 to 1).
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_actuator_value_async(actuator_number, value))
    
    def set_actuator_value_async(self, actuator_number: int, value: float) -> Future:
        value = value * 2 - 1 # Conversion from 0-1 to (-1)-1
        return self._schedule(self._set_actuator_value(actuator_number, value))
    
    def set_flight_limits(self, flight_limits: FlightLimits) -> None:
        """
        Set flight limits (offboard control mode).
        
        :param FlightLimits flight_limits: Flight limits to set.
        :raises RuntimeError: If the FCInterface is not started yet, the command fails or sending times out.
        """
        self._wait_for_command(self.set_flight_limits_async(flight_limits))
    
    def set_flight_limits_async(self, flight_limits: FlightLimits) -> Future:
        return self._schedule(self._set_flight_limits(flight_limits))
    
    #############################################################
    # Public API (telemetry)                                    #
    #############################################################
    
    def get_raw_telemetry(self) -> VehicleState:
        """
        Get the current telemetry state.
        
        Some fields may be None if telemetry has not been received yet.
        
        :returns VehicleState: The current telemetry state.
        """
        
        with self._telemetry_lock:
            return copy.copy(self._telemetry)
    
    def is_armed(self) -> bool:
        """
        Get the current armed state.
        
        :returns bool: Current armed state. (False if not received yet)
        """
        return self._telemetry.armed if self._telemetry.armed is not None else False
    
    def is_in_air(self) -> bool:
        """
        Get the current in-air state.
        
        :returns bool: Current in-air state. (False if not received yet)
        """
        return self._telemetry.in_air if self._telemetry.in_air is not None else False
    
    def is_health_ok(self) -> bool:
        """
        Get the current health state.
        
        :returns bool: Current health state. (False if not received yet)
        """
        return self._telemetry.health_ok if self._telemetry.health_ok is not None else False
    
    def get_flight_mode(self) -> FlightMode:
        """
        Get the current flight mode.
        
        :returns FlightMode: Current flight mode. (FlightMode.UNKNOWN if not received yet)
        """
        return self._telemetry.flight_mode if self._telemetry.flight_mode is not None else FlightMode.UNKNOWN
    
    def is_in_setpoint_control_mode(self) -> bool:
        """
        Get the current setpoint control mode state.
        
        :returns bool: Current setpoint control mode state. (False if unknown)
        """
        return (self.get_flight_mode() == FlightMode.OFFBOARD)
    
    def get_battery_state(self) -> Battery:
        """
        Get the current battery state.
        
        :returns Battery: Current battery state.
        """
        return self._telemetry.battery_state if self._telemetry.battery_state is not None else Battery(
            id=0,
            temperature_degc=float('nan'),
            voltage_v=0.0,
            current_battery_a=float('nan'),
            capacity_consumed_ah=float('nan'),
            remaining_percent=0.0,
            time_remaining_s=0.0,
            battery_function=0
        )
    
    def get_position(self) -> VectorNED:
        """
        Get the current position.
        
        :returns VectorNED: Current position (in meters, NED frame). (0, 0, 0 if not received yet)
        """
        return self._telemetry.position if self._telemetry.position is not None else VectorNED(0.0, 0.0, 0.0)
    
    def get_velocity(self) -> VectorNED:
        """
        Get the current velocity.
        
        :returns VectorNED: Current velocity (in meters per second, NED frame). (0, 0, 0 if not received yet)
        """
        return self._telemetry.velocity if self._telemetry.velocity is not None else VectorNED(0.0, 0.0, 0.0)
    
    def get_quaternion(self) -> Quaternion:
        """
        Get the current orientation as a quaternion.
        
        :returns Quaternion: Current orientation as a quaternion. (0, 0, 0, 1 if not received yet)
        """
        return self._telemetry.quaternion if self._telemetry.quaternion is not None else Quaternion(0.0, 0.0, 0.0, 1.0)
    
    def get_gps_info_blocking(self, timeout_s: float) -> GpsInfo:
        """
        Get the current GPS info.
        
        :returns GpsInfo: Current GPS info.
        :raises TimeoutError: If the GPS info has not been received within the timeout period.
        """
        return self._block_until_value(lambda: self._telemetry.gps_info, timeout_s)
    
    def get_raw_gps_position_blocking(self, timeout_s: float) -> PositionWGS84:
        """
        Get the current GPS position.
        
        :returns PositionWGS84: Current GPS position (in degrees/meters, WGS84 frame).
        :raises TimeoutError: If the GPS position has not been received within the timeout period.
        """
        raw_gps = self._block_until_value(lambda: self._telemetry.raw_gps, timeout_s)
        return PositionWGS84(
            latitude=raw_gps.latitude_deg,
            longitude=raw_gps.longitude_deg,
            altitude=raw_gps.absolute_altitude_m
        )
    
    def get_gps_global_origin(self) -> PositionWGS84:
        """
        Get the GPS global origin (home position).
        
        :returns PositionWGS84: GPS global origin (in degrees/meters, WGS84 frame).
        :raises RuntimeError: If the GPS global origin is not available.
        """
        return self._wait_for_command(self.get_gps_global_origin_async())
    
    def get_gps_global_origin_async(self) -> Future:
        return self._schedule(self._get_gps_global_origin())
    
    #############################################################
    # Commands                                                  #
    #############################################################
    
    # Arm, disarm, kill
    async def _arm(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.arm()
    
    async def _disarm(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.disarm()
    
    async def _kill(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.kill()
    
    # Takeoff, land
    async def _takeoff(self, takeoff_altitude_m: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.set_takeoff_altitude(takeoff_altitude_m)
        await self._system.action.takeoff()
    
    async def _land(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.land()
    
    # Simple flight
    async def _orbit(self, radius_m: float, velocity_m_s: float, center_position: PositionWGS84 | None) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        if center_position is None:
            await self._system.action.do_orbit(radius_m, velocity_m_s, ORBIT_YAW_BEHAVIOR, None, None, None)
        else:
            await self._system.action.do_orbit(radius_m, velocity_m_s, ORBIT_YAW_BEHAVIOR, center_position.latitude, center_position.longitude, center_position.altitude)
    
    async def _goto_location(self, position: PositionWGS84, yaw_deg: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.goto_location(position.latitude, position.longitude, position.altitude, yaw_deg)
    
    async def _hold(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.hold()
    
    async def _return_to_launch(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.return_to_launch()
    
    # Offboard flight
    async def _enter_offboard_mode(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await self._system.offboard.start()
    
    async def _exit_offboard_mode(self) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.stop()
    
    async def _set_target_position_wgs(self, position: PositionWGS84, yaw_deg: float, altitude_type: AltitudeType) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_position_global(PositionGlobalYaw(position.latitude, position.longitude, position.altitude, yaw_deg, altitude_type))
    
    async def _set_target_position_ned(self, position: VectorNED, yaw_deg: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_position_ned(PositionNedYaw(position.north, position.east, position.down, yaw_deg))
    
    async def _set_target_velocity_ned(self, velocity: VectorNED, yaw_deg: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_velocity_ned(VelocityNedYaw(velocity.north, velocity.east, velocity.down, yaw_deg))
    
    async def _set_target_position_velocity_ned(self, position: VectorNED, velocity: VectorNED, yaw_deg: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_position_velocity_ned(
                PositionNedYaw(position.north, position.east, position.down, yaw_deg),
                VelocityNedYaw(velocity.north, velocity.east, velocity.down, yaw_deg)
            )
    
    async def _set_target_position_velocity_acceleration_ned(self, position: VectorNED, velocity: VectorNED, acceleration: VectorNED, yaw_deg: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.offboard.set_position_velocity_acceleration_ned(
                PositionNedYaw(position.north, position.east, position.down, yaw_deg),
                VelocityNedYaw(velocity.north, velocity.east, velocity.down, yaw_deg),
                AccelerationNed(acceleration.north, acceleration.east, acceleration.down)
            )
    
    # Other
    async def _set_actuator_value(self, actuator_number: int, value: float) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        await self._system.action.set_actuator(actuator_number, value)
    
    async def _set_flight_limits(self, flight_limits: FlightLimits) -> None:
        if self._system is None:
            raise RuntimeError("FCInterface is not started.")
        match self._fc_type:
            case FCType.PX4:
                await asyncio.gather(
                    self._system.param.set_param_float("MPC_XY_VEL_MAX", flight_limits.velocity_horizontal_m_s),
                    self._system.param.set_param_float("MPC_Z_VEL_MAX_UP", flight_limits.velocity_vertical_m_s),
                    self._system.param.set_param_float("MPC_Z_VEL_MAX_DN", flight_limits.velocity_vertical_m_s),
                    self._system.param.set_param_float("MPC_ACC_HOR", flight_limits.acceleration_horizontal_m_s2),
                    self._system.param.set_param_float("MPC_ACC_UP_MAX", flight_limits.acceleration_vertical_m_s2),
                    self._system.param.set_param_float("MPC_ACC_DOWN_MAX", flight_limits.acceleration_vertical_m_s2),
                    self._system.param.set_param_float("MPC_YAWRAUTO_MAX", flight_limits.rate_yaw_deg_s),
                    self._system.param.set_param_float("MPC_YAWRAUTO_ACC", flight_limits.acceleration_yaw_deg_s2),
                )
            case FCType.ARDUPILOT:
                await asyncio.gather(
                    self._system.param.set_param_float("WPNAV_SPEED", flight_limits.velocity_horizontal_m_s * 100),       # (m/s -> cm/s)
                    self._system.param.set_param_float("WPNAV_SPEED_UP", flight_limits.velocity_vertical_m_s * 100),      # (m/s -> cm/s)
                    self._system.param.set_param_float("WPNAV_SPEED_DN", flight_limits.velocity_vertical_m_s * 100),      # (m/s -> cm/s)
                    self._system.param.set_param_float("WPNAV_ACCEL", flight_limits.acceleration_horizontal_m_s2 * 100),  # (m/s^2 -> cm/s^2)
                    self._system.param.set_param_float("WPNAV_ACCEL_Z", flight_limits.acceleration_vertical_m_s2 * 100),  # (m/s^2 -> cm/s^2)
                    self._system.param.set_param_float("ATC_RATE_Y_MAX", flight_limits.rate_yaw_deg_s),
                    self._system.param.set_param_float("ATC_ACCEL_Y_MAX", flight_limits.acceleration_yaw_deg_s2 * 100),   # (deg/s^2 -> cdeg/s^2)
                )
    
    #############################################################
    # Telemetry Subscriptions                                   #
    #############################################################
    
    # I removed ALL locks for testing purposes, add before flight
    
    async def _telemetry_subscribe_armed(self) -> None:
        # await self._system.telemetry.set_rate_armed(TELEMETRY_RATE_FLIGHT_STATUS_HZ)
        assert self._system is not None
        async for armed in self._system.telemetry.armed():
            self._telemetry.armed = bool(armed)
    
    async def _telemetry_subscribe_in_air(self) -> None:
        assert self._system is not None
        await self._system.telemetry.set_rate_in_air(TELEMETRY_RATE_FLIGHT_STATUS_HZ)
        async for in_air in self._system.telemetry.in_air():
            self._telemetry.in_air = bool(in_air)
    
    async def _telemetry_subscribe_health(self) -> None:
        assert self._system is not None
        await self._system.telemetry.set_rate_health(TELEMETRY_RATE_FLIGHT_STATUS_HZ)
        async for health in self._system.telemetry.health():
            self._telemetry.health_ok = bool(health.is_global_position_ok and health.is_home_position_ok)
    
    async def _telemetry_subscribe_flight_mode(self) -> None:
        # await self._system.telemetry.set_rate_flight_mode(TELEMETRY_RATE_FLIGHT_STATUS_HZ)
        assert self._system is not None
        async for flight_mode in self._system.telemetry.flight_mode():
            self._telemetry.flight_mode = flight_mode
    
    async def _telemetry_subscribe_battery(self) -> None:
        assert self._system is not None
        await self._system.telemetry.set_rate_battery(TELEMETRY_RATE_BATTERY_HZ)
        async for battery in self._system.telemetry.battery():
            self._telemetry.battery_state = battery
    
    async def _telemetry_subscribe_pos_vel(self) -> None:
        assert self._system is not None
        await self._system.telemetry.set_rate_position_velocity_ned(TELEMETRY_RATE_POSE_HZ)
        async for pv in self._system.telemetry.position_velocity_ned():
            self._telemetry.position = VectorNED(
                north=pv.position.north_m,
                east=pv.position.east_m,
                down=pv.position.down_m,
            )
            self._telemetry.velocity = VectorNED(
                north=pv.velocity.north_m_s,
                east=pv.velocity.east_m_s,
                down=pv.velocity.down_m_s,
            )
    
    async def _telemetry_subscribe_quaternion(self) -> None:
        assert self._system is not None
        await self._system.telemetry.set_rate_attitude_quaternion(TELEMETRY_RATE_POSE_HZ)
        async for quat in self._system.telemetry.attitude_quaternion():
            self._telemetry.quaternion = Quaternion(
                w=quat.w,
                x=quat.x,
                y=quat.y,
                z=quat.z,
            )
    
    async def _telemetry_subscribe_gps_info(self) -> None:
        assert self._system is not None
        async for gps_info in self._system.telemetry.gps_info():
            self._telemetry.gps_info = gps_info
    
    async def _telemetry_subscribe_raw_gps(self) -> None:
        assert self._system is not None
        async for raw_gps in self._system.telemetry.raw_gps():
            self._telemetry.raw_gps = raw_gps
    
    async def _get_gps_global_origin(self) -> PositionWGS84:
        assert self._system is not None
        pos = await self._system.telemetry.get_gps_global_origin()
        return PositionWGS84(
            latitude=pos.latitude_deg,
            longitude=pos.longitude_deg,
            altitude=pos.altitude_m
        )
    
    #############################################################
    # Helper Methods                                            #
    #############################################################
    
    def _schedule(self, coro: typing.Coroutine) -> Future:
        if self._loop is None:
            raise RuntimeError("FCInterface is not started.")
        return asyncio.run_coroutine_threadsafe(coro, self._loop)
    
    def _wait_for_command(self, fut: Future) -> typing.Any:
        try:
            return fut.result(timeout=COMMAND_TIMEOUT_S)
        except TimeoutError as e:
            self._logger.debug("Command timed out.")
            raise RuntimeError("Command timed out.") from e
        except Exception as e:
            self._logger.debug("Command failed.")
            raise RuntimeError("Command failed.") from e
    
    def _block_until_value(self, read_fn, timeout_s: float) -> typing.Any:
        start = time.monotonic()
        
        while True:
            with self._telemetry_lock:
                value = read_fn()
            if value is not None:
                return value
            if (time.monotonic() - start) > timeout_s:
                raise TimeoutError("Timeout waiting for telemetry value.")
            
            time.sleep(1.0 / TELEMETRY_WAIT_RATE_HZ)