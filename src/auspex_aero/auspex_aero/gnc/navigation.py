# This module provides the navigation for the vehicle.
# It is just a forward for FCInterface plus coordinate conversion.

import math
import logging
import time

import pymap3d

from ._types import VectorNED, PositionWGS84, Quaternion
from .fc_interface import FCNavigationPort

#############################################################
# Constants                                                 #
#############################################################

# Timeouts
ORIGIN_INIT_TIMEOUT_S = 120.0
ORIGIN_INIT_POLL_RATE_HZ = 1.0


#############################################################
# Navigation Class                                          #
#############################################################

class Navigation:
    def __init__(self, fc: FCNavigationPort):
        self._fc = fc
        self._logger = logging.getLogger("navigation")
        self.origin = None
    
    def wait_for_gnss_origin_location(self) -> PositionWGS84:
        self._logger.info("Waiting for GNSS origin location...")
        deadline = time.monotonic() + ORIGIN_INIT_TIMEOUT_S
        while True:
            try:
                pos = self._fc.get_raw_gps_position_blocking(timeout_s=deadline - time.monotonic())
                gps_info = self._fc.get_gps_info_blocking(timeout_s=deadline - time.monotonic())
                if gps_info.fix_type.value >= 3:  # 3D fix or better
                    self._logger.info(f"Received GNSS origin location: {pos}.")
                    return pos
            except TimeoutError:
                self._logger.warning("Timeout waiting for GNSS origin location.")
                raise RuntimeError("Failed to get GNSS origin location within timeout.")
            self._logger.debug("GNSS origin location not valid yet.")
            time.sleep(1.0 / ORIGIN_INIT_POLL_RATE_HZ)
    
    def set_best_effort_origin(self) -> None:
        self._logger.info("Setting best effort origin...")
        try:
            pos = self._fc.get_gps_global_origin()
            self._logger.info("Setting origin to GPS global origin from FC...")
            self.set_origin(pos)
            return
        except Exception as e:
            self._logger.warning(f"Failed to get GPS global origin from FC: {e}")
        self._logger.info("Setting origin to raw GPS position...")
        pos = self._fc.get_raw_gps_position_blocking(timeout_s=0.1)
        self.set_origin(pos)
    
    def set_origin(self, origin: PositionWGS84) -> None:
        self._logger.info(f"Set origin location to {origin}.")
        self.origin = origin
    
    def get_origin(self) -> PositionWGS84 | None:
        if self.origin is None:
            return None
        return self.origin
    
    # get state
    def get_position_ned(self) -> VectorNED:
        return self._fc.get_position()
    
    def get_position_wgs84(self) -> PositionWGS84:
        pos_ned = self._fc.get_position()
        return self.convert_ned_to_wgs84(pos_ned)
    
    def get_velocity_ned(self) -> VectorNED:
        return self._fc.get_velocity()
    
    def get_quaternion(self) -> Quaternion:
        return self._fc.get_quaternion()
    
    def get_euler_angles(self, degrees: bool = True) -> tuple[float, float, float]:
        q = self._fc.get_quaternion()
        yaw, pitch, roll = self._quaternion_to_euler(q)
        if degrees:
            yaw_deg = math.degrees(yaw)
            pitch_deg = math.degrees(pitch)
            roll_deg = math.degrees(roll)
            return yaw_deg, pitch_deg, roll_deg
        return yaw, pitch, roll
    
    # utils
    def convert_wgs84_to_ned(self, position: PositionWGS84) -> VectorNED:
        """
        Convert a position from WGS84 coordinates to NED coordinates.
        
        :param PositionWGS84 position: position (in degrees/meters, WGS84 frame).
        :return VectorNED: position (in meters, NED frame).
        :raises RuntimeError: If the reference position has not been set.
        """
        if self.origin is None:
            raise RuntimeError("Origin position not set. Call set_origin() before converting coordinates.")
        north, east, down = pymap3d.geodetic2ned(
            lat=position.latitude,
            lon=position.longitude,
            h=position.altitude,
            lat0=self.origin.latitude,
            lon0=self.origin.longitude,
            h0=self.origin.altitude,
            ell=None,
            deg=True
        )
        return VectorNED(north=north, east=east, down=down)
    
    def convert_ned_to_wgs84(self, position: VectorNED) -> PositionWGS84:
        """
        Convert a position from NED coordinates to WGS84 coordinates.
        
        :param VectorNED position: position (in meters, NED frame).
        :return PositionWGS84: position (in degrees/meters, WGS84 frame).
        :raises RuntimeError: If the origin position has not been set.
        """
        if self.origin is None:
            raise RuntimeError("Origin position not set. Call set_origin() before converting coordinates.")
        lat, lon, alt = pymap3d.ned2geodetic(
            n=position.north,
            e=position.east,
            d=position.down,
            lat0=self.origin.latitude,
            lon0=self.origin.longitude,
            h0=self.origin.altitude,
            ell=None,
            deg=True
        )
        return PositionWGS84(latitude=lat, longitude=lon, altitude=alt)
    
    def _quaternion_to_euler(self, q: Quaternion) -> tuple[float, float, float]:
        w, x, y, z = q.w, q.x, q.y, q.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90° if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll