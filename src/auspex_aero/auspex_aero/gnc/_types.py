# A module defining data structures for internal use in the GNC system.

from dataclasses import dataclass
from enum import Enum
import math
import numpy as np
from numpy.typing import NDArray

class AltitudeType(Enum):
    REL_HOME = 0
    AMSL = 1
    AGL = 2

#############################################################
# Position / Geometry related data structures               #
#############################################################

@dataclass(frozen=True)
class PositionWGS84:
    latitude: float
    longitude: float
    altitude: float
    altitude_type: AltitudeType = AltitudeType.AMSL

@dataclass(frozen=True, slots=True)
class VectorNED:
    _v: NDArray[np.float64]  # shape (3,)
    
    def __init__(self, north: float, east: float, down: float):
        object.__setattr__(
            self,
            "_v",
            np.array([north, east, down], dtype=np.float64),
        )
    
    @property
    def north(self) -> float:
        return float(self._v[0])

    @property
    def east(self) -> float:
        return float(self._v[1])

    @property
    def down(self) -> float:
        return float(self._v[2])

    def norm(self) -> float:
        return float(np.linalg.norm(self._v))

    def horizontal_norm(self) -> float:
        return float(np.linalg.norm(self._v[:2]))

    def angle_from_north(self) -> float:
        return math.degrees(math.atan2(self.east, self.north))
    
    # Mathematical operations
    def __add__(self, other: "VectorNED") -> "VectorNED":
        return VectorNED.from_array(self._v + other._v)

    def __sub__(self, other: "VectorNED") -> "VectorNED":
        return VectorNED.from_array(self._v - other._v)
    
    def __neg__(self) -> "VectorNED":
        return VectorNED.from_array(-self._v)
    
    def __mul__(self, scalar: float) -> "VectorNED":
        return VectorNED.from_array(self._v * float(scalar))

    def __rmul__(self, scalar: float) -> "VectorNED":
        return self.__mul__(scalar)
    
    @staticmethod
    def from_array(arr: np.ndarray) -> "VectorNED":
        a = np.asarray(arr, dtype=np.float64).reshape(3,)
        return VectorNED(float(a[0]), float(a[1]), float(a[2]))

@dataclass(frozen=True)
class Quaternion:
    w: float
    x: float
    y: float
    z: float

#############################################################
# Other data structures                                     #
#############################################################

@dataclass(frozen=True)
class FlightLimits:
    velocity_horizontal_m_s: float
    velocity_vertical_m_s: float
    acceleration_horizontal_m_s2: float
    acceleration_vertical_m_s2: float
    rate_yaw_deg_s: float
    acceleration_yaw_deg_s2: float

#############################################################
# Guidance data structures                                  #
#############################################################

@dataclass(frozen=True)
class ControlSetpoint:
    yaw_deg: float
    position: VectorNED | None = None
    velocity: VectorNED | None = None
    acceleration: VectorNED | None = None

@dataclass(frozen=True)
class CheckpointTolerances:
    position_horizontal_m: float
    position_vertical_m: float
    velocity_total_m_s: float
    yaw_deg: float