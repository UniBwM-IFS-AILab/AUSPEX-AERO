# This module provides classes and functions for complex vehicle guidance.

from ._types import VectorNED, CheckpointTolerances, ControlSetpoint


#############################################################
# Guidance Classes                                          #
#############################################################

class BasicFlyTo:
    def __init__(self, current_position: VectorNED, current_velocity: VectorNED, target_position: VectorNED, guidance_settings: CheckpointTolerances, look_ahead_distance_m: float = 10.0) -> None:
        self._target_pos = target_position
        self._current_pos = current_position
        self._current_vel = current_velocity
        self._guidance_settings = guidance_settings
        self._look_ahead_distance_m = look_ahead_distance_m
        diff = (self._target_pos - self._current_pos)
        self._initial_distance = diff.norm()
        self._yaw = diff.angle_from_north()
    
    def update(self, current_position: VectorNED, current_velocity: VectorNED) -> ControlSetpoint:
        self._current_pos = current_position
        self._current_vel = current_velocity
        distance = (self._target_pos - self._current_pos).norm()
        if distance < self._look_ahead_distance_m:
            return ControlSetpoint(
                yaw_deg=self._yaw,
                position=self._target_pos
            )
        t = self._look_ahead_distance_m / distance
        pos = self._current_pos + t * (self._target_pos - self._current_pos)
        return ControlSetpoint(
            yaw_deg=self._yaw,
            position=pos
        )
    
    def progress_percent(self) -> float:
        distance = (self._target_pos - self._current_pos).norm()
        if self._initial_distance == 0:
            return 100.0
        return min(100.0, (self._initial_distance - distance) / self._initial_distance * 100.0)
    
    def is_finished(self) -> bool:
        diff = self._target_pos - self._current_pos
        return (
            diff.horizontal_norm() < self._guidance_settings.position_horizontal_m and
            abs(diff.down) < self._guidance_settings.position_vertical_m and
            self._current_vel.norm() < self._guidance_settings.velocity_total_m_s
        )