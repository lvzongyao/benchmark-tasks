"""Expose the exact starting orientation to the task's stock waypoint player."""

from yam_arena import demo, ik, station
from yam_arena.constants import READY


class ReadyRotation(float):
    def __new__(cls, rotation):
        yaw = super().__new__(cls, 0.)
        yaw.rotation = rotation
        return yaw


def ready_pose(arm):
    stock_rotation = demo.grip_rotation
    if not getattr(stock_rotation, "_sleeve_ready_pose", False):
        def grip_rotation(yaw=0.):
            if isinstance(yaw, ReadyRotation):
                return yaw.rotation
            return stock_rotation(yaw)

        grip_rotation._sleeve_ready_pose = True
        demo.grip_rotation = grip_rotation
    position, rotation = ik.tcp_pose(READY, station.BASES[arm])
    return tuple(position), ReadyRotation(rotation)
