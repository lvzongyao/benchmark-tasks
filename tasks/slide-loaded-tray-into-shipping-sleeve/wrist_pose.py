"""Play waypoints with the tilted wrist orientations used for bimanual packing."""

from yam_arena import demo


class GripRotation(float):
    def __new__(cls, rotation):
        yaw = super().__new__(cls, 0.)
        yaw.rotation = rotation
        return yaw


def as_waypoint_rotation(rotation):
    stock_rotation = demo.grip_rotation
    if not getattr(stock_rotation, "_c2_grip_pose", False):
        def grip_rotation(yaw=0.):
            if isinstance(yaw, GripRotation):
                return yaw.rotation
            return stock_rotation(yaw)

        grip_rotation._c2_grip_pose = True
        demo.grip_rotation = grip_rotation
    return GripRotation(rotation)
