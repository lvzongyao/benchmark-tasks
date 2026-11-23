"""PhysX contact channels for load transfer and robot grasp and arm clearance."""

import numpy as np


class Contacts:
    def __init__(self):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        bodies = [str(p.GetPath()) for p in get_current_stage().Traverse()
                  if "env_0" in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        left = [p for p in bodies if "/LeftArm/" in p]
        right = [p for p in bodies if "/RightArm/" in p]
        left_fingers = [p for p in left if p.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        right_fingers = [p for p in right if p.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        temporary, foot = [next(p for p in bodies if p.endswith(f"/{name}/body")) for name in ("temporary", "foot")]
        panel = next(p for p in bodies if p.endswith("/panel_unit/panel"))
        self.physics = PhysxManager.get_physics_sim_view()
        self.supports = self.make([panel], [temporary, foot])
        self.pinch = self.make(left_fingers, [panel])
        self.block_grip = self.make(right_fingers, [temporary, foot])
        self.cross = self.make(left, right)
        self.cross_count = 0
        self.sample_count = 0
        self.snapshot = {"support": [0.,0.], "panel_grip": [[0.],[0.]], "block_grip": [[0.,0.],[0.,0.]]}

    def make(self, senders, receivers):
        return self.physics.create_rigid_contact_view(
            senders, filter_patterns=[receivers] * len(senders), max_contact_data_count=64 * len(senders))

    @staticmethod
    def loads(view, dt):
        forces, _, _, _, counts, starts = view.get_contact_data(dt=dt)
        forces, counts, starts = (np.asarray(a.numpy()) for a in (forces, counts, starts))
        return [[float(np.abs(forces[first:first+count]).sum()) for first, count in zip(start, row)]
                for start, row in zip(starts, counts)]

    def poll(self, dt):
        self.snapshot = {"support": self.loads(self.supports, dt)[0], "panel_grip": self.loads(self.pinch, dt),
                         "block_grip": self.loads(self.block_grip,dt)}
        _, _, _, gaps, counts, starts = (np.asarray(a.numpy()) for a in self.cross.get_contact_data(dt=dt))
        self.cross_count += sum(np.any(gaps[i:i+n] <= 0.) for i, n in zip(starts.flat, counts.flat) if n)
        self.sample_count += 1
