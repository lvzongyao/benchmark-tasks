"""Optional PhysX diagnostics for contact between the left and right YAM arms."""

import json

import numpy as np


class ArmContacts:
    def __init__(self, path):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        bodies = {"LeftArm": [], "RightArm": []}
        for prim in get_current_stage().Traverse():
            parts = prim.GetPath().pathString.split("/")
            if "env_0" not in parts or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            for arm in bodies:
                if arm in parts:
                    bodies[arm].append(prim.GetPath().pathString)
        left, right = bodies.values()
        if not left or not right:
            raise RuntimeError("Could not locate both YAM arms for contact diagnostics")

        physics = PhysxManager.get_physics_sim_view()
        self.view = physics.create_rigid_contact_view(
            left, filter_patterns=[right] * len(left), max_contact_data_count=128 * len(left)
        )
        self.left = left
        self.right = right
        self.path = path
        self.substeps = self.contacts = 0
        self.record(substep=0, left_bodies=left, right_bodies=right)

    def record(self, **event):
        with open(self.path, "a") as log:
            log.write(json.dumps(event) + "\n")

    def poll(self, dt):
        self.substeps += 1
        _, _, _, separations, counts, starts = self.view.get_contact_data(dt=dt)
        separations, counts, starts = (np.asarray(v.numpy()) for v in (separations, counts, starts))
        for left, right in zip(*np.nonzero(counts)):
            start, count = starts[left, right], counts[left, right]
            gap = float(separations[start:start + count].min())
            if gap <= 0:
                self.contacts += 1
                self.record(substep=self.substeps, time_s=self.substeps * dt, separation_m=gap,
                            left=self.left[left].rsplit("/", 1)[-1],
                            right=self.right[right].rsplit("/", 1)[-1])
