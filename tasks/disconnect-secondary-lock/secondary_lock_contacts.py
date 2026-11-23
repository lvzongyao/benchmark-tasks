"""Confirm opposing finger contact using the native contact stream."""

import json

import numpy as np


class Contacts:
    def __init__(self, names, path):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        arms = {"LeftArm": [], "RightArm": []}
        parts = {}
        for prim in get_current_stage().Traverse():
            path_string = prim.GetPath().pathString
            elements = path_string.split("/")
            if "env_0" not in elements or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            for arm, bodies in arms.items():
                if arm in elements:
                    bodies.append(path_string)
            if elements[-2] in names and elements[-1] == "body":
                parts[elements[-2]] = path_string
            if elements[-2] == "plug" and elements[-1] in ("safety", "latch"):
                parts[elements[-1]] = path_string
        self.left, self.right = arms.values()
        physics = PhysxManager.get_physics_sim_view()
        self.arm_view = physics.create_rigid_contact_view(
            self.left, filter_patterns=[self.right] * len(self.left),
            max_contact_data_count=128 * len(self.left),
        )
        fingers = [body for arm in arms.values() for body in arm
                   if body.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        if len(fingers) != 4:
            raise RuntimeError("Expected two fingers on each YAM arm")
        filters = [parts[name] for name in names]
        self.part_view = physics.create_rigid_contact_view(
            fingers, filter_patterns=[filters] * 4, max_contact_data_count=512
        )
        self.names = names
        self.path = path
        self.count = 0
        self.substep = 0
        self.loads = np.zeros((4, len(names)))

    def poll(self, dt):
        self.substep += 1
        if self.path:
            _, _, _, separation, counts, starts = self.arm_view.get_contact_data(dt=dt)
            separation, counts, starts = (np.asarray(value.numpy()) for value in (separation, counts, starts))
            for i, j in zip(*np.nonzero(counts)):
                start = starts[i, j]
                gap = float(separation[start:start + counts[i, j]].min())
                if gap <= 0:
                    self.count += 1
                    with open(self.path, "a") as stream:
                        stream.write(json.dumps(dict(substep=self.substep, separation_m=gap,
                                                     left=self.left[i], right=self.right[j])) + "\n")
        data = self.part_view.get_contact_force_matrix(dt=dt)
        self.loads = np.linalg.norm(np.asarray(data.numpy()), axis=-1)

    def grasp(self, arm, part):
        first = 0 if arm == "left" else 2
        return bool(np.all(self.loads[first:first + 2, self.names.index(part)] >= .3))
