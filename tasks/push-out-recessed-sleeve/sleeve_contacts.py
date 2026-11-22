"""Optional PhysX diagnostics for cross-arm contact and the plate pinch."""

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
        plate = None
        for prim in get_current_stage().Traverse():
            parts = prim.GetPath().pathString.split("/")
            if "env_0" not in parts or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            for arm in bodies:
                if arm in parts:
                    bodies[arm].append(prim.GetPath().pathString)
            if parts[-2:] == ["plate", "body"]:
                plate = prim.GetPath().pathString
        left, right = bodies.values()
        if not left or not right:
            raise RuntimeError("Could not locate both YAM arms for contact diagnostics")

        physics = PhysxManager.get_physics_sim_view()
        self.view = physics.create_rigid_contact_view(
            left, filter_patterns=[right] * len(left), max_contact_data_count=128 * len(left)
        )
        fingers = [path for path in left if path.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        if len(fingers) != 2 or plate is None:
            raise RuntimeError("Could not locate both left fingers and the plate")
        self.pinch = physics.create_rigid_contact_view(
            fingers, filter_patterns=[[plate]] * len(fingers), max_contact_data_count=128
        )
        self.left = left
        self.right = right
        self.path = path
        self.substeps = self.contacts = 0
        self.grasp_samples = []
        self.record(substep=0, left_bodies=left, right_bodies=right, grasp_fingers=fingers, plate=plate)

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

        force, points, _, _, counts, starts = self.pinch.get_contact_data(dt=dt)
        force, points, counts, starts = (np.asarray(v.numpy()) for v in (force, points, counts, starts))
        pads = []
        for start, count in zip(starts.flat, counts.flat):
            loads = force[start:start + count].reshape(-1)
            total = float(loads.sum())
            position = (np.average(points[start:start + count], axis=0, weights=loads).tolist()
                        if total > 0 else None)
            pads.append((total, position))
        self.grasp_samples.append(pads)

    def read_grasp(self, plate):
        samples, self.grasp_samples = self.grasp_samples, []
        rotation = plate.rotation()
        return {
            "substeps": len(samples),
            "two_loaded": sum(all(force >= .5 for force, _ in pads) for pads in samples),
            "minimum_force_n": [min((pads[i][0] for pads in samples), default=0.) for i in range(2)],
            "contact_local": [[((np.array(position) - plate.pos) @ rotation).tolist()
                               if position is not None else None for _, position in pads]
                              for pads in samples],
        }
