"""Native contact points for the two gauges and the actual rotor and pad."""

import numpy as np


class Contacts:
    def __init__(self, gauges):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        paths = [str(p.GetPath()) for p in get_current_stage().Traverse()
                 if "env_0" in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        left = [p for p in paths if "/LeftArm/" in p]
        right = [p for p in paths if "/RightArm/" in p]

        def body(name):
            return next(p for p in paths if p.endswith(f"/{name}/body"))

        sim = PhysxManager.get_physics_sim_view()
        rotor, pad = body("rotor"),body("pad")
        self.probes = sim.create_rigid_contact_view(
            [body(name) for name in gauges], filter_patterns=[[rotor,pad]]*len(gauges),
            max_contact_data_count=160)
        self.support = sim.create_rigid_contact_view([pad],filter_patterns=[[rotor]],max_contact_data_count=64)
        self.cross = sim.create_rigid_contact_view(left,filter_patterns=[right]*len(left),
                                                    max_contact_data_count=64*len(left))
        self.pairs = [[[],[]] for _ in gauges]
        self.pad_support = 0.
        self.arm_collisions = 0

    def poll(self, dt):
        force, points, normals, _, count, start = (
            np.asarray(a.numpy()) for a in self.probes.get_contact_data(dt=dt))
        for gauge, reference in np.ndindex(count.shape):
            sl = slice(start[gauge,reference],start[gauge,reference]+count[gauge,reference])
            self.pairs[gauge][reference] = list(zip(force[sl].reshape(-1),points[sl],normals[sl]))
        self.pad_support = abs(float(self.support.get_contact_force_matrix(dt=dt).numpy()[0,0,2]))
        _, _, _, gaps, count, start = (np.asarray(a.numpy()) for a in self.cross.get_contact_data(dt=dt))
        self.arm_collisions += sum(np.any(gaps[i:i+n] <= 0.) for i,n in zip(start.flat,count.flat) if n)
