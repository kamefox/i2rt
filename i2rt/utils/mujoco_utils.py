import os

import mujoco
import numpy as np


class MuJoCoKDL:
    """MuJoCo 逆动力学；需安装 mujoco 包。仅在 MotorChainRobot 传入 xml_path 时加载。"""

    def __init__(self, path: str):
        self.model = mujoco.MjModel.from_xml_path(os.path.expanduser(path))
        self.data = mujoco.MjData(self.model)
        self.set_gravity(np.array([0, 0, -9.81]))

        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0
        self.model.jnt_limited[:] = 0

    @property
    def joint_limits(self) -> np.ndarray:
        m = self.model
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        slide = int(mujoco.mjtJoint.mjJNT_SLIDE)
        lims: list[list[float]] = []
        for j in range(m.njnt):
            jt = int(m.jnt_type[j])
            if jt in (hinge, slide):
                lims.append([float(m.jnt_range[j, 0]), float(m.jnt_range[j, 1])])
        if not lims:
            return np.zeros((0, 2))
        return np.array(lims)

    def compute_inverse_dynamics(self, q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray) -> np.ndarray:
        assert len(q) == len(qdot) == len(qdotdot)
        length = len(q)
        self.data.qpos[:length] = q
        self.data.qvel[:length] = qdot
        self.data.qacc[:length] = qdotdot
        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[:length]

    def set_gravity(self, gravity: np.ndarray) -> None:
        assert gravity.shape == (3,)
        self.model.opt.gravity = gravity
