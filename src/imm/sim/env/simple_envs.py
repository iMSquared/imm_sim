#!/usr/bin/env python3

from imm.sim.env.env_base import EnvironmentBase

from dataclasses import dataclass, field
from typing import List, Tuple
from nptyping import NDArray
import numpy as np
import pybullet as pb


class PlaneEnvironment(EnvironmentBase):
    def __init__(self):
        # self.plane_shape = None
        self.vis_idx = -1
        self.col_idx = -1
        self.plane_index = -1

    def reset(self, sim_id: int):
        # Add ground plane.
        vis_idx = pb.createVisualShape(
            pb.GEOM_PLANE, physicsClientId=sim_id)
        col_idx = pb.createCollisionShape(
            pb.GEOM_PLANE, physicsClientId=sim_id)
        plane_index = pb.createMultiBody(
            0, baseCollisionShapeIndex=col_idx, physicsClientId=sim_id)
        # plane_index = pb.createMultiBody(
        #    0, baseCollisionShapeIndex=col_idx, baseVisualShapeIndex=vis_idx, physicsClientId=sim_id)
        pb.resetBasePositionAndOrientation(
            plane_index, [0, 0, 0.0], [0, 0, 0, 1], physicsClientId=sim_id)

        # Save data ...
        # self.plane_shape = plane_shape
        self.vis_idx = vis_idx
        self.col_idx = col_idx
        self.plane_index = plane_index


@dataclass
class Box2DEnvironmentSettings:
    box_dim: List[NDArray[3, float]] = field(default_factory=list)
    box_pos: List[NDArray[3, float]] = field(default_factory=list)
    box_rot: List[NDArray[1, float]] = field(default_factory=list)


class Box2DEnvironment(EnvironmentBase):
    """
    Environment of boxes.
    """

    def __init__(self, settings: Box2DEnvironmentSettings):
        self.settings_ = settings
        pass

    def reset(self, sim_id: int):
        s = self.settings_

        # Add boxes.
        for d, p, r in zip(s.box_dim, s.box_pos, s.box_rot):
            d = np.asarray(d)
            p = np.asarray(p)
            r = np.asarray(r)
            box_shape = pb.createCollisionShape(pb.GEOM_BOX,
                                                halfExtents=0.5 * d)
            pb.createMultiBody(0, box_shape,
                               basePosition=p,
                               baseOrientation=pb.getQuaternionFromAxisAngle((0, 0, 1), r))

        # Add ground plane.
        plane_shape = pb.createCollisionShape(
            pb.GEOM_PLANE, physicsClientId=sim_id)
        plane_index = pb.createMultiBody(
            0, plane_shape, physicsClientId=sim_id)
        pb.resetBasePositionAndOrientation(
            plane_index, [0, 0, 0.0], [0, 0, 0, 1], physicsClientId=sim_id)
