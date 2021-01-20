#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass
from typing import List, Dict, Tuple
import pybullet as pb


class RobotBase(ABC):
    @abstractmethod
    def reset(self, sim_id: int):
        raise NotImplementedError('')

    @abstractproperty
    def robot_id(self):
        raise NotImplementedError('')


class SphereRobot(RobotBase):
    def __init__(self):
        self.sim_id_ = -1
        self.robot_id_ = -1

    def reset(self, sim_id: int):
        self.sim_id_ = sim_id

        # Create geometry
        col_idx = pb.createCollisionShape(pb.GEOM_SPHERE, radius=1.0,
                                          physicsClientId=self.sim_id)
        vis_idx = pb.createVisualShape(pb.GEOM_SPHERE, radius=1.0,
                                       physicsClientId=self.sim_id)

        model_args = dict(
            baseCollisionShapeIndex=col_idx,
            baseVisualShapeIndex=vis_idx)

        self.robot_id_ = pb.createMultiBody(
            **model_args,
            physicsClientId=self.sim_id)

    @property
    def sim_id(self):
        return self.sim_id_

    @property
    def robot_id(self):
        return self.robot_id_


@dataclass
class SphereTreeRobotSettings:
    radius: float
    link_positions: List[Tuple[float, float, float]]
    parent_indices: List[int]


class SphereTreeRobot(RobotBase):
    def __init__(self, settings: SphereTreeRobotSettings):
        self.sim_id_ = -1
        self.robot_id_ = -1
        self.settings_ = settings

    def reset(self, sim_id: int):
        settings = self.settings_

        self.sim_id_ = sim_id

        # Create geometry
        col_idx = pb.createCollisionShape(pb.GEOM_SPHERE, radius=settings.radius,
                                          physicsClientId=self.sim_id)
        vis_idx = pb.createVisualShape(pb.GEOM_SPHERE, radius=settings.radius,
                                       physicsClientId=self.sim_id)

        num_links = len(self.settings_.link_positions)
        self.robot_id_ = pb.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=col_idx,
            baseVisualShapeIndex=vis_idx,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            baseInertialFramePosition=[0, 0, 0],
            baseInertialFrameOrientation=[0, 0, 0, 1],

            linkMasses=num_links * [1],
            linkCollisionShapeIndices=num_links * [col_idx],
            linkVisualShapeIndices=num_links * [vis_idx],
            linkPositions=settings.link_positions,
            linkOrientations=num_links * [[0, 0, 0, 1]],
            linkInertialFramePositions=num_links * [[0, 0, 0]],
            linkInertialFrameOrientations=num_links * [[0, 0, 0, 1]],
            linkParentIndices=settings.parent_indices,
            linkJointTypes=num_links * [pb.JOINT_REVOLUTE],
            linkJointAxis=num_links * [[0, 0, 1]],
            physicsClientId=self.sim_id)

        pb.resetBasePositionAndOrientation(self.robot_id,
                                           [0, 0, settings.radius], [
                                               0, 0, 0, 1],
                                           physicsClientId=self.sim_id)

    @property
    def sim_id(self):
        return self.sim_id_

    @property
    def robot_id(self):
        return self.robot_id_
