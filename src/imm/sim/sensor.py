#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass
from typing import List
import logging
import numpy as np
import pybullet as pb


def get_index_from_name(robot_id: int, sim_id: int,
                        names: List[str]):
    """
    Get the joint index of a joint from its name.
    each returned entry is -1 if not found.
    """
    num_joints = pb.getNumJoints(robot_id, physicsClientId=sim_id)
    indices = [-1 for _ in names]
    for i in range(num_joints):
        joint_info = pb.getJointInfo(
            robot_id, i, physicsClientId=sim_id)
        joint_name = joint_info[1].decode('utf-8')
        if joint_name in names:
            indices[names.index(joint_name)] = i
    return indices


class SensorBase(ABC):
    @abstractmethod
    def sense(self, sim_id: int):
        raise NotImplementedError('')


class BasePoseSensor(SensorBase):
    """
    Pybullet sensor for base pose.
    """

    def __init__(self):
        self.sim_id_ = -1
        self.robot_id_ = -1

    def reset(self, sim_id: int, robot_id: int):
        # Save IDs...
        self.sim_id_ = sim_id
        self.robot_id_ = robot_id

    def sense(self):
        txn, rxn = pb.getBasePositionAndOrientation(self.robot_id,
                                                    physicsClientId=self.sim_id)
        return (txn, rxn)

    @property
    def sim_id(self):
        return self.sim_id_

    @property
    def robot_id(self):
        return self.robot_id_


class JointStateSensor(SensorBase):
    """
    Pybullet sensor for joint states.
    """

    def __init__(self):
        # handles
        self.sim_id_ = -1
        self.robot_id_ = -1

        # params
        self.num_joints_ = 0

    def reset(self, sim_id: int, robot_id: int):
        # Save IDs...
        self.sim_id_ = sim_id
        self.robot_id_ = robot_id

        # Get relevant parameters.
        self.num_joints_ = pb.getNumJoints(
            self.robot_id, physicsClientId=sim_id)

    def sense(self):
        joint_indices = list(range(self.num_joints_))
        joint_states = pb.getJointStates(
            self.robot_id, joint_indices, physicsClientId=self.sim_id)
        return joint_states

    @property
    def sim_id(self):
        return self.sim_id_

    @property
    def robot_id(self):
        return self.robot_id_


@dataclass
class ImageSettings:
    width: int = 512
    height: int = 512
    vfov: float = np.deg2rad(90)
    hfov: float = np.deg2rad(90)
    near: float = 0.001
    far: float = 100.0
    sensor_link: str = ''


class ImageSensor(SensorBase):
    def __init__(self, settings: ImageSettings):
        self.sim_id_ = -1
        self.robot_id_ = -1
        self.link_id_ = -1
        self.settings_ = settings

        # Derived parameters
        self.projection_matrix = pb.computeProjectionMatrixFOV(
            fov=np.rad2deg(settings.hfov),
            aspect=settings.vfov / settings.hfov,
            nearVal=settings.near,
            farVal=settings.far
        )

    def reset(self, sim_id: int, robot_id: int):
        # Save IDs...
        self.sim_id_ = sim_id
        self.robot_id_ = robot_id
        num_joints = pb.getNumJoints(self.robot_id,
                                     physicsClientId=self.sim_id)

        # Search link id by link name.
        link_id = get_index_from_name(self.robot_id, self.sim_id, [
                                      self.settings_.sensor_link])[0]

        # If link not found, abort.
        if link_id < 0:
            logging.warn('sensor link : {} not found!'.format(
                self.settings_.sensor_link))
            return

        self.link_id_ = link_id

    def sense(self):
        settings = self.settings_

        # Early exit if in error state.
        if self.link_id < 0:
            return (None, None)

        # Camera position is defined in relation to a
        # (pybullet link frame) * offset.
        # Retrieve current pose of attached frame.
        link_state = pb.getLinkState(self.robot_id, self.link_id, 0, 0,
                                     physicsClientId=self.sim_id)

        # Compose view matrix.
        link_position = link_state[0]
        link_rotation = pb.getMatrixFromQuaternion(link_state[1])
        link_pose = np.eye(4)
        link_pose[:3, :3] = np.reshape(link_rotation, (3, 3))
        link_pose[:3, 3] = link_position

        # TODO(ycho): Consider adding an optional local offset here.
        # link_pose = link_pose @ settings.rel_offset

        # Convert link frame to optical frame.
        optical_rotation = np.asarray([
            [0, 0, -1, 0],
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        camera_pose = link_pose @ optical_rotation

        # Invert camera pose -> camera_from_world transform.
        view_matrix = np.eye(4)
        view_matrix[:3, 3] = -camera_pose[:3, :3].T @ camera_pose[:3, 3]
        view_matrix[:3, :3] = camera_pose[:3, :3].T

        # Convert to transposed (column-major) OpenGL convention.
        view_matrix = view_matrix.T

        out = pb.getCameraImage(
            width=settings.width,
            height=settings.height,
            projectionMatrix=self.projection_matrix,
            viewMatrix=view_matrix.ravel(),
            renderer=pb.ER_BULLET_HARDWARE_OPENGL,
            flags=pb.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self.sim_id)
        _, _, color_image, depth_image, _ = out
        return (color_image, depth_image)

    @property
    def sim_id(self):
        return self.sim_id_

    @property
    def robot_id(self):
        return self.robot_id_

    @property
    def link_id(self):
        return self.link_id_
