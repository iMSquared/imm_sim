#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass, field
from typing import Dict, List
import time
import importlib
import numpy as np
import pkg_resources
import logging
import pybullet as pb

# from imm.sim.env import PlaneEnvironment, Box2DEnvironment, Box2DEnvironmentSettings, SampleGibsonEnvironment
from imm.sim.env import ThreeDFrontEnvironment, ThreeDFrontEnvironmentSettings
from imm.sim.robot import SphereRobot, SphereTreeRobot, SphereTreeRobotSettings, UrdfRobotSettings, UrdfRobot
from imm.sim.sensor import SensorBase, JointStateSensor, BasePoseSensor, ImageSensor
from imm.sim.sensor import ImageSettings
from imm.sim.sim_debug_utils import debug_draw_inertia_box, debug_draw_frame_axes, debug_draw_bounding_box


@dataclass
class SimulatorSettings:
    render: bool = True
    sensor: dict = field(default_factory=dict)
    task: str = 'none'
    gravity: float = -9.81
    timestep: float = 1.0/240


class Simulator(object):
    sensors_: Dict[str, SensorBase]

    def __init__(self, settings: SimulatorSettings):
        self.settings_ = settings

        # env
        # FIXME(ycho): Should load from settings instead of hardcoding.
        # self.env_ = PlaneEnvironment()
        # self.env_ = SampleGibsonEnvironment()
        self.env_ = ThreeDFrontEnvironment(
            ThreeDFrontEnvironmentSettings(
                model_dir='/media/ssd/datasets/3DFRONT/3D-FUTURE-model/3D-FUTURE-model/',
                scene_dir='/media/ssd/datasets/3DFRONT/3D-FRONT/',
                scene_file='/media/ssd/datasets/3DFRONT/3D-FRONT/4b41ef33-c496-455c-b8f2-aa32d5152878.json'
                # scene_file='/media/ssd/datasets/3DFRONT/3D-FRONT/b7efb4f9-33a9-4b2b-acf4-574b8d2a6f18.json'
            ))

        # robot
        # FIXME(ycho): Should load from settings instead of hardcoding.
        # self.robot_ = SphereRobot()
        # self.robot_ = SphereTreeRobot(SphereTreeRobotSettings(
        #    link_positions=[[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
        #    parent_indices=[0, 0, 0],
        #    radius=0.125
        # ))

        fetch_urdf_file = pkg_resources.resource_filename(
            'imm.sim', 'data/fetch_description/robots/fetch.urdf')
        self.robot_ = UrdfRobot(UrdfRobotSettings(
            filename=fetch_urdf_file, position=[-1.0, 0, 0.01]))

        # sensors
        # FIXME(ycho): Should load from settings instead of hardcoding.
        # TODO(ycho): Consider multi-robot scenarios, which would instead be
        # a mapping i.e. { "robot_a" : {"joints" : ...}, "robot_b" : {"" : ...}}
        self.sensors_ = {'joints': JointStateSensor(),
                         'base_pose': BasePoseSensor(),
                         'camera': ImageSensor(ImageSettings(sensor_link='head_camera_joint'))
                         }

        # task

        # state
        self.sim_id_ = -1

    def sense(self):
        out = {}
        for sensor_id, sensor in self.sensors_.items():
            out[sensor_id] = sensor.sense()
        return out

    def start(self):
        # Bring param to this scope
        settings = self.settings_

        # Connect to simulation.
        if settings.render:
            sim_id = pb.connect(pb.GUI)
        else:
            sim_id = pb.connect(pb.DIRECT)
        self.sim_id_ = sim_id

    def reset(self):
        settings = self.settings_

        # Hard-reset simulation.
        pb.resetSimulation(physicsClientId=self.sim_id)

        # Reconfigure simulation.
        pb.setGravity(0, 0, settings.gravity, physicsClientId=self.sim_id)
        pb.setTimeStep(settings.timestep, physicsClientId=self.sim_id)
        pb.setRealTimeSimulation(False, physicsClientId=self.sim_id)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)

        # Reset environment and agents.
        self.env_.reset(self.sim_id)
        self.robot_.reset(self.sim_id)
        for _, sensor in self.sensors_.items():
            sensor.reset(self.sim_id, self.robot_.robot_id)

        # Delegate robot positioning to the env in order to find a collision-free initial position.
        # TODO(ycho): Figure out the optimal architecture for this.
        # TODO(ycho): Unsupported API from EnvBase, only ThreeDFrontEnvironment.
        self.env_.place(self.robot_.robot_id)

        # Add debugging handlers.
        if True:
            debug_draw_frame_axes(self.sim_id, self.robot_.robot_id, scale=0.2)
            num_joints = pb.getNumJoints(self.robot_.robot_id)

            vd = pb.getVisualShapeData(self.robot_.robot_id)
            for v in vd:
                pb.changeVisualShape(v[0], v[1], rgbaColor=[0, 1, 0, 0.5])
            for i in [-1] + list(range(num_joints)):
                debug_draw_inertia_box(
                    self.robot_.robot_id, i, color=(0, 0, 1))

            debug_draw_bounding_box(self.sim_id, self.robot_.robot_id)
            # debug_draw_bounding_box(self.sim_id, self.env_.env_ids_[0])

    def step(self):
        out = pb.stepSimulation(physicsClientId=self.sim_id)
        return self.sense()

    @property
    def sim_id(self):
        return self.sim_id_


def main():
    logging.basicConfig(level=logging.INFO)
    settings = SimulatorSettings()
    sim = Simulator(settings)
    sim.start()

    sim.reset()

    # NOTE(ycho): Only for exporting animation.
    # pb.startStateLogging(
    # pb.STATE_LOGGING_VIDEO_MP4, '/tmp/imm-sim.mp4')
    # for i in range(256):

    t = 0
    while True:
        data = sim.step()
        t += sim.settings_.timestep
        # print('current time = {}'.format(t))
        # NOTE(ycho): For debugging sensors...
        if False:
            print(data)

        # NOTE(ycho): Only for exporting animation ...
        # TODO(ycho): Configurable camera trajectory?
        if False:
            pb.resetDebugVisualizerCamera(cameraDistance=7.0, cameraYaw=i*180/128,
                                          cameraPitch=-50,
                                          cameraTargetPosition=(0, 0, 0))

        # NOTE(ycho): For debugging image outputs.
        if False and 'camera' in data:
            import cv2
            color_image, depth_image = data['camera']
            # RGBA -> BGRA
            cv2.imshow('color', color_image[..., [2, 1, 0, 3]])
            cv2.waitKey(1)

        # time.sleep(0.1)


if __name__ == '__main__':
    main()
