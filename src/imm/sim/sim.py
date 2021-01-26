#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import Dict

import pybullet as pb

from imm.sim.env import SampleGibsonEnvironment, PlaneEnvironment
from imm.sim.robots.sphere_robot import SphereTreeRobot, SphereTreeRobotSettings
from imm.sim.robots.ur5 import UR5, UR5Settings
from imm.sim.sensor import SensorBase, JointStateSensor, BasePoseSensor, ImageSensor
from imm.sim.sensor import ImageSettings
import numpy as np


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
        self.env_ = PlaneEnvironment()

        # robot
        # FIXME(ycho): Should load from settings instead of hardcoding.
        # self.robot_ = SphereRobot()
        # (bkim) we probably want to load an URDF, and you must connect to a physics engine to do that
        """
        self.robot_ = SphereTreeRobot(SphereTreeRobotSettings(
            link_positions=[[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
            parent_indices=[0, 0, 0],
            radius=0.125
        ))
        """

        # sensors
        # FIXME(ycho): Should load from settings instead of hardcoding.
        # TODO(ycho): Consider multi-robot scenarios, which would instead be
        # a mapping i.e. { "robot_a" : {"joints" : ...}, "robot_b" : {"" : ...}}
        # (bkim) it might be better to have the sensors inside the robot class, because each
        #   robot have different DoFs and require DoF indices to get the joint values
        self.sensors_ = {'joints': JointStateSensor(),
                         'base_pose': BasePoseSensor(),
                         'camera': ImageSensor(ImageSettings(sensor_link='joint3'))}

        # task

        # state
        self.sim_id_ = -1

    def sense(self):
        # (bkim) continuing on the note above, this function can be integrated into the robot class
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

        self.robot_ = UR5(UR5Settings(joint_angles=[np.pi / 2.0, -np.pi / 2.0, 0, 0, 0, 0]))
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        self.sim_id_ = sim_id
        kwargs = dict(physicsClientId=sim_id)

        # Configure simulation.
        pb.setGravity(0, 0, settings.gravity, **kwargs)
        pb.setTimeStep(settings.timestep, **kwargs)
        pb.setRealTimeSimulation(False, **kwargs)

    def reset(self):
        # why do we need to reset everything? Perhaps the name could be more informative?
        #pb.resetSimulation(self.sim_id)
        self.env_.reset(self.sim_id)
        self.robot_.reset(self.sim_id)
        for _, sensor in self.sensors_.items():
            sensor.reset(self.sim_id, self.robot_.robot_id)

    def step(self):
        pb.stepSimulation(physicsClientId=self.sim_id)
        return self.sense()

    @property
    def sim_id(self):
        return self.sim_id_


def main():
    settings = SimulatorSettings()
    sim = Simulator(settings)
    sim.start()

    sim.reset()
    pb.startStateLogging(
                pb.STATE_LOGGING_VIDEO_MP4, '/tmp/imm-sim.mp4')

    import pdb;pdb.set_trace()
    for i in range(256):
        data = sim.step()
        # NOTE(ycho): For debugging sensors...
        if False:
            print(data)
        """
        pb.resetDebugVisualizerCamera(cameraDistance=7.0, cameraYaw=i*180/128,
                cameraPitch=-50,
                cameraTargetPosition=(0,0,0))

        # NOTE(ycho): For debugging image outputs.
        if True:
            import cv2
            color_image, depth_image = data['camera']
            # RGBA -> BGRA
            cv2.imshow('color', color_image[..., [2, 1, 0, 3]])
            cv2.waitKey(1)
        """

        # time.sleep(0.1)



if __name__ == '__main__':
    main()
