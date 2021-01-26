from robots.robot_base import RobotBase
from dataclasses import dataclass
from typing import List, Dict, Tuple

import pybullet as pb


@dataclass
class UR5Settings:
    joint_angles: List[Tuple[float, float, float, float, float, float]]


class UR5(RobotBase):
    def __init__(self, settings: UR5Settings):
        ee_name = 'suction'
        self.settings_ = settings
        self.robot_id_ = pb.loadURDF(f'robots/robot_descriptions/ur5/ur5-{ee_name}.urdf')
        num_joints = pb.getNumJoints(self.robot_id_)
        joints = [pb.getJointInfo(self.robot_id_, i) for i in range(num_joints)]
        self.joints = [j[0] for j in joints if j[2] == pb.JOINT_REVOLUTE]
        self.ee_tip_link_index = 12

    def reset(self, sim_id):
        for j_idx, j in enumerate(self.joints):
            pb.resetJointState(self.robot_id_, j, self.settings_.joint_angles[j_idx])

    @property
    def robot_id(self):
        return self.robot_id_

    def solve_ik(self, pose):
        pass

    def get_joint_angles(self):
        joint_angles = [pb.getJointState(self.robot_id_, j)[0] for j in self.joints]
        return joint_angles

