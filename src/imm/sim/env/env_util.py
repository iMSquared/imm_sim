#!/usr/bin/env python3
import pybullet as pb


def place_robot(sim_id: int, env_id: int, robot_id: int):
    env_aabb = pb.getAABB(env_id, -1, physicsClientId=robot_id)
