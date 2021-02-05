#!/usr/bin/env python3

from typing import List
import pybullet as pb
import numpy as np
import math
import itertools


def debug_get_full_aabb(sim_id: int, robot_id: int):
    """
    Get full AABB of a robot including all of its constituent links.
    """
    num_joints = pb.getNumJoints(robot_id, physicsClientId=sim_id)
    aabb = np.asarray(pb.getAABB(
        robot_id, -1, physicsClientId=sim_id), dtype=np.float32)
    for i in range(num_joints):
        link_aabb = pb.getAABB(robot_id, i, physicsClientId=sim_id)
        aabb[0] = np.minimum(aabb[0], link_aabb[0], out=aabb[0])
        aabb[1] = np.maximum(aabb[1], link_aabb[1], out=aabb[1])
    return aabb


def debug_draw_bounding_box(sim_id: int, robot_id: int):
    # Get full AABB in world coordinates.
    aabb = debug_get_full_aabb(sim_id, robot_id)

    # World coord -> Base coord
    bp, bq = pb.getBasePositionAndOrientation(robot_id, physicsClientId=sim_id)
    R = np.reshape(pb.getMatrixFromQuaternion(bq), (3, 3))
    aabb = (aabb - R.T @ bp) @ R

    # Draw box (TODO(ycho): Optimize!)
    for i0 in itertools.product([0, 1], repeat=3):
        for i1 in itertools.product([0, 1], repeat=3):
            if np.equal(i0, i1).sum() != 2:
                continue
            c = np.cross(i0, i1)
            # u = 0.5 * np.add(p0, p1)
            # if c.dot(u) < 0:
            # continue
            # p0 = np.asarray(i0)
            # p1 = np.asarray(i1, dtype=np.int32)
            p0 = aabb[i0, [0, 1, 2]]
            p1 = aabb[i1, [0, 1, 2]]
            pb.addUserDebugLine(p0, p1, [0, 0, 1],
                                parentObjectUniqueId=robot_id,
                                physicsClientId=sim_id)


def debug_draw_frame_axes(sim_id: int, robot_id: int, scale: float = 1.0, joint_indices: List[int] = None):
    """
    Draw XYZ frame axes over links (color: RGB)
    If `joint_indices` is not specified, frames will be drawn for all joints.
    """
    # If joint indices not specified, draw over all existing links.
    if joint_indices is None:
        joint_indices = range(pb.getNumJoints(
            robot_id, physicsClientId=sim_id))

    for ji in joint_indices:
        for ax in np.eye(3):
            pb.addUserDebugLine([0, 0, 0], scale * ax, ax,
                                parentObjectUniqueId=robot_id,
                                parentLinkIndex=ji, physicsClientId=sim_id)


def debug_draw_inertia_box(parentUid, parentLinkIndex, color):
    """
    taken from pybullet/examples/quadruped.py
    """
    p = pb
    dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
    mass = dyn[0]
    frictionCoeff = dyn[1]
    inertia = dyn[2]
    if (mass > 0):
        Ixx = inertia[0]
        Iyy = inertia[1]
        Izz = inertia[2]
        boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
        boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
        boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

        halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
        pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
               [-halfExtents[0], halfExtents[1], halfExtents[2]],
               [halfExtents[0], -halfExtents[1], halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], halfExtents[2]],
               [halfExtents[0], halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], halfExtents[1], -halfExtents[2]],
               [halfExtents[0], -halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]

        p.addUserDebugLine(pts[0],
                           pts[1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[0],
                           pts[4],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[5],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[6],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[7],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[4 + 0],
                           pts[4 + 1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 1],
                           pts[4 + 3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 3],
                           pts[4 + 2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 2],
                           pts[4 + 0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
