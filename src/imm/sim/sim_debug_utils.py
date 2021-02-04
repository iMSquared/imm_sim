#!/usr/bin/env python3

from typing import List
import pybullet as pb
import numpy as np
import math

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


