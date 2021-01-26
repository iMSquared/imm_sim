#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass, field
from typing import List, Tuple
from nptyping import NDArray
import numpy as np
import pybullet as pb


class EnvironmentBase(ABC):
    @abstractmethod
    def reset(self, sim_id: int):
        raise NotImplementedError('')


class PlaneEnvironment(EnvironmentBase):
    def __init__(self):
        self.plane_shape = None
        self.plane_index = -1

    def reset(self, sim_id: int):
        # (bkim) Should we be resetting the objects to their original poses rather than creating them?
        #   Perhaps I misunderstood the implication of "reset"
        # Add ground plane.
        plane_shape = pb.createCollisionShape(
            pb.GEOM_PLANE, physicsClientId=sim_id)
        plane_index = pb.createMultiBody(
            0, plane_shape, physicsClientId=sim_id)
        pb.resetBasePositionAndOrientation(
            plane_index, [0, 0, -0.05], [0, 0, 0, 1], physicsClientId=sim_id)

        # Save data ...
        self.plane_shape = plane_shape
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


def _get_gibson_env_args():
    args = []

    room_pos = np.asarray([-9.550345, - 0.020595, - 1.7319335])
    room_dim = np.asarray([6.61371,  5.82243,  2.289713])

    room_pos0 = room_pos.copy()
    room_pos[0] = 0
    room_pos[1] = 0
    room_pos[2] = 0.5 * room_dim[2]

    # create walls.
    dims = []
    poss = []
    rots = []
    for axis in [0, 1, 2]:
        for sign in [-1, +1]:
            # NOTE(ycho): Omitting ceiling for now.
            if axis == 2:
                continue
            delta = np.zeros(3)
            delta[axis] = 0.5 * sign * room_dim[axis]
            pos = room_pos + delta

            dim = np.copy(room_dim)
            dim[axis] = 0.001  # thin wall
            print(dim)

            dims.append(dim)
            poss.append(pos)
            rots.append(0)

    objs = [[[-11.53166838,  -0.91719885,  -2.37988421],
             [1.0340247,  1.21300617, 0.96312665]],
            [[-10.82257805,   1.57771568,  -2.46199754],
             [2.25832757, 1.12761466, 0.80739635]],
            [[-9.32991398, -2.20684131, -1.72572009],
             [1.4041856, 0.9604687, 0.9326944]],
            [[-12.65817192,  -0.26651379,  -2.20800889],
             [0.34358125, 0.48218821, 0.15264422]],
            [[-8.78562739, -0.2068401,  -1.30027797],
             [0.15620964, 2.63670967, 0.54305325]],
            [[-11.92512889,   2.2138277,   -1.35238892],
             [1.24478475, 1.1150199,  1.51239998]],
            [[-10.21153113,   2.72963058,  -1.73007356],
             [4.18224676, 0.27717014, 2.22364437]],
            [[-8.63737393,  2.75835732, -1.15031033],
             [0.93847601, 0.13781747, 0.59718779]],
            [[-7.54175084,  2.71732009, -0.91827751],
             [0.83824107, 0.08062678, 0.1382604]],
            [[-11.96370916,   2.75282812,  -0.86385341],
             [0.16262788, 0.08879988, 0.17830753]],
            [[-12.54824522,   0.74395948,  -2.47258913],
             [0.1400429,  1.05539944, 0.52965069]],
            [[-8.78578509,  2.72200841, -1.61014202],
             [0.63875082, 0.03720206, 0.28677278]],
            [[-11.95910916,   2.37578096,  -2.37052202],
             [0.37135846, 0.51393595, 0.66976016]],
            [[-11.9776881,    2.30970187,  -2.54975087],
             [0.26682292, 0.37003432, 0.24778953]],
            [[-12.52049507,   1.54107777,  -2.66817758],
             [0.11066429, 0.47054442, 0.30421549]],
            [[-12.50416596,  -0.81947222,  -2.30552539],
             [0.03356049, 0.95836586, 0.19838245]],
            [[-12.05270194,   2.4273823,   -2.25735051],
             [0.12758444, 0.40575824, 0.33061129]],
            [[-8.79327127, -0.33027898, -1.67406873],
             [0.14310903, 0.96706501, 0.24565965]],
            [[-8.64830359, -0.43835292, -2.15187905],
             [0.44565189, 1.17945511, 0.72321382]],
            [[-8.70329352, -0.9827516,  -2.66746781],
             [0.31482338, 0.41465757, 0.30099835]],
            [[-12.49730948,  -2.04347152,  -2.51401615],
             [0.04124159, 1.50147032, 0.6435191]],
            [[-12.55067868,  -0.58088025,  -2.55647935],
             [0.13763857, 1.45284475, 0.4015983]],
            [[-12.54484448,  -1.18887977,  -2.7764629],
                [0.13447007, 0.21665006, 0.11553238]],
            [[-12.52464233,  -0.91816745,  -2.7601176],
                [0.10086077, 0.34437094, 0.14388747]],
            [[-8.65847228, -0.98015309, -2.02569947],
             [0.40166525, 0.50922885, 0.61463143]],
            [[-8.78288617,  0.37708207, -1.67194706],
             [0.15002563, 0.38098157, 0.22621442]]]

    for p, d in objs:
        dims.append(d)
        poss.append(p + room_pos - room_pos0)
        rots.append(0)
    return dims, poss, rots


class SampleGibsonEnvironment(Box2DEnvironment):
    def __init__(self):
        args = _get_gibson_env_args()
        super().__init__(Box2DEnvironmentSettings(*args))
