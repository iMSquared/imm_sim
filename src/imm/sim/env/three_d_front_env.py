#!/usr/bin/env python3

import os
import random
import json
import numpy as np
import time
import logging
from typing import List

from pathlib import Path
import pybullet as pb
from dataclasses import dataclass

from imm.sim.env.env_base import EnvironmentBase
from imm.sim.sim_debug_utils import debug_get_full_aabb


def _ceildiv(a, b):
    """ from https://stackoverflow.com/a/17511341 """
    return -(-a // b)


def _split_multibody_kwargs(kwargs: dict) -> List[dict]:
    """
    Split kwargs for createMultiBody, so that
    the number of links do not exceed the hardcoded #128 limit.

    NOTE(ycho): This is a hack - consider alternative solutions.
    """

    num_links = len(kwargs['linkMasses'])
    num_bodies = _ceildiv(num_links, 128)

    # NOTE(ycho): Still return as a list even
    # in a trivial case.
    if num_bodies <= 1:
        return [kwargs]
    m = num_links // num_bodies

    out = [None for _ in range(num_bodies)]
    for i in range(num_bodies):
        # link slice range ...
        i0 = i * m
        i1 = min((i+1)*m, num_links)

        # Start from a copy of `kwargs`.
        out[i] = dict(kwargs)

        # Reset properties that should be unique to
        # a single base.
        # NOTE(ycho): This does not pedantically clear ALL properties -
        # only the ones that are known to be set at creation.
        out[i]['baseCollisionShapeIndex'] = -1
        out[i]['baseVisualShapeIndex'] = -1

        # Set a slice of link properties.
        for k, v in kwargs.items():
            if k.startswith('link'):
                out[i][k] = v[i0:i1]

    # Move base-unique properties into the first element in the split args.
    out[0]['baseCollisionShapeIndex'] = kwargs['baseCollisionShapeIndex']
    out[0]['baseVisualShapeIndex'] = kwargs['baseVisualShapeIndex']

    return out


def _load_tdf_scene(scene_file: str, model_dir: str, sim_id: int,
                    use_convex_collision: bool):
    """
    Load a 3DFRONT scene.
    """
    model_dir = Path(model_dir)

    data = None
    with open(scene_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if data is None:
        return None

    # Build mapping from uid <-> jid
    model_uid = []
    model_jid = []
    model_map = dict()
    for f in data['furniture']:
        if 'valid' in f and f['valid']:
            model_map[f['uid']] = f['jid']

    # Build mapping from uid <-> mesh = { vertices, face_indices }
    mesh_map = dict()
    for m in data['mesh']:
        mesh_map[m['uid']] = (
            np.reshape(m['xyz'], (-1, 3)).astype(np.float32),
            np.reshape(m['faces'], (-1, 3))
        )

    # NOTE(ycho): Special handling for `floor` for facilitating free space sampling.
    # TODO(ycho): Remove this workaround after PR#3238 in the remote is merged.
    floor_set = set()
    for m in data['mesh']:
        if m['type'].strip() == 'Floor':
            floor_set.add(m['uid'])

    # Iterate through and add shapes . . .
    scene = data['scene']
    room = scene['room']
    shape_map = dict()
    kwargs = {
        'baseMass': 0,  # fixed,
        'baseCollisionShapeIndex': -1,
        'baseVisualShapeIndex': -1,
        'basePosition': [0, 0, 0],
        # NOTE(ycho): +Y up convention -> +Z up convention
        'baseOrientation': pb.getQuaternionFromEuler([np.pi/2, 0, 0]),
        'baseInertialFramePosition': [0, 0, 0],
        'baseInertialFrameOrientation': [0, 0, 0, 1],

        'linkMasses': [],
        'linkCollisionShapeIndices': [],
        'linkVisualShapeIndices': [],
        'linkPositions': [],
        'linkOrientations': [],
        'linkInertialFramePositions': [],
        'linkInertialFrameOrientations': [],
        'linkParentIndices': [],
        'linkJointTypes': [],
        'linkJointAxis': [],

        'physicsClientId': sim_id
    }

    # NOTE(ycho): Special handling for `floor` for facilitating free space sampling.
    # TODO(ycho): Remove this workaround after PR#3238 in the remote is merged.
    floor_vertices = np.empty((0, 3), dtype=np.float32)
    floor_face_indices = np.empty((0), dtype=np.int32)

    for r in room:
        room_id = r['instanceid']
        children = r['children']
        for c in children:
            ref = c['ref']

            def _lookup_cache(ref):
                """ Lookup collision shape from cache. """
                if ref not in shape_map:
                    return -1, -1
                return shape_map[ref]

            def _lookup_model(ref):
                """ Lookup collision shape from 3D-FUTURE models. """
                if ref not in model_map:
                    return -1, -1
                mid = model_map[ref]
                model_file = (model_dir / mid / 'raw_model.obj')

                # NOTE(ycho): Temporary workaround for degenerate model?
                # if '39057a21-0a68-3494-8522-2e473dd6a38f' in str(model_file):
                #    return -1, -1

                if not model_file.exists():
                    logging.warn('No such model file : {}'.format(model_file))
                    return -1, -1
                # TODO(ycho): remove this flag ... only for visualization
                # or leave it in? idk...
                col_id = pb.createCollisionShape(
                    pb.GEOM_MESH, fileName=str(model_file),
                    meshScale=c['scale'],
                    # flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
                    physicsClientId=sim_id
                )
                vis_id = pb.createVisualShape(
                    pb.GEOM_MESH, fileName=str(model_file),
                    meshScale=c['scale'],
                    flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
                    physicsClientId=sim_id
                )
                return (col_id, vis_id)

            def _lookup_mesh(ref):
                """ Lookup collision shape from inline mesh  """
                if ref not in mesh_map:
                    return -1, -1
                (vertices, face_indices) = mesh_map[ref]
                vertices = vertices.astype(np.float64).reshape(-1, 3)

                # FIXME(ycho): This is a hack to enable visualization
                # against pybullet issues with backface culling - i.e.
                # rendering the walls from the exterior through a window.
                face_indices = np.c_[
                    face_indices,
                    face_indices[..., ::-1]
                ]
                face_indices = face_indices.astype(np.int32).reshape(-1)
                col_id = pb.createCollisionShape(pb.GEOM_MESH,
                                                 vertices=vertices,
                                                 indices=face_indices,
                                                 flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
                                                 physicsClientId=sim_id)
                # NOTE(ycho): VisualShape is redundant for now.
                # vis_id = pb.createVisualShape(pb.GEOM_MESH,
                #                               vertices=vertices,
                #                               indices=face_indices,
                #                               flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
                #                               physicsClientId=sim_id)
                vis_id = -1
                return (col_id, vis_id)

            # NOTE(ycho): Special handling for `floor` for facilitating free space sampling.
            # TODO(ycho): Remove this workaround after PR#3238 in the remote is merged.
            if (ref in floor_set) and (ref in mesh_map):
                vertices, face_indices = mesh_map[ref]
                face_indices = face_indices.astype(np.int32).reshape(-1)
                R = np.reshape(pb.getMatrixFromQuaternion(c['rot']), (3, 3))
                floor_face_indices = np.r_[
                    floor_face_indices, face_indices + len(floor_vertices)]
                floor_vertices = np.r_[
                    floor_vertices, vertices @ R.T + c['pos']]
                continue

            # Loop through lookup methods until shape is found.
            col_id, vis_id = -1, -1
            for method in [_lookup_cache, _lookup_model, _lookup_mesh]:
                (col_id, vis_id) = method(ref)
                if col_id >= 0:
                    break

            # Abort this entry if shape not found.
            if col_id < 0:
                continue

            # Cache any newly created shapes.
            if ref not in shape_map:
                shape_map[ref] = (col_id, vis_id)

            # NOTE(ycho):mass==0 indicates fixed body.
            kwargs['linkMasses'].append(0)
            kwargs['linkCollisionShapeIndices'].append(col_id)
            kwargs['linkVisualShapeIndices'].append(vis_id)
            kwargs['linkPositions'].append(c['pos'])
            kwargs['linkOrientations'].append(c['rot'])
            kwargs['linkInertialFramePositions'].append([0, 0, 0])
            kwargs['linkInertialFrameOrientations'].append([0, 0, 0, 1])
            kwargs['linkParentIndices'].append(0)
            kwargs['linkJointTypes'].append(pb.JOINT_FIXED)
            kwargs['linkJointAxis'].append([0, 0, 0])

    logging.info('# links = {}'.format(len(kwargs['linkMasses'])))

    # NOTE(ycho): Add special handling for floors.
    # NOTE(ycho): Remove this workaround after PR#3238 in the remote is merged.
    floor_col_id = pb.createCollisionShape(pb.GEOM_MESH,
                                           vertices=floor_vertices,
                                           indices=floor_face_indices,
                                           flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
                                           physicsClientId=sim_id)
    kwargs['baseCollisionShapeIndex'] = floor_col_id

    # NOTE(ycho): pybullet does not expose MAX_DEGREE_OF_FREEDOM
    # limit on the maximum number of links possible on the multibody,
    # so we duplicate the hardcoded constant here.
    kwargss = _split_multibody_kwargs(kwargs)
    # kwargss = [kwargss[1]]
    body_ids = [pb.createMultiBody(**kwargs) for kwargs in kwargss]
    print('body_ids = {}'.format(body_ids))

    for body_id in body_ids:
        # Finally, add texture information to the visual shapes.
        # NOTE(ycho): Texture can only be added in the post-processing step
        # through pb.changeVisualShape().
        tex_map = {}
        vis_data = pb.getVisualShapeData(body_id, physicsClientId=sim_id)
        for i, v in enumerate(vis_data):
            # Lookup mesh file.
            mesh_file = v[4].decode('utf-8')
            if not mesh_file:
                continue

            # Find texture file based on mesh file path.
            # NOTE(ycho): Relies on the dataset structure of 3DFRONT.
            texture_file = Path(mesh_file).parent / 'texture.png'
            if not texture_file.exists():
                logging.error(
                    'Texture file : {} does not exist!'.format(texture_file))
                continue

            # Deal with texture id caching logic ...
            tex_id = -1
            if texture_file not in tex_map:
                tex_map[texture_file] = pb.loadTexture(
                    str(texture_file), physicsClientId=sim_id)
            tex_id = tex_map[texture_file]

            # Finally, add texture information to the link.
            pb.changeVisualShape(body_id, i,
                                 textureUniqueId=tex_id,
                                 physicsClientId=sim_id)
    return body_ids


@dataclass
class ThreeDFrontEnvironmentSettings:
    model_dir: str
    scene_file: str = ''
    scene_dir: str = ''
    use_convex_collision: bool = True

    use_fast_aabb_in_placement: bool = True
    max_placement_iter: int = 256


class ThreeDFrontEnvironment(EnvironmentBase):
    def __init__(self, settings: ThreeDFrontEnvironmentSettings):
        self.settings_ = settings
        self.env_ids_ = []
        self.sim_id_ = -1

    @property
    def sim_id(self):
        return self.sim_id_

    def reset(self, sim_id: int):
        self.sim_id_ = sim_id

        # Fetch or lookup scene file to instantiate.
        scene_file = ''
        if self.settings_.scene_file:
            scene_file = self.settings_.scene_file
        else:
            if self.settings_.scene_dir:
                # convenient shorthand.
                d = self.settings_.scene_dir
                scene_file = Path(d)/random.choice(os.listdir(d))
        scene_file = Path(scene_file)
        logging.info('Loading scene file = {}'.format(scene_file))
        if not scene_file.is_file():
            logging.error('Scene file : {} does not exist!'.format(scene_file))
            return

        # Load the scene.
        self.env_ids_ = _load_tdf_scene(str(scene_file),
                                        self.settings_.model_dir, sim_id,
                                        self.settings_.use_convex_collision)

    def place(self, robot_id: int):
        # TODO(ycho): Consider exposing this parameter.
        EPS = 1e-3

        robot_pose = pb.getBasePositionAndOrientation(robot_id,
                                                      physicsClientId=self.sim_id)
        old_pos = robot_pose[0]
        old_rot = robot_pose[1]
        old_z = old_pos[2]

        floor_aabb = np.asarray(pb.getAABB(
            self.env_ids_[0], -1, physicsClientId=self.sim_id), dtype=np.float32)
        robot_aabb = debug_get_full_aabb(self.sim_id, robot_id)
        robot_size = robot_aabb[1] - robot_aabb[0]

        # NOTE(ycho): Shrink the sampled space by the robot radius.
        pos_min = floor_aabb[0, :2] + 0.5 * robot_size[:2]
        pos_max = floor_aabb[1, :2] - 0.5 * robot_size[:2]

        for i in range(self.settings_.max_placement_iter):
            logging.debug('Placement {}/{}'.format(i,
                                                   self.settings_.max_placement_iter))
            # Sample X-Y position from floor AABB.
            x, y = np.random.uniform(pos_min, pos_max)

            # Cast ray from robot top -> floor.
            ray_src = [x, y, floor_aabb[1, 2] +
                       robot_aabb[1, 2] - robot_aabb[0, 2]]
            ray_dst = [x, y, floor_aabb[0, 2] - EPS]
            ray_res = pb.rayTest(ray_src, ray_dst, physicsClientId=self.sim_id)

            # If by some magic, multiple intersections happened,
            # ignore this case.
            if len(ray_res) != 1:
                continue
            ray_res = ray_res[0]

            # The ray must hit env + floor.
            if (ray_res[0] != self.env_ids_[0]) or (ray_res[1] != -1):
                continue

            # Complete the desired new position.
            # new_z = floor_aabb[1, 2] + (old_z - robot_aabb[0, 2])
            new_z = floor_aabb[1, 2] + (old_z - robot_aabb[0, 2])
            new_pos = np.asarray([x, y, new_z + EPS], dtype=np.float32)
            new_rot = old_rot
            # NOTE(ycho): Alternatively, sample from a random SE2 orientation:
            # new_rot = pb.getQuaternionFromEuler([0.0, 0.0, np.random.uniform(-np.pi, np.pi)])

            # Reject the new position if it collides with existing objects.
            if self.settings_.use_fast_aabb_in_placement:
                # NOTE(ycho): This query is conservative, so the returned objects
                # may not actually overlap with the robot. However,
                # perhaps if the object is close enough to the robot that we should
                new_aabb = robot_aabb + new_pos - old_pos
                o = pb.getOverlappingObjects(new_aabb[0], new_aabb[1],
                                             physicsClientId=self.sim_id)
                if o is not None:
                    continue

                pb.resetBasePositionAndOrientation(robot_id,
                                                   new_pos,
                                                   new_rot,
                                                   physicsClientId=self.sim_id)
                break
            else:
                # Actually place the robot where it would go,
                # and then check if it results in a collision.
                # Try placing the robot here now ...
                # NOTE(ycho): Since pybullet uses a default collision margin (0.04 I think?),
                # even this may be a little bit more conservative than the actual collision.
                pb.resetBasePositionAndOrientation(robot_id,
                                                   new_pos,
                                                   new_rot,
                                                   physicsClientId=self.sim_id)
                col = False
                for env_id in self.env_ids_:
                    cpts = pb.getClosestPoints(env_id, robot_id,
                                               np.inf,
                                               physicsClientId=self.sim_id)
                    for cpt in cpts:
                        if cpt[8] < 0:
                            col = True
                            break
                    # Early exit if collision found
                    if col:
                        break
                # Continue searching if collision found
                if col:
                    continue

                # All is well! break.
                break
