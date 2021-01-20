#!/usr/bin/env python3

import pybullet as pb
import numpy as np


class SceneGraph(object):
    """
    Abstract scene graph modelling object relations without pose designations.
    """

    def __init__(self):
        self.nodes_ = []
        self.edges_ = []

    def add_node(self, node: str):
        self.nodes_.append(node)

    def add_edge(self, source: str, target: str):
        self.edges_.append((source, target))


class SceneGraphWithPose(SceneGraph):
    def __init__(self):
        self.poses_ = {}

    def set_node_pose(self, node: str, pose: np.ndarray):
        pass


def main():
    graph_def = {
        'living_room': ''
    }
    pass


if __name__ == '__main__':
    main()
