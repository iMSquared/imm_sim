#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty



class RobotBase(ABC):
    @abstractmethod
    def reset(self, sim_id: int):
        raise NotImplementedError('')

    @abstractproperty
    def robot_id(self):
        raise NotImplementedError('')


