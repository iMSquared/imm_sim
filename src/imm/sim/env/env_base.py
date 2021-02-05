#!/usr/bin/env python3

from abc import ABC, abstractmethod, abstractproperty


class EnvironmentBase(ABC):
    @abstractmethod
    def reset(self, sim_id: int):
        raise NotImplementedError('')
