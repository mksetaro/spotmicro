import pybullet_data
import pybullet
import abc
import math


class BaseSimulationScenario(abc.ABC):
    def __init__(self, name) -> None:
        self._name = name

    @property
    def name(self):
        return self._name

    # do not override
    def load(self) -> None:
        orn = pybullet.getQuaternionFromEuler([math.pi / 30 * 0, 0 * math.pi / 50, 0])
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeUid = pybullet.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)
        pybullet.changeDynamics(planeUid, -1, lateralFriction=1)
        pybullet.setGravity(0, 0, -9.81)
        self.create_environment()

    @abc.abstractclassmethod
    def create_environment(self):
        pass

    def step():
        pass
