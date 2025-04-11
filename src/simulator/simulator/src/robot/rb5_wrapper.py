import os
import numpy as np
from numpy import nan_to_num, sum
from numpy.linalg import pinv

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

from ament_index_python.packages import get_package_share_directory


class state():
    def __init__(self):
        self.q: np.array
        self.v: np.array
        self.a: np.array
        self.q_des: np.array
        self.v_des: np.array
        self.a_des: np.array
        self.q_ref: np.array
        self.v_ref: np.array

        self.acc: np.array
        self.tau: np.array
        self.torque: np.array
        self.v_input: np.array

        self.nq: np.array
        self.nv: np.array
        self.na: np.array

        self.id: np.array
        self.G: np.array
        self.M: np.array
        self.J: np.array
        self.M_q: np.array

        self.oMi : pin.SE3


class rb5Wrapper(RobotWrapper):
    def __init__(self):
        package_name = "rbpodo_description"
        urdf_file_path = os.path.join(get_package_share_directory(package_name), "robots", "urdf", "rb5_850e.urdf")
        self.__robot = self.BuildFromURDF(urdf_file_path)

        self.data, self.__collision_data, self.__visual_data = \
            pin.createDatas(self.__robot.model, self.__robot.collision_model, self.__robot.visual_model)
        self.model = self.__robot.model
        self.state = state()

        self.__ee_joint_name = "wrist3"
        self.state.id = self.index(self.__ee_joint_name)

        self.state.nq = self.__robot.nq
        self.state.nv = self.__robot.nv
        self.state.na = self.__robot.nv

        self.state.q = zero(self.state.nq)
        self.state.v = zero(self.state.nv)
        self.state.a = zero(self.state.na)
        self.state.acc = zero(self.state.na)
        self.state.tau = zero(self.state.nv)


    def computeAllTerms(self):

        pin.computeAllTerms(self.model, self.data, self.state.q, self.state.v)
        # pin.compute
        self.state.G = self.nle(self.state.q, self.state.v)     # NonLinearEffects
        self.state.M = self.mass(self.state.q)                  # Mass
        self.state.J = self.getJointJacobian(self.state.id)
        self.state.a = pin.aba(self.model, self.data, self.state.q, self.state.v, self.state.tau)
        self.state.oMi = self.data.oMi[self.state.id]
