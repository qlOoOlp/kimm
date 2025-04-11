import numpy as np
from simulator.src.robot.rb5_wrapper import rb5Wrapper
from simulator.src.trajectory.trajManager import jointTraj, SE3Traj
from rclpy.node import Node

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

import time


class rb5Controller:
    def __init__(self, robot : rb5Wrapper, clock):
        self.robot = robot
        self.clock = clock

        # Control : Time
        self.stime = None
        self.ctime = None
        self.duration  = None

        # Control : Target
        self.targetJoint = None
        self.targetSE3 = pin.SE3()

        # Robot Data
        self.nq = self.robot.state.nq

        # Trajectory
        self.jointTraj = jointTraj(self.nq)
        self.se3Traj = SE3Traj()

    def initJointPosture(self, target : np.array, duration):
        self.stime = self.clock.now().nanoseconds/1e9
        self.duration = duration
        self.targetJoint = np.copy(target)

        self.jointTraj.setStartTime(self.stime)
        self.jointTraj.setDuration(self.duration)
        self.jointTraj.setTargetSample(self.targetJoint)
        self.jointTraj.setInitSample(self.robot.state.q)

    def controlJointPosture(self):
        self.ctime = self.clock.now().nanoseconds/1e9
        self.jointTraj.setCurrentTime(self.ctime)
        qdes = self.jointTraj.computeNext()

        return qdes
    
    def initSE3(self, target : pin.SE3, duration):
        self.stime = self.clock.now().nanoseconds/1e9
        self.duration = duration
        self.targetSE3 = target

        self.se3Traj.setStartTime(self.stime)
        self.se3Traj.setDuration(self.duration)
        self.se3Traj.setTargetSample(self.targetSE3)
        self.se3Traj.setInitSample(self.robot.state.oMi)
        

    def controlSe3(self):
        self.ctime = self.clock.now().nanoseconds/1e9
        self.se3Traj.setCurrentTime(self.ctime)
        se3Ref = self.se3Traj.computeNext()
        print("Target \n",se3Ref)
        oMi = self.robot.state.oMi
        dMi = oMi.inverse() * se3Ref
        dMi_6d_motion = pin.log(dMi)
        dMi_6d = np.zeros((6))
        dMi_6d[:3] = dMi_6d_motion.linear
        dMi_6d[3:] = dMi_6d_motion.angular
        
        
        # print(dMi_6d)
        
        p_gain = 10.0
        
        qdot = np.linalg.pinv(self.robot.state.J) @ (p_gain * dMi_6d)
        qdes = self.robot.state.q + qdot * 0.01
        

        return qdes


        