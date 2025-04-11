import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy

from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray

import time
import numpy as np

from simulator.src.robot.rb5_wrapper import rb5Wrapper
from simulator.src.controller.controller import rb5Controller
from rclpy.time import Time
from rclpy.duration import Duration

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *


class rb5Simulator(Node):
    def __init__(self):
        super().__init__("rb5_gazebo_simulator")


        self.robot = rb5Wrapper()
        self.ctrl = rb5Controller(self.robot, self.get_clock())
        
        self.targetTest = None


        self.joint_names = None
        self.interface_name = None
        self.interface_values = None
        self.joint_order = [
            "base", "shoulder","elbow","wrist1","wrist2","wrist3"
        ]

        # model state subscriber
        subscribe_qos_profile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.joint_state_subscriber = self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.joint_state_callback, subscribe_qos_profile)


        cal_timer_period = 0.01  # seconds
        pub_timer_period = 0.01  # seconds
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)
        self.pub_timer = self.create_timer(pub_timer_period, self.pub_timer_callback)

        # arm publisher
        self.arm_msg = Float64MultiArray()
        self.arm_publisher = self.create_publisher(Float64MultiArray, '/rb5_joint_controller/commands', 10)

        self.controlFlag = False
        self.initControl = True
        self.iter = 0


    def cal_timer_callback(self):

        self.iter +=1
        self.target = np.array([0.0,0.0, np.pi/2, 0.0, np.pi/2, 0.0])
        
        # TEST
        self.targetTest = pin.SE3()
        # self.targetTest.translation = np.array([0.5, 0.0, 0.5])
        # self.targetTest.rotation = np.zeros((3,3))
        # self.targetTest.rotation[0,0] = 1.0
        # self.targetTest.rotation[2,1] = -1.0
        # self.targetTest.rotation[1,2] = 1.0


        self.targetTest.translation = self.robot.state.oMi.translation
        self.targetTest.translation[0] += 0.05
        # self.targetTest.rotation = self.robot.state.oMi.rotation
        self.targetTest.rotation = np.zeros((3,3))
        self.targetTest.rotation[0,2] = -1.0
        self.targetTest.rotation[2,1] = -1.0
        self.targetTest.rotation[1,0] = 1.0

        qdes = np.copy(self.target)
        if(self.iter>300):
            self.controlFlag = True

        if self.controlFlag:
            if self.initControl:
                # self.ctrl.initJointPosture(self.target, 3.0)
                self.ctrl.initSE3(self.targetTest, 5.0)
                
                self.initControl = False
            else:
                # qdes = self.ctrl.controlJointPosture()
                qdes = self.ctrl.controlSe3()
            
        print("Current : \n",self.robot.state.oMi)
        self.arm_msg.data = qdes.tolist()



    def pub_timer_callback(self):
        self.arm_publisher.publish(self.arm_msg)
        return


    def joint_state_callback(self, msg):
        self.joint_names = msg.joint_names
        self.interface_name = [iv.interface_names for iv in msg.interface_values]
        values = [list(iv.values) for iv in msg.interface_values]

        index_map = [self.joint_names.index(joint) for joint in self.joint_order]
        self.interface_values = np.array([values[i] for i in index_map])  # shape: [num_joints, num_interfaces]

        self.robot.state.q = self.interface_values[:, 0]  # position
        self.robot.state.v = self.interface_values[:, 1]  # velocity
        self.robot.computeAllTerms()

# ISS
def main():
    rclpy.init()
    node = rb5Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

