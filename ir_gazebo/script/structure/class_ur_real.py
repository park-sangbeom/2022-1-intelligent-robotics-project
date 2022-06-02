import matplotlib.pyplot as plt
import numpy as np
import time
""" CUSTOM CLASS """
from structure.class_robot import *
""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient
from structure.utils.util_grasp import *
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import time 
""" FOR REALSENSE """
from structure.class_realsense435 import Realsense435
from structure.utils.util_realsense import callback

class UR_REAL:
    def __init__(self):
        rospy.init_node("REAL_WORLD")
        self.robot       = ROBOT()
        self.client      = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)

    def prepose(self):
        try: 
            q = [0.9, -0.6596, 1.3364, 0.0350, 0, 0]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def real_move(self, joint_list, num_interpol):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        for i, q in enumerate(joint_list):
            if i==0:
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joints_pos   = joint_states.position
                g.trajectory.points = [
                    JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                    JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
                d=3
            else:
                vel = (q-prev_q)/num_interpol # TODO: CHECK VELOCITY
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
            prev_q = q
            d+=0.002 
        try:
            print("MOVE")
            self.client.send_goal(g)
            self.client.wait_for_result()
        except:
            raise

    def grasp_single_obj(self, start_pos, target_pos, num_interpol, desired_vel):
        try:
            graspclient = ModbusTcpClient('192.168.0.110')
            slave = 65
            toolslave = 63
            client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            client.wait_for_server()
            print("Connected to server")
            print("Please make sure that your robot can move freely between these poses before proceeding!")
            inp = input("Continue? y/n: ")[0]
            if (inp == 'y'):
                self.prepose()
                open_grasp(230, 1000, graspclient)
                q_list_forward  = self.robot.waypoint_plan(start_pos, target_pos, num_interpol, desired_vel)
                self.real_move(q_list_forward, num_interpol)
                time.sleep(1)
                close_grasp(230, 700, graspclient)
                q_list_upward  = self.robot.waypoint_plan(target_pos, target_pos+[0, 0, 0.3], num_interpol, desired_vel)
                self.real_move(q_list_upward, num_interpol)
                time.sleep(2)
                self.prepose()
                open_grasp(230, 1000, graspclient)
                print("FINISHED")
            else:
                print("Halting program")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

