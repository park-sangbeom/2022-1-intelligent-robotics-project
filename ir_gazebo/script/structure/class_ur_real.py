import matplotlib.pyplot as plt
import numpy as np
import time
""" CUSTOM CLASS """
from structure.class_robot import *
from structure.utils.util_ik import *
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

class RealUR:
    def __init__(self):
        rospy.init_node("REAL_WORLD")
        self.robot       = ROBOT()
        self.client      = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)
        self.up_offset   = np.array([0, 0, 0.25])

    def move_arm(self, joints):
        try: 
            q = joints
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

    def init_pose(self):
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

    def start_pose(self):
        try: 
            q = [-3.35916187e-01, -13.90421628e-01,  2.52584599e+00, -1.13542436e+00, 1.23408381e+00, -1.59279665e-03]
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
                vel = (q-prev_q) #num_interpol # TODO: CHECK VELOCITY
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

    def grasp_single_obj(self, target_pos, num_interpol, desired_vel, direction_offset):
        try:
            graspclient = ModbusTcpClient('192.168.0.110')
            slave = 65
            toolslave = 63
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            """ Initialize """
            self.init_pose()
            open_grasp(230, 1000, graspclient)
            """ Ready to start """
            self.start_pose
            start_pos = [0.71, 0, 0.83]
            # Get q list using linear interpolation plan 
            q_list = self.robot.linear_move(start_pos, target_pos, desired_vel, direction_offset, num_interpol, "forward")
            """ Linear move """
            # Move UR5e 
            self.real_move(q_list, num_interpol)
            time.sleep(1)
            # Close gripper to grasp the target object
            close_grasp(250, 700, graspclient)
            """ Move to upward """
            # Get q list to move upward 
            start_pos2 = get_curr_wrist_pos(self.robot.chain.joint)
            q_list_upward  = self.robot.linear_move(start_pos2, start_pos2+self.up_offset, desired_vel, direction_offset, num_interpol, "upward")
            # Move UR5e 
            self.real_move(q_list_upward, num_interpol)
            time.sleep(1)
            """ Back to initialize pose """
            # Initialize 
            self.init_pose()
            time.sleep(1)
            # Open gripper to place the target object 
            open_grasp(250, 1000, graspclient)
            time.sleep(1)
            print("Finish plan")

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

