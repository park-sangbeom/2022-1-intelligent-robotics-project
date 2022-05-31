from turtle import home
import matplotlib.pyplot as plt
import numpy as np
import time
""" CUSTOM CLASS """
from structure.class_robot import *
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import time 

robot       = ROBOT()
client      = None
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)

def prepose():
    try: 
        q = [0.9, -0.6596, 1.3364, 0.0350, 0, 0]
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos   = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def homepose():
    try: 
        q = [2.0, -0.6596, 1.3364, 0.0350, 0, 0]
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points[JointTrajectoryPoint(positions=q, velocities=[0.]*6, time_from_start=rospy.Duration(1.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def real_move(joint_list):
    print("Check")
    print("joint_list", joint_list)
    print("joint_list shape", joint_list.shape)
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    for i, q in enumerate(joint_list):
        if i==0:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            d=3
        else:
            vel = (q-prev_q)/5 # TODO: CHECK TIME
            print("q", q, "prev_q",prev_q)
            print("vel", vel)
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
        prev_q = q
        d+=0.002 # TODO: CHECK TIME
    try:
        print("MOVE")
        client.send_goal(g)
        client.wait_for_result()
    except:
        raise

def main():
    global client
    try:
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        print("Please make sure that your robot can move freely between these poses before proceeding!")
        inp = raw_input("Continue? y/n: ")[0]
        # inp = input("Continue? y/n: ")
        print("inp", inp)
        if (inp == 'y'):
            prepose()
            s = time.clock()
            q_list_forward  = robot.waypoint_plan(np.array([0.6, 0, 0.85]), np.array([0.9, 0, 0.85]), 5, 0.2)
            print("q_list_forward",q_list_forward)
            print("q_shape", q_list_forward.shape)
            real_move(q_list_forward)
            time.sleep(1)
            # q_list_backward = robot.waypoint_plan(np.array([0.9, 0, 0.85]), np.array([0.6, 0, 0.85]), 5, 0.2)
            # real_move(q_list_backward)
            print("finished")
            print(time.clock()-s)

        else:
            print("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise



if __name__ == "__main__":
    rospy.init_node("REAL_WORLD")
    main()


