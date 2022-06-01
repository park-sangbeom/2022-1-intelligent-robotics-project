import matplotlib.pyplot as plt
import numpy as np
import time
""" CUSTOM CLASS """
from structure.class_robot import *
""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient
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

robot       = ROBOT()
client      = None
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)

def grasped(graspclient):
    slave = 65
    flag = graspclient.read_input_registers(268,1,unit=slave).registers[0]
    flag = (flag&0x02) == 2
    if flag:
        print("Grasp detected: True")
    return flag

def graspable(graspclient):
    slave = 65
    flag = graspclient.read_input_registers(268,1,unit=slave).registers[0]
    flag = (flag&0x08) == 8
    if flag:
        print("Grasp availablity: False")
    return flag

def reset_tool(graspclient):
    print('Tool reseting')
    toolslave = 63
    graspclient.write_register(0,2,unit=toolslave)
    time.sleep(3)
    print("Reset Fininshed", end='\r')

def close_grasp(force,width,graspclient):
    # If grasped, reset&openGrasp
    if grasped(graspclient):
        reset_tool(graspclient)
        open_grasp(400,1000,graspclient)
    # If S1 activated, reset
    if graspable(graspclient):
        reset_tool(graspclient)
    slave = 65
    graspclient.write_registers(0,[force,width,1],unit=slave)
    time.sleep(1)

def open_grasp(force,width,graspclient):
    # If S1 activated, reset
    if graspable(graspclient):
        reset_tool(graspclient)
    slave = 65
    graspclient.write_registers(0,[force,width,1],unit=slave)
    time.sleep(1)

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

def real_move(joint_list, num_interpol):
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
            vel = (q-prev_q)/num_interpol # TODO: CHECK VELOCITY
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
        prev_q = q
        d+=0.002 
    try:
        print("MOVE")
        client.send_goal(g)
        client.wait_for_result()
    except:
        raise

def main(start_pos, target_pos, num_interpol, desired_vel):
    global client
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
            prepose()
            s = time.clock()
            q_list_forward  = robot.waypoint_plan(start_pos, target_pos, num_interpol, desired_vel)
            real_move(q_list_forward, num_interpol)
            time.sleep(1)
            close_grasp(230, 500, graspclient)
            print("FINISHED")
            print("WALL CLOCK: {}".format(time.clock()-s))
        else:
            print("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == "__main__":
    rospy.init_node("REAL_WORLD")
    realsense = Realsense435()
    middle_points = callback(realsense.point_cloud)
    middle_points = middle_points[0]
    x = middle_points[0]
    y = middle_points[1]
    z = middle_points[2]
    print("x:{}, y:{}, z:{}".format(x,y,z))
    main(start_pos=np.array([0.6, 0, 0.85]), target_pos=np.array([x, y, z]), num_interpol=5, desired_vel=0.2)
    # main(start_pos=np.array([0.6, 0, 0.85]), target_pos=np.array([0.9, 0, 0.85]), num_interpol=5, desired_vel=0.2)


