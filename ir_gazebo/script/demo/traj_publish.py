import rospy
import numpy as np 
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

print("Packages Loaded.")

rospy.init_node('publish_joints')
arm_joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint', 
                'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
gripper_joint_names = ['gripper_finger1_joint']
rate    = rospy.Rate(50)
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

def move_gripper(joint):
    global gripper_joint_names, gripper_pub
    """ Open- Close Gripper """
    gripper        = JointTrajectory()
    gripper_value  = JointTrajectoryPoint()
    gripper.header = Header()
    gripper.joint_names = gripper_joint_names
    gripper_value.positions = [joint] # Open pose
    gripper_value.time_from_start = rospy.Duration.from_sec(0.01)
    gripper.points.append(gripper_value)
    gripper_pub.publish(gripper)


def move_arm(joint_seq): 
    Flag = True 
    global arm_joint_names, rate, arm_pub
    while Flag:
        """ Move to pick an object """
        for idx, joints in enumerate(joint_seq):
            joint = joints.reshape([6,])
            arm = JointTrajectory()
            arm_value = JointTrajectoryPoint()
            arm.header = Header()
            arm.joint_names= arm_joint_names
            arm_value.positions       = joint
            arm_value.time_from_start = rospy.Duration.from_sec(0.01)
            arm.points.append(arm_value)
            arm_pub.publish(arm)
            rate.sleep()    
            if idx+1 == len(joint_seq): 
                Flag = False 

def main():
    """ Load joint values """
    # load_data       = np.load("data/sample_traj.npz")
    load_data_pick  = np.load("data/sample_traj_pick.npz")
    load_data_place = np.load("data/sample_traj_place.npz")
    load_data_home  = np.load("data/sample_traj_home.npz")

    joints_seq_pick  = load_data_pick["joint"]
    joints_seq_place = load_data_place["joint"]
    joints_seq_home  = load_data_home["joint"]

    move_arm(joints_seq_pick)
    move_gripper(0.8)
    # move_arm(joints_seq_place)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")