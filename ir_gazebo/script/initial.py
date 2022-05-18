import rospy
from structure import *
from utils_ik import *
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np 

rospy.init_node('main')

class UR5:
    def __init__(self):
        self.chain = Chain()
        self.joint_list = joint_list_config()
        self.ur5_jc = self.chain.add_joi_to_robot(self.joint_list)
        self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)
        self.gripper_pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size = 10)

    def get_q_from_ik(self,variable):
        # Heurisitc 
        #joint2 = JointClass(name='shoulder_lift_joint', id=3, mother=2, child=[4], q=-1, a=column_v(0,1,0), b=column_v(0, 0.13585, 0), 
        #            p=column_v(0, 0, 0), R=rot_y(3.141592/2))    
        #self.ur5_jc[2] = joint2
        self.chain.fk_chain(1)
        self.chain.aug_inverse_kinematics_LM(variable)
        q_list = get_q_chain(self.ur5_jc)
        q_list = q_list[:7] #Fromb base_offset to Wrist_3_joint (Control Joint)
        return q_list 

# Varialbe for single target IK
#variable = np.array([['wrist_3_joint'], [[0.7, 0.2, 0.73]], [[-1.57, 0, 1.57 ]], [1], [1], 1, 1, [], 7])
####################################
# Variables for multi target IK
gripper_height = 0.15
end_tip_target_pos = [0.8, 0.2, 1.2]
wrist3_target_pos = [0.8 - gripper_height, 0.2, 1.2]
aug_variable = np.array([['wrist_3_joint', 'gripper_finger1_finger_tip_joint'], [wrist3_target_pos,end_tip_target_pos], [[0, 3.14, -1.57],[0,0,0]], [1,1], [1,0], 1, 1, [], 7])

###################################
# Instance & IK solve
ur = UR5()
q_list = ur.get_q_from_ik(aug_variable)
q = q_list[1:7]
rospy.sleep(0.08)
###################################
# Add the joints info
arm_joint = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint', 
                          'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
gripper_joint = ['gripper_finger1_joint']
###################################
# Setting for publisher 1
arm= JointTrajectory()
gripper= JointTrajectory()
###################################
# Setting for publisher 2
arm_value = JointTrajectoryPoint()
gripper_joint_value = JointTrajectoryPoint()
###################################
arm.joint_names = arm_joint
gripper.joint_names= gripper_joint
q = [2.0, -0.6596, 1.3364, 0.0350, 0, 0]
arm_value.positions = q
gripper_q = [0.61]
gripper_q = np.reshape(gripper_q,(1,1))
gripper_joint_value.positions = gripper_q[0]
arm_value.time_from_start = rospy.Duration.from_sec(1)
gripper_joint_value.time_from_start = rospy.Duration.from_sec(1)
arm.points.append(arm_value)
gripper.points.append(gripper_joint_value)
ur.arm_pub.publish(arm)
ur.gripper_pub.publish(gripper)
print(q)
print('check')
###################################
    