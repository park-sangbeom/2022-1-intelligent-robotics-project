from numpy import interp
import rospy
from class_structure import *
from utils.util_ik import *
from utils.util_structure import *
from utils.util_parser import PARSER
from class_fcl import PyFCL

class ROBOT: 
    def __init__(self, _file_name="structure/ur5e_onrobot.urdf"): #ir_gazebo/script/structure/ur5e_onrobot.urdf
        self.chain  = CHAIN(_file_name)
        self.parser = PARSER(_file_name)
        self.fcl    = PyFCL(_verbose=False)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()
        self.joint_tree = self.chain.joint 
        self.link_tree  = self.chain.link

    def waypoint_plan(self, start_pos, target_pos, num_interpol): 
        offset = [0.15, 0, 0]
        total_q_list = [] 
        interpoled_points = np.linspace(start=start_pos, stop=target_pos, num=num_interpol)
        for num in range(num_interpol):
            wrist_point  = interpoled_points[num]
            finger_point = wrist_point + offset
            if ((num) == (num_interpol-1)):
                q_list = self.chain.get_q_from_ik(variable) 
                control_q_list = q_list[1:7]
                reshaped_q = control_q_list.reshape([6,])
                total_q_list.append(reshaped_q) # Get manipulator joints 
                break 
            else: 
                variable = make_ik_input(target_name = ['wrist_3_joint', 'gripper_finger1_finger_tip_joint'],
                                         target_pos  = [wrist_point, finger_point],
                                         target_rot  = [[0, 3.14, -1.57],[0,0,0]],
                                         solve_pos   = [1, 1],
                                         solve_rot   = [1, 0],
                                         weight_pos  = 1,
                                         weight_rot  = 1,
                                         disabled_joi_id = [],
                                         joi_ctrl_num=7)             
                q_list = self.chain.get_q_from_ik(variable)
                control_q_list = q_list[1:7]
                reshaped_q = control_q_list.reshape([6,])
                total_q_list.append(reshaped_q) # Get manipulator joints 
        desired_time = get_desired_time(start_pos, target_pos, desired_vel=0.2)
        interpoled_q = q_interpolation(total_q_list, desired_time, num_interpol)
        return interpoled_q 

if __name__== "__main__": 
    robot = ROBOT()
    start = CONFIG["start_joints"]
    target = np.array([0, -0.,-1.89270333133803,0,-1.6044670780114405,-1.5697175550123657,2.8396582340451655])
    obj1 = {"name":"obj1", "type":"capsule", "position":[0.45, 0, 0.7], "orientation":[0, 0, 0], "size":[0.2, 0.2, 0.2]}
    obj2 = {"name":"obj2", "type":"capsule", "position":[-0.45, 0.2, 0.7], "orientation":[0, 0, 0], "size":[0.2, 0.2, 0.2]}
    obs_lst = [obj1, obj2]
    inter_q = robot.waypoint_plan(np.array([0.6, 0, 0.85]), np.array([0.9,0,0.85]), 5)
