from calendar import c
from numpy import interp
import rospy
from structure.class_structure import *
from structure.utils.util_ik import *
from structure.utils.util_structure import *
from structure.utils.util_parser import PARSER
from structure.class_fcl import PyFCL

class ROBOT: 
    def __init__(self, _file_name="structure/utils/ur5e_onrobot.urdf"): #ir_gazebo/script/structure/ur5e_onrobot.urdf  #structure/ur5e_onrobot.urdf
        self.chain  = CHAIN(_file_name)
        self.parser = PARSER(_file_name)
        self.fcl    = PyFCL(_verbose=False)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()

    def waypoint_plan(self, start_pos, target_pos, num_interpol, desired_vel): 
        offset = [0.277, 0, 0] 
        total_q_list = [] 
        interpoled_points = np.linspace(start=start_pos, stop=target_pos, num=num_interpol)
        for num in range(num_interpol):
            wrist_pos  = interpoled_points[num]
            finger_pos = wrist_pos + offset
            if ((num) == (num_interpol-1)):
                q_list = self.chain.get_q_from_ik(variable) 
                control_q_list = q_list[1:7] # Excluding base joint 
                reshaped_q = control_q_list.reshape([6,])
                total_q_list.append(reshaped_q) # Get manipulator joints 
                break 
            else: 
                variable = make_ik_input(target_name = ['wrist_3_joint', 'gripper_finger1_finger_tip_joint'],
                                         target_pos  = [wrist_pos, finger_pos],
                                         target_rot  = [[0, 3.14, -1.57],[0,0,0]],
                                         solve_pos   = [1, 1],
                                         solve_rot   = [1, 0],
                                         weight_pos  = 1,
                                         weight_rot  = 1,
                                         disabled_joi_id = [],
                                         joi_ctrl_num=7) # Including base joint       
                q_list = self.chain.get_q_from_ik(variable)
                control_q_list = q_list[1:7] # Excluding base joint 
                reshaped_q = control_q_list.reshape([6,])
                total_q_list.append(reshaped_q) # Get manipulator joints 
        desired_time = get_desired_time(start_pos, target_pos, desired_vel)
        interpoled_q = q_interpolation(total_q_list, desired_time, num_interpol)
        return interpoled_q 


