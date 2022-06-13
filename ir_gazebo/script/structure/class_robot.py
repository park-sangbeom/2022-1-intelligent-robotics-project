from calendar import c
from numpy import interp
import rospy
from structure.class_structure import *
from structure.utils.util_ik import *
from structure.utils.util_structure import *
from structure.utils.util_parser import PARSER
from structure.class_fcl import PyFCL

class ROBOT: 
    def __init__(self, _file_name="structure/utils/ur5e_onrobot.urdf"): 
        self.chain  = CHAIN(_file_name)
        self.parser = PARSER(_file_name)
        self.fcl    = PyFCL(_verbose=False)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()

    def linear_move(self, start_pos, target_pos, desired_vel, offset_angle, num_interpol):
        freq = 500
        """ Start Pose """
        variable_start = make_ik_input(target_name = ['wrist_3_joint', 'gripper_tcp_joint'],
                                    target_pos  = [[0,0,0], start_pos],
                                    target_rot  = [[0, 3.14, offset_angle-1.57],[0,0,0]],
                                    solve_pos   = [0, 1],
                                    solve_rot   = [1, 0],
                                    weight_pos  = 1,
                                    weight_rot  = 1,
                                    disabled_joi_id = [0],
                                    joi_ctrl_num=7) # Including base joint        
        q_list_start = self.chain.get_q_from_ik(variable_start)
        q_list_start = q_list_start[1:7] # Excluding base joint
        ctrl_q_list_start = q_list_start.reshape(-1,)        
        ctrl_q_list_start[0] = offset_angle
        ctrl_q_list_start[4] = 1.57
        """ Goal Pose """
        variable = make_ik_input(target_name = ['wrist_3_joint', 'gripper_tcp_joint'],
                                    target_pos  = [[0,0,0], target_pos],
                                    target_rot  = [[0, 3.14, offset_angle-1.57],[0,0,0]],
                                    solve_pos   = [0, 1],
                                    solve_rot   = [1, 0],
                                    weight_pos  = 1,
                                    weight_rot  = 1,
                                    disabled_joi_id = [0],
                                    joi_ctrl_num=7) # Including base joint
        q_list_goal = self.chain.get_q_from_ik(variable)
        q_list_goal = q_list_goal[1:7] # Excluding base joint
        ctrl_q_list_goal = q_list_goal.reshape(-1,)        
        desired_time = get_desired_time(start_pos, target_pos, desired_vel)
        interpoled_q = np.linspace(start=ctrl_q_list_start, stop=ctrl_q_list_goal, num=int(freq*desired_time)*num_interpol)
        return interpoled_q

   