from structure.class_robot import ROBOT
from structure.class_structure import *
import rospy 
import numpy as np 
import math 

if __name__ == "__main__":
    chain = CHAIN(_file_name="structure/utils/ur5e_onrobot.urdf", verbose=False)
    chain.add_joi_to_robot()
    chain.add_link_to_robot()
    chain.fk_chain(1)
    wrist_pos=[0.6,-0.2,0.85]
    angle = math.atan2(-0.2, (0.9))
    finger_pos = wrist_pos+[0.15,0,0]
    base_pos = [0.18, 0.85,0]
    print(angle)
    variable = make_ik_input(target_name = ['shoulder_pan_joint'],
                                target_pos  = [base_pos],
                                target_rot  = [[0, 0, angle]],
                                solve_pos   = [1],
                                solve_rot   = [1],
                                weight_pos  = 1,
                                weight_rot  = 1,
                                disabled_joi_id = [],
                                joi_ctrl_num=7) # Including base joint   
    # variable = make_ik_input(target_name = ['wrist_3_joint', 'shoulder_pan_joint'],
    #                             target_pos  = [wrist_pos,base_pos],
    #                             target_rot  = [[0, 3.14, -1.57],[0, 0, angle]],
    #                             solve_pos   = [1, 1],
    #                             solve_rot   = [1,0],
    #                             weight_pos  = 1,
    #                             weight_rot  = 1,
    #                             disabled_joi_id = [],
    #                             joi_ctrl_num=7) # Including base joint     

    # variable = make_ik_input(target_name = ['wrist_3_joint', 'gripper_tcp_joint'],
    #                             target_pos  = [wrist_pos, finger_pos, base_pos],
    #                             target_rot  = [[0, 3.14, -1.57],[0,0,0], [0, 0, angle]],
    #                             solve_pos   = [1, 1, 0],
    #                             solve_rot   = [1, 0, 1],
    #                             weight_pos  = 1,
    #                             weight_rot  = 1,
    #                             disabled_joi_id = [],
    #                             joi_ctrl_num=7) # Including base joint       
    q_list = chain.get_q_from_ik(variable)
    control_q_list = q_list[1:7] # Excluding base joint 
    print(control_q_list)