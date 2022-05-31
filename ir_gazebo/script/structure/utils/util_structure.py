import numpy as np
from util_fk import *
from util_ik import * 

def update_q_chain(robot_jc, q_list, ctrl_joint):
    for idx in range(len(robot_jc)):
        if idx < ctrl_joint:
            robot_jc[idx].q = q_list[idx]

def get_scale(robot_lc):
    link_lst = []
    arr = []
    for link in robot_lc: 
        for lc in link.scale.split():
            lc = float(lc)
            arr.append(lc)
        link_lst.append(arr)
        arr = []
    return link_lst

def get_color_from_urdf(color_value):
    color_value_list = []
    for color in color_value.split():
        color_value_list.append(float(color))
    return color_value_list

def get_link_color(robot_lc):
    n_link = len(robot_lc)
    color_list = []
    for idx in range(n_link):
        color_list.append(robot_lc[idx].color)
    return color_list

def get_mesh_chain(robot_lc):
    n_link = len(robot_lc)
    mesh_path_list = []
    for idx in range(n_link):
        mesh_path_list.append(robot_lc[idx].mesh_path) 
    return mesh_path_list 

def get_mother_id_chain(robot_jc):
    n_jc = len(robot_jc)
    id_list = []
    for idx in range(n_jc):
        id_list.append(robot_jc[idx].mother)
    return id_list

def get_scale_chain(robot_lc):
    n_link = len(robot_lc)
    scale_list = [] 
    for idx in range(n_link):
        scale_list.append(robot_lc[idx].scale)
    return scale_list 

def get_axis_chain(robot_jc):
    n_link = len(robot_jc)
    axis_list =[]
    for idx in range(n_link):
        axis_list.append(robot_jc[idx].a)
    return axis_list 

def get_R_offset_chain(robot_jc):
    n_link = len(robot_jc)
    R_offset_list = []
    for idx in range(n_link):
        R_offset_list.append(robot_jc[idx].R_offset)
    return R_offset_list

def get_q_chain(robot_jc):
    n_joint = len(robot_jc)
    q_list = np.zeros((n_joint,1))
    for idx in range(n_joint):
        q_list[idx] = robot_jc[idx].q 
    return q_list 

def get_p_chain(robot_jc):
    n_joint = len(robot_jc)
    p_list = np.zeros((n_joint,3))
    for idx in range(n_joint):
        p_list[idx] = robot_jc[idx].p.T
    return p_list 
  

def get_center_p_chain(robot_lc):
    n_link = len(robot_lc)#TODO:End joint is None, Should fix it later. 
    p_list = np.zeros((n_link,3))
    for idx in range(n_link):
        p_list[idx] = robot_lc[idx].cap.center_p.T
    return p_list   

def get_name_chain(robot_lc): 
    n_link = len(robot_lc)
    name_list = [] 
    for idx in range(n_link): 
        name_list.append(robot_lc[idx].name)
    return name_list 


def get_R_chain(robot_jc):
    n_joint = len(robot_jc)
    R_list = [] 
    for idx in range(n_joint):
        R_list.append(robot_jc[idx].R)
    return R_list


def get_height_chain(robot_lc):
    height_lst = []
    n_link = len(robot_lc)
    for idx in range(n_link):  
        height_lst.append(robot_lc[idx].cap.height)
    return height_lst


def get_rpy_from_R_mat(R_list):
    rpy_list = []
    for idx in range(len(R_list)):
        rpy = decompose_rotation_matrix(R_list[idx])
        rpy_list.append(rpy)
    return rpy_list 

def get_rev_joi_chain(robot_jc, ctrl_num):
    n_joint = len(robot_jc)
    revolute_type = [] 
    for idx in range(n_joint):
        if robot_jc[idx].type == 'revolute' and idx <=ctrl_num: # This is essenstial condition, to remove gripper joint in IK slover
            revolute_type.append(robot_jc[idx].id)
    return revolute_type 

def q_interpolation(joint_val_seq, desired_time, num_interpol):
    freq = 500
    joint_seq_arr    = np.array(joint_val_seq, dtype=object)
    joint_seq        = joint_seq_arr.T
    new_q_list = []
    for idx in range(len(joint_seq)):
        print("idx",idx)
        if idx ==4:
            print(joint_seq[idx])
        for i in range(len(joint_seq[idx])):

            if i ==(len(joint_seq[idx])-1):
                break 
            if i == 0:
                pre_q, after_q = joint_seq[idx][i:i+2]
                new_first_q    = np.linspace(pre_q, after_q, int(freq*(desired_time)/4))
                new_q_arr      = new_first_q 
            else:
                pre_q, after_q = joint_seq[idx][i:i+2]
                print("preq", pre_q, "afterq", after_q)
                new_q          = np.linspace(pre_q, after_q, int(freq*(desired_time)/4))
                new_q_arr      = np.append(new_q_arr, new_q)

        print("q_shape",new_q_arr.shape)
        new_q_list.append(new_q_arr)
    np_q = np.array(new_q_list, dtype=object)
    np_q_trans =np_q.T 
    return np_q_trans

def euclidean_dist(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2) for i in range(len(point1))]))

def get_desired_time(start_pos, target_pos, desired_vel): 
    length = euclidean_dist(start_pos, target_pos)    
    desired_time = length/desired_vel
    print("Time", desired_time)
    return desired_time
