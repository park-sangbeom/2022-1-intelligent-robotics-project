import math 
import numpy as np

def make_ik_input(target_name=["base_joint"], 
                  target_pos=[[0, 0, 0]], 
                  target_rot=[[0, 0, 0]], 
                  solve_pos=[1], 
                  solve_rot=[1], 
                  weight_pos=1, 
                  weight_rot=0, 
                  disabled_joi_id=[], 
                  joi_ctrl_num=7):
    return {"target_joint_name":target_name, 
            "target_joint_position":target_pos,
            "target_joint_rotation":target_rot,
            "solve_position":solve_pos,
            "solve_rotation":solve_rot,
            "position_weight":weight_pos,
            "rotation_weight":weight_rot,
            "disabled_joi_id":disabled_joi_id, 
            "joint_ctrl_num":joi_ctrl_num}

def get_aug_ik_ingredients(robot_jc, variables, _wn_pos=1/0.3, _wn_ang=1/(2*np.math.pi)):
    # Set variables
    joint_name_trgt   = variables['target_joint_name']
    p_trgt_goal       = variables["target_joint_position"]
    r_trgt            = variables["target_joint_rotation"]
    IK_P              = variables["solve_position"]
    IK_R              = variables["solve_rotation"]
    p_err_weight      = variables["position_weight"]
    R_err_weight      = variables["rotation_weight"]
    disable_joi       = variables["disabled_joi_id"]
    joint_length_ctrl = variables["joint_ctrl_num"]
    final_joint_id    = joint_length_ctrl+1 # Including base joint
    p_trgt_curr       = []
    R_trgt_curr       = []
    id_trgt_curr      = []
    mother_trgt_curr  = []
    child_trgt_curr   = []

    # Weight Position, Orientation
    wn_pos = _wn_pos
    wn_ang = _wn_ang
    wn_pre = []
    for choose_p in IK_P:
        if choose_p:
            for i in range(3):
                wn_pre.append(wn_pos)
        else: 
            pass 
    for choose_ang in IK_R:
        if choose_ang:
            for i in range(3):
                wn_pre.append(wn_ang)
        else: 
            pass 
    we = np.diag(wn_pre)
    # Get IK Error 
    J_use = np.array([])
    ik_err = np.array([]) 

    # Index From Root 
    idx_fr_root = find_route(robot_jc, final_joint_id)
    joint_idxs_use = list(set(idx_fr_root) - set(disable_joi))
    # Get Jacobian 
    J = aug_jacobian(robot_jc, joint_idxs_use)
    # Set Current Target Joint 
    for jc in robot_jc: 
        for joint_name in joint_name_trgt:
            if jc.name == joint_name: 
                p_trgt_curr.append(jc.p)
                R_trgt_curr.append(jc.R)
                id_trgt_curr.append(jc.id)
                mother_trgt_curr.append(jc.mother) 
                child_trgt_curr.append(jc.child) 

    for idx in range(len(joint_name_trgt)):
        curr_joint  = [p_trgt_curr[idx], R_trgt_curr[idx]]
        R_trgt_goal = rpy2r(r_trgt[idx])
        trgt_joint  = [column_v(p_trgt_goal[idx][0], p_trgt_goal[idx][1], p_trgt_goal[idx][2]), R_trgt_goal]
        err         = Cal_VWerr(trgt_joint, curr_joint)
        
        if IK_P[idx]: 
            if len(J_use)==0:
                J_use = J[0:3]
            else:
                J_use = np.concatenate((J_use, J[0:3]), axis=0) 
            if len(ik_err) == 0:
                ik_err = p_err_weight*err[0:3]
            else:
                ik_err = np.concatenate((ik_err, p_err_weight*err[0:3]), axis=0) 

        if IK_R[idx]:
            if len(J_use)==0:
                J_use = J[3:6]
            else:
                J_use = np.concatenate((J_use, J[3:6]), axis=0)

            if len(ik_err)==0:
                ik_err = R_err_weight*err[3:6]
            else:
                ik_err = np.concatenate((ik_err, R_err_weight*err[3:6]), axis=0) 

    return J_use, ik_err, we

def column_v(x,y,z):
    return np.array([[x, y, z]]).T

def column(input):
    arr = np.array([])
    for i in input.split(): 
        i = float(i)
        arr = np.append(arr, np.array([i]), axis=0)
    arr = np.array([arr])
    return arr.T

def column_size(matrix):
    m = len(matrix[0])
    return m

def trim_scale(in_val, th):
    max_abs_val = np.max(abs(in_val.flatten()))
    if max_abs_val > th:
        out_val = in_val/max_abs_val * th 
    else:
        out_val = in_val 

    return out_val

def aug_jacobian(robot_jc, idx):
    jsize = len(idx)
    target_jc = robot_jc[idx[-1]-1].p
    J = np.zeros((6, jsize))
    for i in range(jsize):
        j = idx[i]
        mom = robot_jc[j-1].mother
        a = np.matmul(robot_jc[mom].R, robot_jc[j-1].a)
        J_i = np.concatenate((np.cross(a.T, (target_jc - robot_jc[j-1].p).T).T, a))
        J[:, i]=J_i.squeeze()
    if len(idx)<6:
        for i in range(6-len(idx)):
            add_j = np.zeros((6,1))
            J=np.concatenate((J, add_j), axis=1)
    return J  

def damped_ls(J_use, ik_err):
    lambda_rate = 0.01
    lambda_min = 1e-6
    lambda_max = 1e-0
    step_size = 0.1
    dq_th = 40*np.math.pi/180
    ik_err_avg = np.mean(abs(ik_err))
    # Damping Term 
    lambda_ = lambda_rate * ik_err_avg + lambda_min 
    n_ctrl = column_size(J_use)
    # Lamda Scheduling 
    J_culumn_sum = abs(np.sum(J_use, axis =0))
    #print(J_culumn_sum.shape)
    for j in range(len(J_culumn_sum)):
        for i in J_culumn_sum:
            idx_nz = j
            J_use_nz = J_use[:,idx_nz].reshape(1, -1)
            det_J = np.linalg.det(np.matmul(J_use_nz, J_use_nz.T))
            if i >0.1:
                if det_J > 1e-3:
                    lambda_=1e-4
                elif det_J < 1e-20:
                    lambda_ = lambda_max
    J_n_ctrl = np.matmul(J_use.T, J_use) + lambda_* np.eye(n_ctrl, n_ctrl)
    dq_raw = np.matmul(np.linalg.solve(J_n_ctrl, J_use.T), ik_err)
    dq = trim_scale(step_size * dq_raw, dq_th)
    return dq, det_J

def move_joints(robot_jc, idx, dq):
    for i in range(len(idx)):
        id = idx[i]
        list_idx = id-1
        robot_jc[list_idx].q = robot_jc[list_idx].q + dq[i]

def add_joints(robot_jc, ids, joints): 
    for joint, id in zip(joints, ids): 
        list_idx = id-1 
        robot_jc[list_idx].q = joint

def get_jc_by_id(robot_jc, id):
    for jc in robot_jc:
        if id == jc.id:
            return jc 
        elif id == 0:
            return 
    
def find_mother(ulink, node_id):
    if node_id != 0:
        if node_id == 1:
            ulink[node_id]['mother']=0
        if ulink[node_id]['child']:
            for child_id in ulink[node_id]['child']:
                if child_id == 0:
                    continue 
                ulink[child_id]['mother']=node_id
                find_mother(ulink, child_id)
    
    return ulink

def rpy2r(rpy):
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    Cphi = np.math.cos(roll)
    Sphi = np.math.sin(roll)
    Cthe = np.math.cos(pitch)
    Sthe = np.math.sin(pitch)
    Cpsi = np.math.cos(yaw)
    Spsi = np.math.sin(yaw)

    rot = np.array([
        [Cpsi * Cthe, -Spsi * Cphi + Cpsi * Sthe * Sphi, Spsi * Sphi + Cpsi * Sthe * Cphi],
        [Spsi * Cthe, Cpsi * Cphi + Spsi * Sthe * Sphi, -Cpsi * Sphi + Spsi * Sthe * Cphi],
        [-Sthe, Cthe * Sphi, Cthe * Cphi]
    ])
    assert rot.shape == (3, 3)
    return rot

def r2w(R):
    el = np.array([
            [R[2,1] - R[1,2]],
            [R[0,2] - R[2,0]], 
            [R[1,0] - R[0,1]]
        ])
    norm_el = np.linalg.norm(el)
    if norm_el > 1e-10:
        w = np.arctan2(norm_el, np.trace(R)-1) / norm_el * el
    elif R[0,0] > 0 and R[1,1] > 0 and R[2,2] > 0:
        w = np.array([[0, 0, 0]]).T
    else:
        w = np.math.pi/2 * np.array([[R[0,0]+1], [R[1,1]+1], [R[2,2]+1]])
    return w

def Cal_VWerr(cref, cnow):
    perr = (cref[0] - cnow[0]).astype(float)
    Rerr = np.matmul(np.linalg.inv(cnow[1].astype(float)), cref[1].astype(float))
    werr = np.matmul(cnow[1].astype(float), r2w(Rerr))
    err =  np.concatenate([perr, werr], axis=0)
    assert err.shape == (6, 1)
    return err

def rodrigues(a, q):
    norm_a=np.linalg.norm(a)
    if norm_a <1e-10:
        R = np.eye(3)
    else:
        a = a/norm_a
        th = norm_a*q
        a_hat = np.array([[0,  -a[2], a[1]],
                          [a[2],  0, -a[0]],
                          [-a[1], a[0], 0]], dtype=object)
        R = np.eye(3) + a_hat * np.sin(th) + np.linalg.matrix_power(a_hat, 2) * (1-np.cos(th))
    return R 

def find_route(robot_jc, link_id):
    ulink = get_jc_by_id(robot_jc, link_id)
    mother_id=ulink.mother
    if mother_id ==0:
        return
    elif mother_id == 1:
        idx = [link_id]
        return idx
    else: 
        idx = np.append(find_route(robot_jc, mother_id), [link_id])
        return idx  
    
def decompose_rotation_matrix(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], (math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])

def get_direction_offset(x,y):
    # x <-> y direction should be changed in World Coordinate
    base_offset = 0.18
    tcp_offset = 0.1135
    offset_angle = math.atan2(y-tcp_offset, x-base_offset)
    if offset_angle>1.0:
        print("[Error] Unavailable joints")
        return None 
    return offset_angle

def get_curr_wrist_pos(robot_jc):
    return robot_jc[9].p.reshape(-1,)