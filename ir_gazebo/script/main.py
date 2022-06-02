import rospy 
import numpy as np 
from structure.class_ur_real import UR_REAL 
from structure.class_realsense435 import Realsense435

def main():
    STARTPOS      = np.array([0.6, 0, 0.85])
    realsense     = Realsense435()
    real_robot    = UR_REAL()
    center_points = realsense.callback(realsense.point_cloud)
    center_point  = center_points[0]
    x = center_point[0]
    y = center_point[1]
    z = center_point[2]
    real_robot.grasp_single_obj(start_pos=STARTPOS, target_pos=np.array([x, y, z]), num_interpol=5, desired_vel=0.2)


if __name__=="__main__": 
    rospy.init_node("REAL_WORLD")
    main()