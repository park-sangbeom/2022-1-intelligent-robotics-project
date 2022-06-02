from structure.class_robot import ROBOT
import rospy 
import numpy as np 
from structure.class_ur_real import RealUR 
from structure.class_realsense435 import Realsense435
from structure.utils.util_ik import get_direction_offset

def main():
    realsense     = Realsense435()
    real_robot    = RealUR()
    center_points = realsense.callback(realsense.point_cloud)
    center_point  = center_points[0]
    x = center_point[0]
    y = center_point[1]
    z = center_point[2]
    offset_angle = get_direction_offset(x, y)
    real_robot.grasp_single_obj(target_pos=[x, y, z], num_interpol=5, desired_vel=0.2, direction_offset=offset_angle)


if __name__=="__main__": 
    main()
