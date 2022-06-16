from structure.class_robot import ROBOT
import rospy 
import numpy as np 
from structure.class_ur_real import RealUR 
from structure.class_realsense435 import Realsense435
from structure.utils.util_fk import *
from structure.utils.util_realsense import * 
from structure.utils.util_ik import get_direction_offset
import os, subprocess
import time
from operator import itemgetter 
import math 
import numpy as np 

def main():
    subprocess.run(["cd ~/UnseenObjectClustering; ./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh $GPU_ID 0"], shell=True)
    obj_array = np.load('/home/rilab/UnseenObjectClustering/obj_array.npy')

    # """
    # Need Calibration
    # """
    
    rotation_mt_z = Rotation_Z(-math.pi / 2)
    rotation_mt_x = Rotation_X(-math.pi *(28/36))
    rotation_mt = np.dot(rotation_mt_z, rotation_mt_x)
    position_mt= Translation(0.4, 0.03 ,1.45)
    transform_mt = HT_matrix(rotation_mt, position_mt)
    obj_lists = list(camera_to_base(transform_mt, obj_array))

    """
    Group 1 
    """
    # def get_direction_offset(x,y):
    #     # x <-> y direction should be changed in World Coordinate
    #     base_offset = 0.18
    #     tcp_offset = 0.1135
    #     offset_angle = math.atan2(y-tcp_offset, x-base_offset)
    #     if offset_angle>1.0:
    #         print("[Error] Unavailable joints")
    #         return None 
    #     return offset_angle

    # def pick_order(obj_lst):
    #     planner_list = []
    #     final_plan_list=[]

    #     for idx, obj in enumerate(obj_lst): 
    #         obj_x, obj_y = obj[0], obj[1]
    #         obj_theta = get_direction_offset(obj_x, obj_y)
    #         obj_dict = {"x":obj_x, "y":obj_y, "theta": obj_theta*math.pi, "temp_idx": idx}
    #         planner_list.append(obj_dict)
    #     plan = sorted(planner_list, key=itemgetter("y"))
    #     plan.reverse()
    #     for idx, i_plan in enumerate(plan):
    #         i_plan["temp_idx"] =idx 
    #     for idx in range(len(plan)):
    #         if idx == len(plan)-1: 
    #             break 
    #         if plan[idx]["x"] - plan[idx+1]["x"]>0: 
    #             print("Check", plan[idx]["x"], plan[idx+1]["x"])
    #             if abs(plan[idx]["theta"]-plan[idx+1]["theta"]) <0.5: 
    #                 print("Double check")
    #                 plan[idx+1]["temp_idx"] =idx 
    #                 plan[idx]["temp_idx"] = idx+1
    #     final_plan = sorted(plan, key=itemgetter("temp_idx"))
    #     for plan in final_plan: 
    #         final_plan_list.append([plan["x"], plan["y"], 0.83])
    #     return final_plan_list
    """
    Group 2
    """
    # def pick_order(center_points):
    #     num_obj = len(center_points)
    #     candidate_queue = center_points
    #     compare_queue = []
    #     finished_queue = []
    #     dist_ur = []
    #     left_order = np.argsort(np.asarray(center_points)[:,1])[::-1]
    #     if num_obj ==1 :
    #         finished_queue = center_points
    #         return finished_queue
    
    #     for i in range(num_obj):
    #         compare_queue.append(candidate_queue[left_order[i]])
    #         if len(compare_queue) == 2:
    #             dist_2obj = np.linalg.norm(np.asarray(compare_queue)[:,:2],axis=1)
    #             if np.abs(compare_queue[0][0] - compare_queue[1][0]) > 0.2:
    #                 nearest_obj = compare_queue.pop(0)
    #             else :
    #                 nearest_obj = compare_queue.pop(np.argsort(dist_2obj)[0])
    #             finished_queue.append(nearest_obj)
    #             if len(finished_queue) == num_obj-1:
    #                 finished_queue.append(compare_queue.pop())
    #                 break

    #     return finished_queue
    
    """
    Group 3
    """
    # def pick_order(centerpoints,initpos = [0.43,0.53], alpha=2):
    #     centerpoints = np.array(centerpoints)
    #     centerpointsX = centerpoints[:,0]
    #     centerpointsY = centerpoints[:,1]
        
    #     def swap4vis(pointsX,pointsY):
    #         visualX = -pointsY
    #         visualY = pointsX
    #         return visualX,visualY

    #     visualX,visualY = swap4vis(centerpointsX,centerpointsY)
    #     initVisX,initVisY = swap4vis(initpos[0],initpos[1])
    #     relativeX = visualX - initVisX
    #     relativeY = visualY - initVisY

    #     score = relativeX* alpha + relativeY
    #     order =  np.argsort(score)
    #     centerpoints = list(centerpoints[order])
    #     return centerpoints

    """
    Group 4
    """
    # def pick_order(obj_lists):
    #     sorted_center_points = obj_lists
    #     # (Gazebo 기준) x축으로부터 가까운 point를 앞 순위로 정렬
    #     for i in range(len(sorted_center_points)):
    #         temp_min_idx = i
    #         for i_ in range (i,len(sorted_center_points)):
    #             if sorted_center_points[temp_min_idx][0] > sorted_center_points[i_][0]:
    #                 temp_min_idx = i_
    #         temp = sorted_center_points[i]
    #         sorted_center_points[i] = sorted_center_points[temp_min_idx]
    #         sorted_center_points[temp_min_idx] = temp
    #         # (Gazebo 기준) x축으로 safety offset 범위 안에 있고,
    #         # (Gazebo 기준) y축이 더 큰 point는 앞 순위로 정렬
    #     safety_offset = 0.15
    #     for j in range(len(sorted_center_points)):
    #         temp_idx = j
    #         for j_ in range (j,len(sorted_center_points)):
    #             if sorted_center_points[temp_idx][0] + safety_offset > sorted_center_points[j_][0] and sorted_center_points[temp_idx][1] < sorted_center_points[j_][1]:
    #                 temp_idx = j_
    #         if temp_idx != j:
    #             temp = sorted_center_points[temp_idx]
    #             del sorted_center_points[temp_idx]
    #             sorted_center_points.insert(j, temp)
    #     return sorted_center_points

    """
    Group 5
    """
    def pick_order(middle_point_lists):
        score = np.array([])
        for i, p in enumerate(middle_point_lists):
            score = np.append(score, p[0] - 100 * p[1])
        new_list = []
        for idx in score.argsort():
            new_list.append(middle_point_lists[idx])
        return new_list

    obj_lists = pick_order(obj_lists)

    print("{} objects".format(len(obj_lists)))
    print(obj_lists)

    # realsense     = Realsense435()
    real_robot    = RealUR()
    print("Please make sure that your robot can move freely between these poses before proceeding!")
    inp = input("Continue? y/n: ")[0]
    if inp == "y":
        start = time.time()
        for i in range(len(obj_lists)):
            center_point  = obj_lists[i]
            x = center_point[0]
            y = center_point[1]
            z = 0.83
            # x=0.9
            # y=-0.2
            # z=0.84
            offset_angle = get_direction_offset(x, y)
            real_robot.grasp_single_obj(target_pos=[x, y, z], num_interpol=1, desired_vel=0.1, direction_offset=offset_angle)
        end = time.time()
        print("Time: {}".format(end-start))    
    else:
        print("TOMBOY")

if __name__=="__main__": 
    main()
