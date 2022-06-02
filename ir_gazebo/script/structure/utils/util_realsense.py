#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl
import pcl_helper
from pcl_helper import *
import tf
import utils.util_fk
from tf.transformations import rotation_matrix
import math

# list를 pcd data로 바꾸는 함수
def change_list_to_pcd(lista):
    cloud = pcl.PointCloud_PointXYZRGB()
    cloud.from_list(lista)
    return cloud

# object 여러개를 리스트로 나누기
def get_obj_point(cluster_indices,white_cloud):
    obj_points = []
    for j,indices in enumerate(cluster_indices):
        point_list = []
        for k,indice in enumerate(indices):
            point_list.append([
                white_cloud[indice][0],
                white_cloud[indice][1],
                white_cloud[indice][2],
                1.5
            ])
        obj_points.append(point_list)
    return obj_points

# pcd data를 받아 중심점을 return하는 함수
def get_middle_point(pcd):
    x_total = 0
    y_total = 0
    z_total = []
    pcd_numpy = pcd.to_array()
    for i in range(len(pcd_numpy)):
        x_total += pcd_numpy[i,0]
        y_total += pcd_numpy[i,1]
        z_total.append(pcd_numpy[i,2])
    x = x_total / len(pcd_numpy)
    y = y_total / len(pcd_numpy)
    z = (max(z_total)+(min(z_total)))/2
    return x,y,z

# 복셀화(Down sampling)
def do_voxel_grid_downssampling(pcl_data,leaf_size):
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return  vox.filter()

# 노이즈 제거
def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

# 바닥 제거 함수(random sample consensus 이용)
def do_ransac_plane_segmentation(pcl_data,pcl_sac_model_plane,pcl_sac_ransac,max_distance):
    '''
    Create the segmentation object
    :param pcl_data: point could data subscriber
    :param pcl_sac_model_plane: use to determine plane models, pcl.SACMODEL_PLANE
    :param pcl_sac_ransac: RANdom SAmple Consensus, pcl.SAC_RANSAC
    :param max_distance: Max distance for apoint to be considered fitting the model, 0.01
    :return: segmentation object
    '''
    seg = pcl_data.make_segmenter()
    seg.set_model_type(pcl_sac_model_plane)
    seg.set_method_type(pcl_sac_ransac)
    seg.set_distance_threshold(max_distance)

    # outliner 추출
    inliners, _ = seg.segment()
    inliner_object = pcl_data.extract(inliners,negative=False)
    outliner_object = pcl_data.extract(inliners,negative=True)
    return outliner_object

# clustering 함수
def euclid_cluster(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015) # 0.015
    ec.set_MinClusterSize(300) # 20
    ec.set_MaxClusterSize(3000) # 3000
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices, white_cloud

def cluster_mask(cluster_indices, white_cloud):
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                            white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float( cluster_color[j] )
                                           ])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud

# 관심 영역 설정
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# 좌표 변환 함수
def tf_matrix():
    listener = tf.TransformListener() 
    listener.waitForTransform('camera_link','camera_depth_optical_frame',rospy.Time(),rospy.Duration(2))
    (t,q) = listener.lookupTransform('camera_link','camera_depth_optical_frame', rospy.Time(0))
    t_matrix = tf.transformations.translation_matrix(t)
    r_matrix = tf.transformations.quaternion_matrix(q)
    return np.dot(t_matrix,r_matrix)

def change_frame(matt, points):
    transpose = points[:,0:3]
    ones = np.ones((len(points),1))
    transpose = np.concatenate((transpose,ones),axis=1)
    transpose = transpose.T
    transpose_after = np.dot(matt,transpose)
    transpose_after = transpose_after.T
    transpose_after_after = transpose_after[:,0:3]
    rgb = points[:,3].reshape(len(points),1)
    finalmat = np.concatenate((transpose_after_after,rgb),axis=1)
    return finalmat

def camera_to_base(matt, points):
    ones = np.ones((len(points),1))
    transpose = np.concatenate((points,ones),axis=1)
    transpose = transpose.T
    transpose_after = np.dot(matt,transpose)
    transpose_after = transpose_after.T
    transpose_after_after = transpose_after[:,0:3]
    return transpose_after_after

# numpy에서 pcd data로 바꾸는 함수
def numpy_to_pcd(nump):
    nump = nump.astype(np.float32)
    pcd = pcl.PointCloud_PointXYZI()
    pcd.from_array(nump)    
    return pcd

# callback 함수
def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    cloud = do_passthrough(cloud,'z',0,1.5)
    cloud = do_passthrough(cloud,'x',-0.4,0.4)
    cloud = do_voxel_grid_downssampling(cloud,0.005)
    delete_floor = do_ransac_plane_segmentation(cloud,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.043)
    delete_desk = do_ransac_plane_segmentation(delete_floor,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.043)
    delete_desk_1 = delete_desk.to_array()
    A = tf_matrix()
    delete_desk_points = change_frame(A,delete_desk_1)
    delete_desk_2 = numpy_to_pcd(delete_desk_points)
    cluster_indices, white_cloud = euclid_cluster(delete_desk_2)
    get_color_list.color_list = []
    obj_points = get_obj_point(cluster_indices,white_cloud)
    middle_point_lists = []
    for i in range(len(obj_points)):
        obj_group_cloud = change_list_to_pcd(obj_points[i])
        x,y,z = get_middle_point(obj_group_cloud)
        middle_point = [x,y,z]
        middle_point_lists.append(middle_point)

    """ 
    NEED CAMERA CALIBRATION
    """
    rotation_mt_y = util_fk.Rotation_Y(0.73)
    position_mt= util_fk.Translation(0.115, 0 ,1.48)
    transform_mt = util_fk.HT_matrix(rotation_mt_y,position_mt)
    middle_point_array = np.array(middle_point_lists)
    change = camera_to_base(transform_mt,middle_point_array)
    middle_points = list(change)
    return middle_points
    
if __name__ == "__main__":
    rospy.init_node('pointcloud', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    # pub = rospy.Publisher("/camera/depth/color/points_new",PointCloud2,queue_size=1)
    rospy.spin()