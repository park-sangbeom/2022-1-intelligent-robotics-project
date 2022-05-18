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
    inliners, a = seg.segment()
    inliner_object = pcl_data.extract(inliners,negative=False)
    outliner_object = pcl_data.extract(inliners,negative=True)
    return outliner_object

# clustering 함수
def euclid_cluster(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(3000)
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

# 원하는 object의 data만 추출하는 함수
def choose_obj_group(cluster_indices,white_cloud):
    size = white_cloud.size
    obj = max(white_cloud)
    for i in range(size):
        if white_cloud[i] == obj:
            obj_index = i
    for j,indices in enumerate(cluster_indices):
            for k,indice in enumerate(indices):
                if indice == obj_index:
                    cluster_index = j
    obj_cluster = cluster_indices[cluster_index]
    obj_point_list = []
    for i,indice in enumerate(obj_cluster):
        obj_point_list.append([
                    white_cloud[indice][0],
                    white_cloud[indice][1],
                    white_cloud[indice][2],
                    1.5
                ])
    obj_cloud = pcl.PointCloud_PointXYZRGB()
    obj_cloud.from_list(obj_point_list)
    return obj_cloud

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
    listener.waitForTransform('world','camera_depth_optical_frame',rospy.Time(),rospy.Duration(2))
    (t,q) = listener.lookupTransform('world','camera_depth_optical_frame', rospy.Time(0))
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

# numpy에서 pcd data로 바꾸는 함수
def numpy_to_pcd(nump):
    nump = nump.astype(np.float32)
    pcd = pcl.PointCloud_PointXYZI()
    pcd.from_array(nump)
    return pcd
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    segmenter.set_max_iterations(100)
    segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, coefficients = segmenter.segment()

    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

    print('Model inliers: ' + str(len(indices)))
    for i in range(0, 5):#range(0, len(indices)):
        print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
              str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliers, outliers

# callback 함수
def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    cloud = do_voxel_grid_downssampling(cloud,0.005)
    delete_floor = do_ransac_plane_segmentation(cloud,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.01)
    print(delete_floor)
    delete_desk = do_ransac_plane_segmentation(delete_floor,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.01)
    print(delete_floor)
    delete_desk_1 = delete_desk.to_array()
    delete_desk_1 = cloud

    A = tf_matrix()
    delete_desk_points = change_frame(A,delete_desk_1)
    delete_desk_2 = numpy_to_pcd(delete_desk_points)
    cluster_indices, white_cloud = euclid_cluster(delete_desk_2)

    # cluster된 물체들 전부
    get_color_list.color_list = []
    final = cluster_mask(cluster_indices,white_cloud)
    final = do_passthrough(final,'z',0.75,1.2)
    final_new = pcl_helper.pcl_to_ros(final)
    # 원하는 object만 추출
    obj_cloud = choose_obj_group(cluster_indices,white_cloud)
    # pcl.save(obj_cloud,"obj_cloud2.pcd")
    obj_new = pcl_helper.pcl_to_ros(obj_cloud)
    pub.publish(final_new)
    print("dd")


if __name__ == "__main__":
    rospy.init_node('pointcloud', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    pub = rospy.Publisher("/camera/depth/color/points_new",PointCloud2,queue_size=1)
    rospy.spin()
