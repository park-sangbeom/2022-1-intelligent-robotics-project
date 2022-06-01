import rospy 
from sensor_msgs.msg import PointCloud2, Image
import message_filters
import time 
from pcl_helper import *

class Realsense435():
    def __init__(self):    
        # Create member variables 
        self.tick = 0
        self.point_cloud = None 
        self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)
        tic_temp = 0
        while self.tick<2:
            time.sleep(1e-3)
            tic_temp = tic_temp + 1
            if tic_temp > 5000:
                print ("[ERROR] CHECK REALSENSE435")
                break

    def callback(self, point_cloud_msg):
        self.tick = self.tick+1
        self.point_cloud = point_cloud_msg 