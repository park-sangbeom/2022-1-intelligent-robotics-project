import sys
import rospy
import moveit_commander
import tf
import numpy
import copy
import time
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

print("Packages Loaded.")

moveit_commander.roscpp_initialize(sys.argv)
robot         = moveit_commander.RobotCommander()
group_name=robot.get_group_names()
print("Moveit control group name:%s"%group_name)

arm_group      = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

arm_group.set_named_target("up")
arm_group.go(wait=True)
print("Exeucute Up pose.")

gripper_group.set_named_target("open")
gripper_group.go(wait=True)
print("Exeucute Open pose.")