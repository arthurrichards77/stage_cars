#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from math import sin, cos, atan2, sqrt
  
rospy.init_node('path_follower', anonymous=True)
pub_path = rospy.Publisher('path', Path, queue_size=1)
my_rate = rospy.Rate(1)

# grab coarse path from parameter
path_param = rospy.get_param("path",[])
my_path = Path()
my_path.header.frame_id='world'
for pp in path_param:
    my_pose = PoseStamped()
    my_pose.pose.position.x = pp[0]
    my_pose.pose.position.y = pp[1]
    my_pose.pose.orientation.z = sin(pp[2]*0.5)
    my_pose.pose.orientation.w = cos(pp[2]*0.5)
    my_path.poses += [my_pose]

# smoothing
if len(my_path.poses)>1:
    # setup fixed parameters
    N = 100
    ts = [1.0*r/N for r in range(N)]
    t0 = [(1-t)*(1-t)*(1-t) for t in ts]
    t1 = [3.0*(1-t)*(1-t)*t for t in ts]
    t2 = [3.0*t*t*(1-t) for t in ts]
    t3 = [t*t*t for t in ts]
    # new path
    smoothed_path = Path()
    smoothed_path.header.frame_id='world'
    # smoothing process
    for ii in range(1,len(my_path.poses)):
        theta0 = 2.0*atan2(my_path.poses[ii-1].pose.orientation.z,my_path.poses[ii-1].pose.orientation.w)
        theta3 = 2.0*atan2(my_path.poses[ii].pose.orientation.z,my_path.poses[ii].pose.orientation.w)
        x0 = my_path.poses[ii-1].pose.position.x
        x3 = my_path.poses[ii].pose.position.x
        y0 = my_path.poses[ii-1].pose.position.y
        y3 = my_path.poses[ii].pose.position.y
        L = sqrt((x3-x0)*(x3-x0) + (y3-y0)*(y3-y0))
        x1 = x0 + (L/3.0)*cos(theta0)
        y1 = y0 + (L/3.0)*sin(theta0)
        x2 = x3 - (L/3.0)*cos(theta3)
        y2 = y3 - (L/3.0)*sin(theta3)
        for jj in range(N):
            my_pose = PoseStamped()
            my_pose.pose.position.x = x0*t0[jj] + x1*t1[jj] + x2*t2[jj] + x3*t3[jj]
            my_pose.pose.position.y = y0*t0[jj] + y1*t1[jj] + y2*t2[jj] + y3*t3[jj]
            smoothed_path.poses += [my_pose]
    # replace coarse path
    my_path = smoothed_path               

while not rospy.is_shutdown():
    pub_path.publish(my_path)
    my_rate.sleep()

