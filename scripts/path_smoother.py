#!/usr/bin/env python
import rospy
import roslib
import os
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import sin, cos, atan2, sqrt
import yaml  

rospy.init_node('path_smoother', anonymous=True)

# grab coarse path from parameter
path_param = rospy.get_param("waypoints",[])
my_path = Path()
my_path.header.frame_id='world'
for pp in path_param:
    my_pose = PoseStamped()
    my_pose.pose.position.x = pp[0]
    my_pose.pose.position.y = pp[1]
    my_pose.pose.orientation.z = sin(pp[2]*0.5)
    my_pose.pose.orientation.w = cos(pp[2]*0.5)
    my_path.poses += [my_pose]

# new path
smoothed_path = Path()
smoothed_path.header.frame_id='world'

# smoothing
if len(my_path.poses)>1:
    # setup fixed parameters
    N = 100
    ts = [1.0*r/N for r in range(N)]
    t0 = [(1-t)*(1-t)*(1-t) for t in ts]
    t1 = [3.0*(1-t)*(1-t)*t for t in ts]
    t2 = [3.0*t*t*(1-t) for t in ts]
    t3 = [t*t*t for t in ts]
    k = 2.5
    # smoothing process
    for ii in range(1,len(my_path.poses)):
        theta0 = 2.0*atan2(my_path.poses[ii-1].pose.orientation.z,my_path.poses[ii-1].pose.orientation.w)
        theta3 = 2.0*atan2(my_path.poses[ii].pose.orientation.z,my_path.poses[ii].pose.orientation.w)
        x0 = my_path.poses[ii-1].pose.position.x
        x3 = my_path.poses[ii].pose.position.x
        y0 = my_path.poses[ii-1].pose.position.y
        y3 = my_path.poses[ii].pose.position.y
        L = sqrt((x3-x0)*(x3-x0) + (y3-y0)*(y3-y0))
        x1 = x0 + (L/k)*cos(theta0)
        y1 = y0 + (L/k)*sin(theta0)
        x2 = x3 - (L/k)*cos(theta3)
        y2 = y3 - (L/k)*sin(theta3)
        for jj in range(N):
            my_pose = PoseStamped()
            my_pose.pose.position.x = x0*t0[jj] + x1*t1[jj] + x2*t2[jj] + x3*t3[jj]
            my_pose.pose.position.y = y0*t0[jj] + y1*t1[jj] + y2*t2[jj] + y3*t3[jj]
            smoothed_path.poses += [my_pose]

# just need the 2D position data for the file
simple_path = [[p.pose.position.x, p.pose.position.y] for p in smoothed_path.poses]

# get file name for YAML output
file_name = rospy.get_param('output_filename','path.yaml')

with open(file_name, 'w') as outfile:
    yaml.dump(simple_path, outfile, default_flow_style=True)

print "Working directory %s" % os.getcwd()
print "Saved path to file %s" % file_name
