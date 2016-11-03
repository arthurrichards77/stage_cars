#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sin, cos, atan2, sqrt
import brl_drones.rospid
  
rospy.init_node('path_follower', anonymous=True)
pub_path = rospy.Publisher('path', Path, queue_size=1)
pub_pose = rospy.Publisher('pose', PoseStamped, queue_size=1)
pub_steer = rospy.Publisher('cmd_steer', Point, queue_size=1)
steering_pid = brl_drones.rospid.Rospid(0.15,0.0,0.22,'~steering')
my_rate = rospy.Rate(1)

# get speed from parameter
# won't move without it
my_speed = rospy.get_param("speed",0.0)

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

# callback for control
def ctrl_callback(data):
  # extract x and y position information
  x = data.pose.pose.position.x
  y = data.pose.pose.position.y
  # and quaternion info (just last two for heading)
  qz = data.pose.pose.orientation.z
  qw = data.pose.pose.orientation.w
  # calculate the heading
  theta = 2.0*atan2(qz,qw)
  # add just a little lookahead
  L = 4.0
  x = x + L*cos(theta) 
  y = y + L*sin(theta) 
  # find the closest point on the path
  ds = [sqrt((x-p.pose.position.x)*(x-p.pose.position.x)+(y-p.pose.position.y)*(y-p.pose.position.y)) for p in my_path.poses]
  dmin = min(ds)
  pmin = my_path.poses[ds.index(dmin)]
  cx = pmin.pose.position.x
  cy = pmin.pose.position.y
  # project on to car's Y-axix (lateral)
  e = (cy-y)*cos(theta)-(cx-x)*sin(theta)
  # PID control for constant radius
  #rospy.loginfo('Time is %f',rospy.get_rostime().to_sec())
  u = steering_pid.update(e, 0.0, rospy.get_rostime().to_sec())
  # saturate
  u = brl_drones.rospid.saturate(u,0.3)
  # command
  v = Point()
  v.x = my_speed
  v.y = -u
  # send it
  pub_steer.publish(v)
  # tell the world
  #rospy.loginfo('Got e = %f ctrl=%f', e, u)
  # publish pose info for viewing
  pose_out = PoseStamped()
  pose_out.header.frame_id = 'world'
  pose_out.pose = data.pose.pose
  pub_pose.publish(pose_out)

# start the feedback
pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, ctrl_callback)

while not rospy.is_shutdown():
    pub_path.publish(my_path)
    my_rate.sleep()

