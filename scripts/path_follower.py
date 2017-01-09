#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sin, cos, atan2, sqrt
  
rospy.init_node('path_follower', anonymous=True)
pub_path = rospy.Publisher('path', Path, queue_size=1)
pub_pose = rospy.Publisher('pose', PoseStamped, queue_size=1)
pub_steer = rospy.Publisher('cmd_steer', Point, queue_size=1)
steering_gain = rospy.get_param("~steering_gain",0.3)
steering_lookahead = rospy.get_param("~steering_lookahead",4.0)
max_steer = rospy.get_param("~max_steer",0.3)
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
    my_path.poses.append(my_pose)

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
  x = x + steering_lookahead*cos(theta) 
  y = y + steering_lookahead*sin(theta) 
  # find the closest point on the path
  ds = [sqrt((x-p.pose.position.x)*(x-p.pose.position.x)+(y-p.pose.position.y)*(y-p.pose.position.y)) for p in my_path.poses]
  dmin = min(ds)
  pmin = my_path.poses[ds.index(dmin)]
  cx = pmin.pose.position.x
  cy = pmin.pose.position.y
  # project on to car's Y-axix (lateral)
  e = (cy-y)*cos(theta)-(cx-x)*sin(theta)
  # P control for constant radius
  #rospy.loginfo('Time is %f',rospy.get_rostime().to_sec())
  u = steering_gain*e
  # saturate
  if u>max_steer:
    u = max_steer
  elif u<-max_steer:
    u = -max_steer
  # command
  v = Point()
  v.x = my_speed
  v.y = u
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

