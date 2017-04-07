#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def pointCallback(data):
  # set reference movement to zero
  cmd_vel_msg = Twist()
  # convert to wheel steer information
  cmd_vel_msg.linear.x = data.x
  cmd_vel_msg.angular.z = data.x*data.y
  # publish it
  pub_cmd_vel.publish(cmd_vel_msg)

# callback for control
def pose_callback(data):
  # extract x and y position information
  x = data.pose.pose.position.x
  y = data.pose.pose.position.y
  # package and broadcast
  p = Point()
  p.x = x
  p.y = y
  # send it
  pub_beacon.publish(p)
  # and broadbast all odoms on a common channel
  pub_odom_all.publish(data)

rospy.init_node('car_wrapper', anonymous=True)
sub_drive = rospy.Subscriber('cmd_steer', Point, pointCallback)
sub_pose = rospy.Subscriber('base_pose_ground_truth', Odometry, pose_callback)
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_beacon = rospy.Publisher('/beacon', Point, queue_size=1)
pub_odom_all = rospy.Publisher('/odom_all', Odometry, queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
