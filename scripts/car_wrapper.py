#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point

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
  
rospy.init_node('car_wrapper', anonymous=True)
sub_drive = rospy.Subscriber('cmd_steer', Point, pointCallback)
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
