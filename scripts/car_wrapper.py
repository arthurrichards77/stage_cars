#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import tf

rospy.init_node('car_wrapper', anonymous=True)
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_beacon = rospy.Publisher('/beacon', Point, queue_size=1)
pub_odom_all = rospy.Publisher('/odom_all', Odometry, queue_size=1)
pub_pose = rospy.Publisher('pose', PoseStamped, queue_size=1)
pub_vel = rospy.Publisher('velocity', TwistStamped, queue_size=1)
tf_broadcast = tf.TransformBroadcaster()

frame_name = rospy.resolve_name('base_footprint')

def pointCallback(data):
  # set reference movement to zero
  cmd_vel_msg = Twist()
  # convert to wheel steer information
  cmd_vel_msg.linear.x = data.x
  cmd_vel_msg.angular.z = data.x*data.y
  # publish it
  pub_cmd_vel.publish(cmd_vel_msg)

# callback for position
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
  # publish pose info for viewing
  pose_out = PoseStamped()
  pose_out.header = data.header
  pose_out.header.frame_id = '/world'
  pose_out.pose = data.pose.pose
  pub_pose.publish(pose_out)
  # and twist stamped for DMS interface
  vel_out = TwistStamped()
  vel_out.header = data.header
  vel_out.header.frame_id = '/world'
  vel_out.twist = data.twist.twist
  pub_vel.publish(vel_out)
  # transform
  tf_broadcast.sendTransform((x, y, 0.0),
                     (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                     rospy.Time.now(),frame_name,"/world")

sub_drive = rospy.Subscriber('cmd_steer', Point, pointCallback)
sub_pose = rospy.Subscriber('base_pose_ground_truth', Odometry, pose_callback)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
