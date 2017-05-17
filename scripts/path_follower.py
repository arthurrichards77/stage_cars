#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('stage_cars')
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan2, sqrt
import yaml

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
global my_speed
my_speed = rospy.get_param("speed",0.0)
my_latacc = rospy.get_param("latacc",0.5*9.81)

def speed_callback(data):
  global my_speed
  my_speed = data.data

# global store of 
global min_range
min_range = 1000.0

def laser_callback(data):
  global min_range
  min_range = min(data.ranges)

# optional speed schedule - change speed at time points
global next_speed_sched_index
next_speed_sched_index = 0
start_time_secs = rospy.get_time()
speed_schedule = rospy.get_param("speed_schedule",[])

# grab path from parameter (smoothing done elsewhere)
my_path = Path()
my_path.header.frame_id='world'

# first try from single file
path_param = rospy.get_param("path",[])

# next try sequence of files
path_file_names = rospy.get_param("path_file_names",[])
for fn in path_file_names:
  with open(fn, 'r') as stream:
    try:
      path_xy = yaml.load(stream)
      path_param += path_xy
    except yaml.YAMLError as exc:
      print(exc)

# compile points into a ROS path
for pp in path_param:
    my_pose = PoseStamped()
    my_pose.pose.position.x = pp[0]
    my_pose.pose.position.y = pp[1]
    my_path.poses.append(my_pose)

# rolling window information
global last_index
last_index=0
num_points = len(my_path.poses)
window_len = int(0.25*num_points)

# callback for control
def ctrl_callback(data):
  global last_index
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
  # find the current window
  path_window = [kk % num_points for kk in range(last_index, last_index+window_len)]
  # find the closest point on the path
  ds = [sqrt((x-my_path.poses[kk].pose.position.x)*(x-my_path.poses[kk].pose.position.x)+(y-my_path.poses[kk].pose.position.y)*(y-my_path.poses[kk].pose.position.y)) for kk in path_window]
  dmin = min(ds)
  kmin = ds.index(dmin)
  point_index = path_window[kmin]
  pmin = my_path.poses[point_index]
  cx = pmin.pose.position.x
  cy = pmin.pose.position.y
  # store the index for next time
  last_index = point_index
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
  # check for speed schedule
  global my_speed
  global next_speed_sched_index
  if len(speed_schedule)>next_speed_sched_index:
    if rospy.get_time() >= start_time_secs + speed_schedule[next_speed_sched_index][0]:
      rospy.loginfo("Setting speed to %f at time offset %f" % (speed_schedule[next_speed_sched_index][1],speed_schedule[next_speed_sched_index][0]))
      my_speed = speed_schedule[next_speed_sched_index][1]
      next_speed_sched_index+=1
  # default speed from either schedule or topic
  cmd_speed = my_speed
  # speed limit for turns
  if cmd_speed*cmd_speed*abs(u)>my_latacc:
    cmd_speed = sqrt(my_latacc/abs(u))
  # check two second gap
  if cmd_speed*2.0>(min_range-1.0):
    cmd_speed = (min_range-1.0)/2.0
  # command
  v = Point()
  v.x = cmd_speed
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

# and external speed control
speed_sub = rospy.Subscriber('cmd_speed', Float64, speed_callback)

# and laser stopping
speed_sub = rospy.Subscriber('base_scan', LaserScan, laser_callback)

# local copy of current path for publishing
my_current_path = Path()
my_current_path.header.frame_id='world'

while not rospy.is_shutdown():
    current_window = [kk % num_points for kk in range(last_index, last_index+window_len)]
    my_current_path.poses = [my_path.poses[kk] for kk in current_window]
    pub_path.publish(my_current_path)
    my_rate.sleep()

