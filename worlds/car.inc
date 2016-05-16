# define a "car" macro for a basic vehicle
# size is about a LandRover
# made as two boxes so it steers about its rear axle

include "sick.inc"

define car position
(
  drive "diff"
  color "blue"
  size [2 1.8 2]
  velocity_bounds [-10 10 -10 10 -10 10 -90 90]
  model
  (
    color "white"  
    size [2 1.8 2]
    pose [2 0 -2 0]
  )
)

define car_cam car
(
  camera
  (
    pose [0 0 1.2 0.0] # irritatingly, can only shift camera upwards, not forward or sideways
    pantilt [0 10] # this will rotate camera view
    resolution [320 240]
    range [0.2 80]        
    color "red"
  )
)

define car_laser car
(
  sicklaser
  (
    pose [3.1 0 -1.2 0.0]
  )
)
