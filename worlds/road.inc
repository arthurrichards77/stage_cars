# define "road" macro
# sets collisions off
# and grey colour

define road model
(
  color "gray"
  obstacle_return 0
)

# this macro draws a 20m straight length of road

define road_straight road
(
  size [20 6 0.01]
  # add a centre line  
  model (
    size [20 0.4 0.02]
    color "white"
    obstacle_return 0
  )
)

# this macro draws a 90deg bend on a 20m x 20m tile
# its ugly for now, just using one intermediate point
# could be tidied up later with a little trig

define road_bend road
(
  size [20 20 0.01]
  # done as two blocks as Stage likes them convex
  block
  (
  points 4
  point[0] [0 0]
  point[1] [0.7 0.3]
  point[2] [0.5 0.5]
  point[3] [0 0.3]
  z [0 0.9] # slightly less then full height so centreline shows up
  ) 
  block
  (
  points 4
  point[0] [0.7 0.3]
  point[1] [1 1]
  point[2] [0.7 1]
  point[3] [0.5 0.5]
  z [0 0.9]
  )
  # and the centreline
  block
  (
  points 4
  point[0] [0 0.14]
  point[1] [0.61 0.39]
  point[2] [0.59 0.41]
  point[3] [0 0.16]
  z [0 1]
  color "white"
  )
  block
  (
  points 4
  point[0] [0.86 1]
  point[1] [0.61 0.39]
  point[2] [0.59 0.41]
  point[3] [0.84 1]
  z [0 1]
  color "white"
  )
)
