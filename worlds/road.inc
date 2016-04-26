# define "road" macro
# sets collisions off
# and grey colour

define road_tile model
(
  size [20 20 0.01]
  color "green"
  obstacle_return 0
)

define road model
(
  color "gray"
  obstacle_return 0
)

# this macro draws a 20m straight length of road

define road_straight road_tile
(
  road
  (
    size [20 6 0.01]
  )
)

# this one does just a turnout

define road_turnout road
(
  # side road
  size [20 20 0.01]
  block
  (
  points 4
  point[0] [0.35 1]
  point[1] [0.35 0.35]
  point[2] [0.65 0.35]
  point[3] [0.65 1]
  z [0 0.9]
  )
  # chamfer at the turn
  block
  (
  points 4
  point[0] [0.3 0.3]
  point[1] [0.35 0.35]
  point[2] [0.65 0.35]
  point[3] [0.7 0.3]
  z [0 0.9]
  )
  # main road
  block
  (
  points 4
  point[0] [0 0]
  point[1] [1 0]
  point[2] [1 0.3]
  point[3] [0 0.3]
  z [0 0.9]
  )
  # main road centre line
  block
  (
  points 4
  point[0] [0 0.14]
  point[1] [1 0.14]
  point[2] [1 0.16]
  point[3] [0 0.16]
  z [0 1.0]
  color "white"
  )
  # side road centre line
  block
  (
  points 4
  point[0] [0.49 0.3]
  point[1] [0.51 0.3]
  point[2] [0.51 1]
  point[3] [0.49 1]
  z [0 1.0]
  color "white"
  )
)

define road_bend road_tile
(
  road
  (
    size [13 13 0.01]
    pose [-3.5 3.5 0 270]
    block
    (
      points 4
      point[0] [0 0.538]
      point[1] [0 1]
      point[2] [1 0]
      point[3] [0.538 0]
      z [0 1.0]
    )
  )
)
