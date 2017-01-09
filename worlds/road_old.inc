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

define road_turnout road_straight
(
  # side road
  road
  (
    size [6 7 0.01]
    pose [0 6.5 0 0]
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
