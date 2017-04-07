# stage_cars
Stage worlds for car simulation in ROS.

Worlds folder includes stage world files for road tiles and cars.  Car options include basic, with camera, or with a simple laser range sensor.

Node car_wrapper.py converts a Point message, interpreted as x=speed, y=steering curvature, to the Twist message used by Stage.  Also publishes all odometry to a common topic intended to mimic a beacon capability.

Node path_smoother.py takes coarse coordinates corresponding to waypoints and applies Bezier smoothing.  See smooth_path.launch for examples.

Node path_follower.py provides a simple path-following capability, following a sequence of the path files generated using the smoother.  It will repeat once finished.  Also takes external speed control, slows for maximum lateral acceleration, and will attempt to keep a two second gap ahead if laser sensor present.  See three_cars.launch for examples.
