define kinect ranger
(
  sensor
  (
    fov 360.0
    samples 640
    range [0.2 10]
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.0 0.0  0.00  0.0]

  size [ 0.20 0.20 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
