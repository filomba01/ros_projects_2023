define laser ranger
(
  sensor
  (
    fov 360.0
    samples 640
    range [0.2 20]
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.005 0.005 0.005 0.005 0.01 0.01]

  size [ 0.20 0.20 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  laser(pose [ 0.0 0.0 0.0 0.0 ])
)
