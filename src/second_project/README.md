# Second ROS Project
This repository contains the implementation of the second project of ROS, for the course of Robotics.

## Authors

The autors of the project are:

-  **10719101 Filippo Balzarini**   ([@filomba01](https://github.com/filomba01)) _filippo.balzarini@mail.polimi.it_
-  **10787158 Christian Biffi** ([@creix](https://github.com/creix)) _christian.biffi@mail.polimi.it_
- **10706553 Michele Cavicchioli** ([@trueMaicol](https://github.com/trueMaicol)) _michele.cavicchioli@mail.polimi.it_

From Politecnico di Milano.

## Goal

The Goal of this project is  to perform mapping and then navigation on the created environment, the mapping has been performed with data given in the bagfiles,  collected by a skid-steering robot, with a footprint of 0.6 x 0.4 meters.

## Mapping

In order to perform mapping we have chose to use **[slam toolbox](https://wiki.ros.org/slam_toolbox)**, we configured the parameters as can be seen in [slam_toolbox_param.yalm](cfg/slam_toolbox_param.yaml), after trying different _resolutions_ and _map update intervals_ , we found out that good bias was having a pretty high resolution of the **cost map**, so we choose

```
map_update_interval: 3.0
resolution: 0.02
```

With the above settings we performed mapping with both of the robot's sensors: 2D and 3D laserscans, creating two maps for each bagfile.

### 3D laserscans conversion

Because of **slam toolbox** deals only with 2D scans, we flattened the 3D scans with **[pointcloud_to_laserscan](https://wiki.ros.org/pointcloud_to_laserscan)** package, we published the flattened data in the topic _/scan_converted_ and then given to slam toolbox. We decided to flat the scans between 0.2 and 0.6 of height, in order to map only relevant obstacles. The complete configuration can be found on the launch file [mapping_3D.launch](launch/mapping_3D.launch) and is resumed below:

```
<remap from="cloud_in" to="/velodyne_points"/>
<remap from="scan" to="/scan_converted"/>
<param name="min_height" value="0.2"/>
<param name="max_height" value="0.6"/>
<param name="angle_increment" value="0.004"/>
<param name="range_max" value="20.0"/>
```
## Navigation

In the navigation part of the project we had to correctly setup all move base environment for navigate the created map.
We used **[amcl](https://wiki.ros.org/amcl)** in order to correct the odometry and scans error of the robot and **stage** for the robot simulation.

### Stage

In order to perform a correct navigation we had to simulate the robot in its environment, that has been possible using **stage**. We created the robot and simulated a realistic error in scans and encorders.

For the encoders we assumed an odometry error of:
```
odom_error [0.01 0.01 0.01 0.01 0.01 0.01]
```

The scan we created for the simulation is described below:
```
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
```

In order to correctly match the robot scans in the stage with the navigation, we had to deal with the different system of reference used by slam toolbox, which set the origin in the bottom-left point of the map, and stage, that set it in the center of the map. So, in order to match the slam tool box origin, given in:
```
origin: [-8.932366, -5.748724, 0.000000]
```

We set the center of the map in stage as follow:
```
floorplan
(
  name "maze"
  bitmap "3d_bag1.pgm"
  size [ 18.78 31.0 2.0 ]
  pose [ 0.457634 9.751276 0.0 0.0 ]
)
```

### AMCL

For performing a correct localization we use AMCL.
AMCL is a ros package that make it possible to correct robot odometry and scan errors. We configured AMCL using a **Scan Sensor Model**, all the important [configuration of AMCL](launch/amcl.launch.xml) can be found below:
```
<param name="odom_model_type"           value="diff-corrected"/>
```
The model type has been set to _diff-corrected_, which is a differential drive model with some resolved bugs compared to _diff_.
```
<param name="laser_max_beams"           value="640"/>
<param name="laser_max_range"           value="20.0"/>
```
The above parameters are the characteristics of the robot's sensor.
```
<param name="odom_alpha1"               value="0.005"/>
<param name="odom_alpha2"               value="0.005"/>
<param name="odom_alpha3"               value="0.005"/>
<param name="odom_alpha4"               value="0.005"/>
```
Here we set the odometry error parameters based on the robot encoder, we do not neeed to specify odom_alpha5, that is because is only for omnidirectional robot.
The parameters seem to be a bit low, but that's because we use **diff_corrected** model, which suggests lower values on odometry error, as noted in [this link](https://answers.ros.org/question/227811/tuning-amcls-diff-corrected-and-omni-corrected-odom-models/).
```
<param name="laser_z_hit"               value="0.8"/>
<param name="laser_z_rand"              value="0.2"/>
```
Those are the parameters of our sensor model, because we use the Scan Sensor Model, the only two parameters to be tuned are the probability of hitting an obstacle and the probability of getting a random measurement.

## Move Base
In order to perform a correct Navigation we used **[move base](https://wiki.ros.org/move_base)**.

Move base needs to be configured, in particular it needs the source of the odometry, corrected by AMCL, the map and the sensor source, this one given by Stage, and, using a Global and Local cost map, and both a global and local planner, is able to perform **Navigation**.

### Common Costmap parameters
Both global and local planner shares basic parameters, such as the footprint of the robot and the [inflation](https://wiki.ros.org/costmap_2d/hydro/inflation).

The robot footprint is set as the footprint of the skid-steering robot, and it is
```
footprint: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]
```
For the inflation layer we choose those configurations:
```
 inflation_layer:
    enabled:              true
    cost_scaling_factor:  30.0
    inflation_radius:     0.4
```
We tried different cost_scaling_factors, we reduced the cost in order to obtain smoother navigation. For the inflation radius we set 0.4, that's because the inscripted radius of the robot was 0.36, so having a 0.04 of bias has been resulted a good option for creating a realistic costmap.

### Global Costmap Parameters

Included the discussed above configurations, we just set the correct frames and the update frequency as shown below:

```
global_frame: map
robot_base_frame: base_footprint
update_frequency: 1.0
```

### Local costmap parameters
The Local costmap contains the cost of every point of the local map, in a 3x3 meters area. We configured the local costmap as follow, in order to both reduce as possible the computational effort and obtaining good results during navigation.

```
  width: 3.0
  height: 3.0
  resolution: 0.05
```

The resolution could actually be set to 0.02, but because of the computational effort, we decided to set it to 0.05.

### Global Planner
The aim of the global planner is to find the best path from the robot position to the goal.

For the global planner we choose **[Global Planner package](https://wiki.ros.org/global_planner)**

We configured the planner in order to use the **A\* Alghoritm**, all the parameters given to the Global planner are:
```
<param name="base_global_planner" value="global_planner/GlobalPlanner" />
<param name="planner_frequency" value="1.0" />
<param name="planner_patience" value="5.0" /> 
<param name="use_dijkstra" value="false" />
```

### Local Planner
The aim of the local planner is to taking into account obstacle avoidance while following the path given by the global planner.

For the Local planner, we chose **[Teb local planner](https://wiki.ros.org/teb_local_planner)**, for the configuration of the planner we set a goal tolerance of 0.2 meters, and a rate of the cost map conversion of 5Hz, that is because we want our local planner able to notice the costmap rapidly when individuate an obstacle.
The more important setting manipulation we made are:
```
xy_goal_tolerance: 0.2

footprint_model:
    type: "polygon"
    vertices: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]

# Obstace avoidance
min_obstacle_dist: 0.03   
.
```
All the speific configuration can be found in the [local planner configuration file](cfg/teb_local_planner_params.yaml)