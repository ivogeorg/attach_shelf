### attach_shelf

An RB1 robot in a simulated warehouse world moves forward, turns, detects a shelf, moves underneath it and attaches to it by raising its elevator.

![RB1 attaches to shelf](assets/rb1_attach_shelf.gif)  

#### Submission notes

TODO  


#### Implementation notes*

_*Cumulative for both Task 1 and Task 2._  

##### 1. Put robot in intial state

_Optional, but good practice with entity states._  

1. Can use the `/demo/set_entity_state` service to put the robot in the initial position for repeated testing.  
2. The service is of type `gazebo_msgs/srv/SetEntityState`.
3. The service server is provided by node `/demo/gazebo_ros_state`.  
4. Probably need to use `/demo/get_entity_state` to get the initial state.  

##### 2. ROS2 objects

1. Subscriber to `/scan` (`sensor_msgs/msg/LaserScan`).
2. Subscriber to `/odom` (`nav_msgs/msg/Odometry`).
2. Publisher to `/diffbot_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`).
3. Client to service `/approach_shelf` (custom `GoToLanding.srv`).  
4. Publisher to `/elevator_up` (`std_msgs/msg/Empty`).  
5. Publisher to `/elevator_down` (`std_msgs/msg/Empty`).  

##### 3. Adding a frame

Need to add a frame in the middle of the reflective plates of the shelf.  

1. Use the `intensities` array of the `sensor_msgs/msg/LaserScan` to identify the points of incidence on the two reflective plates. The intensities should be significantly higher for them than other points.
2. There should be two clusters. Use the `ranges` to get their centers. Then find the midpoint between them.
3. Use this point to add a frame `cart_frame` to the tree. Tutorial for [adding a frame](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp.html).  


##### 4. `tf2_ros::TransformListener` for precision movement

1. Use a `TransformListener` with `cart_frame` and `robot_base_footprint` frame of the robot to issue precision commands for the final approach.
