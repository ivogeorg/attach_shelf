### Publishing `PoseWithCovarianceStamped` on the command line

```
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}"
```

### Simulator

### Lab

```
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 0.054368, y: 0.060027, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.000433, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}"
```

                                                                                                                                                            0.054368, 0.060027,                                            -0.000433, 1.0000