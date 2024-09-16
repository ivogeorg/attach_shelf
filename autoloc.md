### Precise autolocalization

#### Initial pose estimate

1. Publish to `/initialpose`
2. Rviz2 is publisher in [`navigation.launch.py`](path_planner_server/launch/navigation.launch.py)
3. `amcl` is a subscriber
4. `/amcl_pose` is not published until `amcl` gets `/initialpose`
5. Covariance indices for quality of pose estimate {"x": 0, "y": 7, "z": 35}  

From `ros2 topic echo /initialpose`:  
```
pose:
  pose:
    position:
      x: -0.014335393905639648
      y: -0.01587963104248047
    orientation:
      z: 0.002442992457099749
      w: 0.9999970158894749
  covariance:
  - 0.25
  - 0.25
  - 0.06853891909122467
  ```

From `ros2 topic echo /amcl_pose`, **before** rotation in place:  
```
pose:
  pose:
    position:
      x: 0.0023854173123385156
      y: -0.020326054652300245
    orientation:
      z: -0.002061716952939946
      w: 0.9999978746593444
  covariance:
  - 0.2235436520487672
  - 0.18954617569475832
  - 0.056362588024433724
```

From `ros2 topic echo /amcl_pose`, **after** rotation in place:  
```
pose:
  pose:
    position:
      x: -0.0187536442759816
      y: -0.002762048499082799
    orientation:
      z: 0.0033806640362963854
      w: 0.9999942855390093
  covariance:
  - 0.020825227848328864
  - 0.022334623708799368
  - 0.013508614749857305
```