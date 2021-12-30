# `urban_road_filter`: a real-time LIDAR-based urban road and sidewalk detection algorithm for autonomous vehicles

<img src="img/urban_road_filter_anim01.gif" width=274/><img src="img/urban_road_filter_static01.png" width=274/>

# Dependency
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Kinetic and Melodic)
- [PCL](https://pointclouds.org/) 


# Install
Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/jkk-research/urban_road_filter
catkin build urban_road_filter
```

# Getting started

```
```


# Cite & paper

If you use any of this code please consider citing the [paper](https://www.mdpi.com/1424-8220/22/1/194):

```

@Article{roadfilt2022horv,
    title = {Real-Time LIDAR-Based Urban Road and Sidewalk Detection for Autonomous Vehicles},
    author = {Horváth, Ernő and Pozna, Claudiu and Unger, Miklós},
    journal = {Sensors},
    volume = {22},
    year = {2022},
    number = {1},
    url = {https://www.mdpi.com/1424-8220/22/1/194},
    issn = {1424-8220},
    doi = {10.3390/s22010194}
}
```

# Realated solutions

- [points_preprocessor](https://github.com/Autoware-AI/core_perception/tree/master/points_preprocessor) `ray_ground_filter` and `ring_ground_filter`

# Videos and images


![](img/marker_poly01.png)
![](img/marker_road_high01.png)
![](img/marker_poly02.png)

- https://www.youtube.com/watch?v=9tdzo2AyaHM
- https://www.youtube.com/watch?v=lp6q_QvWA-Y
