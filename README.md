# `urban_road_filter`: a real-time LIDAR-based urban road and sidewalk detection algorithm for autonomous vehicles

<img src="img/urban_road_filter_anim01.gif" height=620/> <img src="img/urban_road_filter_static01.png" height=620/>

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

Issue the following commands to start roscore, download and play sample data, and start the algorithm with visualization. You can also watch this as a [youtube tutorial](https://www.youtube.com/watch?v=HHnj4VcbSy4).

In a **new terminal** start roscore:

```
roscore
```

In a **new terminal** go to your bag folder (e.g. `~/Downloads`):

```
cd ~/Downloads
```

Download a sample rosbag (~3,3 GB):

```r
wget https://laesze-my.sharepoint.com/:u:/g/personal/herno_o365_sze_hu/EYl_ahy5pgBBhNHt5ZkiBikBoy_j_x95E96rDtTsxueB_A?download=1 -O leaf-2021-04-23-campus.bag
```

Play rosbag:

```r
rosbag play -l ~/Downloads/leaf-2021-04-23-campus.bag
```

In a **new terminal** start the `urban_road_filter` node, `rviz` and `rqt_reconfigure` with roslaunch:

```
roslaunch urban_road_filter demo1.launch
```

# Cite & paper

If you use any of this code please consider citing the [paper](https://www.mdpi.com/1424-8220/22/1/194):

```bibtex
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

# Related solutions

- [`points_preprocessor`](https://github.com/Autoware-AI/core_perception/tree/master/points_preprocessor) `ray_ground_filter` and `ring_ground_filter` (ROS)
- [`linefit_ground_segmentation`](https://github.com/lorenwel/linefit_ground_segmentation) (ROS)
- [`curb_detection`](https://github.com/linyliny/curb_detection) (ROS)
- [`3DLidar_curb_detection`](https://github.com/SohaibAl-emara/3D_Lidar_Curb_Detection) (ROS)
- [`lidar_filter`](https://github.com/ZoltanTozser/lidar_filter)
- Many more algorithms without code mentioned in the [paper](https://doi.org/10.3390/s22010194).

# Videos and images

[<img src="img/yt_demo01.png" width=213/>](https://www.youtube.com/watch?v=T2qi4pldR-E)
[<img src="img/yt_tutorial01.png" width=213/>](https://www.youtube.com/watch?v=HHnj4VcbSy4)

[<img src="img/yt_demo02.png" width=213/>](https://www.youtube.com/watch?v=9tdzo2AyaHM)
[<img src="img/yt_demo03.png" width=213/>](https://www.youtube.com/watch?v=lp6q_QvWA-Y)

<img src="img/marker_poly01.png" width=440/>
<img src="img/marker_road_high01.png" width=440/>
<img src="img/marker_poly02.png" width=440/>

# ROS publications / subscriptions

```mermaid
flowchart LR

P[points]:::gray -->|sensor_msgs/PointCloud2| U([urban_road_filt</br>node]):::gray
U --> |sensor_msgs/PointCloud2| A[curb]:::gray
U --> |sensor_msgs/PointCloud2| B[road]:::gray 
U --> |sensor_msgs/PointCloud2| C[road_probably]:::gray
U --> |sensor_msgs/PointCloud2| D[roi]:::gray
U --> |visualization_msgs/MarkerArray| E[road_marker]:::gray


classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef gray fill:#f6f8fa,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff

```
