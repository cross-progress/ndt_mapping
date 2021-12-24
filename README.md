ndt_mapping
====
![CI](https://github.com/rsasaki0109/ndt_mapping/workflows/Melodic/badge.svg)  
This is the exraction of the mapping from the [Autoware](https://github.com/autowarefoundation/autoware)  
I have no copyright on this software and the license is governed by autoware.
## Overview
ndt_mapping package

## IO
ndt_mapping 
- input  
/points_raw (sensor_msgs/PointCloud2)  
- output  
/ndt_map (sensor_msgs/PointCloud2)  
/curent_pose (geometry_msgs/PoseStamped) 

ndt_matching  
- input   
/filtered_points (sensor_msgs/PointCloud2)  
/points_map (sensor_msgs/PointCloud2)  
/initialpose (geometry_msgs/PoseWithCovarianceStamped)   

- output  
/curent_pose (geometry_msgs/PoseStamped)  

## Parameter

ndt

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|max_iter|int|max iteration for alignment |25|
|step_size|double|step_size maximum step length[m]|0.1|
|ndt_res|double|resolution side length of voxels[m]|1.0|
|transform_epsilon|double|transform epsilon to stop iteration|0.1|
|voxel_leaf_size|double|a down sample size of a input cloud[m]|0.2|

ndt_mapping 

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|min_add_scan_shift|double|a moving distance of a map update[m]|1.5|

## Usage
### Mapping 

```
rviz -d src/ndt_mapping/config/mapping.rviz
```

```
roslaunch ndt_mapping ndt_mapping.launch
```

to save a map

```
rosrun pcl_ros pointcloud_to_pcd input:=/ndt_map prefix:=map
```

### Matching

### Usage I (<span style="color:green"> recommended </span>)

Set the input map name and point cloud topic name in ndt_matching_combined.launch (Default: Map:map.pcd,  PointClound: /points_raw)

You can also change the ndt parameters such as max_iter and ndt_res in ndt_matching_combined.launch.


####  <span style="color:cyan"> ~ ndt_matching_combined.launch ~ </span> 
``` xml
<launch>
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ndt_mapping)/config/matching.rviz"/>
    <node pkg="tf" type="static_transform_publisher" name="velodyne" args="0 0 0 0 0 0 base_link velodyne 100" />
    <node pkg="ndt_mapping" name="map_loader" type="map_loader" output="screen">
        <param name="map_path" value="$(find ndt_mapping)/map/map.pcd"/>
    </node>
    <node pkg="ndt_mapping" name="voxel_grid_filter" type="voxel_grid_filter" output="screen">
        <param name="points_topic" value="/points_raw"/>
        <param name="voxel_leaf_size" value="2.0"/>
        <param name="measurement_range" value="200"/>
    </node>    
    <node pkg="ndt_mapping" type="ndt_matching" name="ndt_matching" output="screen">
        <param name="max_iter_" value="50"/>
        <param name="ndt_res_" value="5.0"/>
        <param name="step_size_" value="0.1"/>
        <param name="trans_eps_" value="0.01"/>
    </node>
    <arg name="leaf_size" default="0.5" />
</launch>
```


run the following command to load map and start ndt_matching

```
roslaunch ndt_mapping ndt_matching_combined.launch
```


run the following command or directly set the initial point by using rviz  (use "2D Pose Estimate" to select the initial point)

```
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header:{frame_id: "map"},pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {z: 0, w: 1}}}}'
```





### Usage II (<span style="color:red"> you need to do a point downsampling by yourself </span>)

```
rviz -d src/ndt_mapping/config/matching.rviz
```


```
roslaunch ndt_mapping ndt_matching.launch
```

```
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header:{frame_id: "map"},pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {z: 0, w: 1}}}}'
```

```
rosrun pcl_ros pcd_to_pointcloud map_0.pcd /cloud_pcd:=/points_map _frame_id:=map
```