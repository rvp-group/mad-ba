Surfel Cloud RViz Plugin
------------------------

Plugin for the ROS (Robot Operating System) visualizer RViz, to display surfel clouds.
Surfel clouds are point clouds where each point has a normal and a radius, i.e., it represents a small oriented disk.

This plugin adds a new display type, named `SurfelCloud`.

This plugin is an extension of the built-in PointCloud2 plugin, and likewise it receives messages of type `sensor_msgs/PointCloud2`.
However, a new display style is present, named `Surfels`, selectable using the `Style` drop-down menu.

The fields `normal_x`, `normal_y`, `normal_z` and `radius` must be present in the point cloud. This requirement is compatible with the `pcl::PointSurfel` point type from PCL (Point Cloud Library).

The `surfel_cloud_rviz_plugin` was tested on ROS Kinetic and Melodic.

2019-08-08