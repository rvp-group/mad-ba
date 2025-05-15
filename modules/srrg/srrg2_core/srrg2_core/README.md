# SRRG2 CORE | [Wiki](https://gitlab.com/srrg-software/srrg2_core/wikis/home)

This package contains some essential libraries and utilities extensively used in
the srrg repos. These include
* base defines (2D/3D transforms, points, elementary linear algebra operations)
* base system utilities (monitoring the CPU frequency of a module, printing a banner)
* reading/writing text formatted log files (txt_io)
* computing 2D distance maps for nearest neighbor search
* computing KD trees for nn search in multidimensional spaces
* drawing lines on a 2D map

Besides the libraries, the package provides also a set of utilities
to convert and merge text based log files plus simple image processing tools

## Prerequisites

requires:
* [srrg_boss](https://gitlab.com/srrg-software/srrg_boss)
* [srrg_cmake_modules](https://gitlab.com/srrg-software/srrg_cmake_modules)

## Applications
The following are some applications provided in the package

* `srrg_messages_converter_euroc_app`: converts from euroc scripts to txt_io format
* `srrg_messages_merger_app`: merges two or more txt_io logs preserving the temporal  order
* `srrg_messages_converter_kitti_app`: converts from kitti_format to txt_io
* `srrg_messages_sorter_app`: sorts a txt_io log based on the timestamp of the records

Image processing (Fully self-contained and depending only on OpenCV2/3)

* `visual_tracker`: tracks a specified number of features for a specified descriptor type over a sequence of images
* `visual_stereo_matcher`: matches a specified number of features for a specified descriptor type of a stereo image pair

To get usage information launch any of the previous programs with the `-h` option:

    rosrun srrg2_core srrg_messages_merger_app -h


## Examples 
SRRG core builds a bunch of examples to help using the libraries.
These include:
* `srrg_messages_synchronizer_example`: example on how to generate an assoc file (for instance for orb slam), containing depth and rgb images
* `srrg_open_file_example`: how to open and parse a srrg file
* `srrg_kdtree_example`:    how to use a KD tree


## TXT-IO file structure
SRRG core uses a set of sensor-based messages.

A `BASE_SENSOR_MESSAGE` contains the following fields:
* _topic_, _frame-id_, _seq_, _timestamp_, _sensor-offset_, _odometry-flag_, _odometry_, _imu-flag_, _imu_

The derived messages are:
* `PINHOLE_IMAGE_MESSAGE`: _BASE-SENSOR-MESSAGE-FIELDS_, _depth-scaling_, _image-path_, _camera-matrix_
* `SPHERICAL_IMAGE_MESSAGE`: _BASE-SENSOR-MESSAGE-FIELDS_, _depth-scaling_, _image-path_, _camera-matrix_
* `IMU_MESSAGE`: _BASE-SENSOR-MESSAGE-FIELDS_, _orientation (qw, qx, qy, qz)_, _angular-velocity_, _linear-acceleration_
* `JOINT_STATE_MESSAGE`:  _BASE-SENSOR-MESSAGE-FIELDS_, _num-joints_, _joint-name_, _joint-position_, _joint-velocity_, _joint-effort_, _[...]_
* `LASER_MESSAGE`: _BASE-SENSOR-MESSAGE-FIELDS_, _min-range_, _max-range_, _min-angle_, _max-angle_, _angle-increment_, _time-increment_, _scan-time_, _range-size_, _ranges[...]_, _intensity-size_, _intensitites[...]_

## Authors
* Giorgio Grisetti
* Jacopo Serafin
* Mayte Lazaro
* Maurilio Di Cicco
* Taigo Bonanni
* Bartolomeo Della Corte
* Dominik Schlegel

## License

BSD 2.0
