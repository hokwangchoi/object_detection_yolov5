**Environment**
* (recommended) Ubuntu 20 or (not tested) Boost python with python3.
* Ros noetic.

**Dependencies**
* [catkin_simple](https://github.com/catkin/catkin_simple)
* [python_module](git@github.com:ethz-asl/schweizer_messer.git): branch: feature/python_version_arg
* [opencv3_catkin](https://github.com/ethz-asl/opencv3_catkin)

**How to run**
$ cd catkin_ws_noetic/src
$ git clone --recursive https://github.com/choi0330/object_detection_yolov5.git
$ catkin build object_detection_yolov5
$ source ../devel/setup.bash
$ cd object_detection_yolov5/src/yolov5
$ rosrun object_detection_yolov5 detector_node_yolov5 _rostopic_name:=/to/image/topic
