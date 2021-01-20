# Yolov5 ros node
This repository introduces a ros node subscribing to a ros topic image and publishing detection arrays and the overlayed image.

## Environment
* (recommended) Ubuntu 20 or (not tested) Boost with python3 and all ros packages depending on python3
* [Ros noetic](http://wiki.ros.org/noetic)

## Dependencies
* [`catkin_simple`](https://github.com/catkin/catkin_simple)
* [`python_module`](https://github.com/ethz-asl/schweizer_messer.git): branch: feature/python_version_arg
* [`opencv3_catkin`](https://github.com/ethz-asl/opencv3_catkin)
* [`vision_msgs`](https://github.com/ros-perception/vision_msgs): branch: noetic-devel

## GPU set up
* `Yolov5` can run on CPU, but using GPU is strongly recommended.
* Download a corresponding nvidia driver and confirm that `$ nvidia-smi` shows the result.

## (Optional) Docker image
* If required, use the following docker image providing Ubuntu20, ros noetic and all necessary python packages for running yolov5.
* First, to run GPU on the docker image, set up [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker) which leverages GPU usage on the docker image.
```
$ docker run --ipc=host --gpus all -v /path/to/host/dir/to/mount:/path/name/to/mounted/point --net=host -it hochoi/environment_ros_node_yolov5:noetic_desktop-full-focal-yolov5 /bin/bash
```

## How to run
```
$ cd catkin_ws_noetic/src  # Make sure the catkin workspace is set up.
$ git clone --recursive https://github.com/choi0330/object_detection_yolov5.git
$ cd object_detection_yolov5/src/yolov5
$ pip3 install -r requirements.txt  # Python version >= 3.8
$ catkin build object_detection_yolov5
$ source ../devel/setup.bash
$ cd object_detection_yolov5/src/yolov5
$ rosrun object_detection_yolov5 detector_node_yolov5 _rostopic_name:=/image/topic/name
```

## Disclaimer
* This ros node is only able to subscribe to a MONO8 (gray scale) image.
* (If you consider to change the code) `cv_bridge` incurs an unknown compatibility issue with the Boost python under python3.
* `Yolov5` will automatically download the model `yolov5s.pt`. In the future, if there is a breaking change in the model design, please download a model with [v3.1](https://github.com/ultralytics/yolov5/releases/download/v3.1/yolov5s.pt). 
