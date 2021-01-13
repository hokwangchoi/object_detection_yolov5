#include <exception>

#include <boost/python.hpp>
#include <python3.8/Python.h>
#include <ros/ros.h>

#include "object_detection_yolov5/detector_yolov5.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "detector");
  ros::NodeHandle private_node_handle("~");
  std::string rostopic;
  if (!private_node_handle.getParam("rostopic_name", rostopic)) {
    ROS_ERROR(
        "The rostopic is missing. Add rostopic_name to launch file or add an "
        "argument _rostopic_name:=.");
    return 1;
  }
  try {
    ROS_INFO("Reading rostopic %s", rostopic.c_str());
    const object_detection_yolov5::ObjectDetectorYolov5 detector_node(rostopic);
    ROS_INFO("detector node initialized, waiting for messages");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Initializing node failed due to %s.", e.what());
  }
  return 0;
}
