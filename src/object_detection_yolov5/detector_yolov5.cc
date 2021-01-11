#include "object_detection_yolov5/detector_yolov5.h"

#include <Python.h>
#include <boost/python.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

namespace object_detection_yolov5 {

ObjectDetectorYolov5::ObjectDetectorYolov5(const std::string& topic) {
  // TODO(choi0330): Load the model with the python code here.
  Py_Initialize();

  constexpr char kDetectedTopic[] = "/detected";
  constexpr char kOverlayedTopic[] = "/overlayed";
  detection_pub_ =
      private_node_handle_.advertise<vision_msgs::Detection2DArray>(
          topic + kDetectedTopic, kQueueSize);
  overlay_pub_ = private_node_handle_.advertise<sensor_msgs::Image>(
      topic + kOverlayedTopic, kQueueSize);
  sub_image_ = it_.subscribe(
      topic, kQueueSize, &ObjectDetectorYolov5::imageCallback, this);
}

void ObjectDetectorYolov5::imageCallback(
    const sensor_msgs::ImageConstPtr& img_msg) {
  // Run the inference with the python function here.
}

}  // namespace object_detection_yolov5
