#ifndef OBJECT_DETECTION_YOLOV5_DETECTOR_YOLOV5_H_
#define OBJECT_DETECTION_YOLOV5_DETECTOR_YOLOV5_H_

#include <string>

#include <boost/python.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <python3.8/Python.h>
#include <ros/ros.h>

namespace object_detection_yolov5 {

namespace bp = boost::python;

class ObjectDetectorYolov5 {
 public:
  explicit ObjectDetectorYolov5(const std::string& topic);

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);
  static constexpr size_t kQueueSize = 10u;

  ros::NodeHandle private_node_handle_;
  image_transport::ImageTransport it_{private_node_handle_};
  image_transport::Subscriber sub_image_;
  ros::Publisher detection_pub_, overlay_pub_;

  // Yolov5 model.
  bp::object python_module;
  bp::object python_class;
  bp::object yolov5_net;
};

}  // namespace object_detection_yolov5

#endif  // OBJECT_DETECTION_YOLOV5_DETECTOR_YOLOV5_H_
