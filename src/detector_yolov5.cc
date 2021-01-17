#include "object_detection_yolov5/detector_yolov5.h"

#include <boost/python/numpy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <python3.8/Python.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>
#include "object_detection_yolov5/implementation/detector_helper_inl.h"

namespace object_detection_yolov5 {

ObjectDetectorYolov5::ObjectDetectorYolov5(const std::string& topic) {
  // Initialize the python instance.
  Py_Initialize();
  np::initialize();

  // Define a python program.
  wchar_t programname[] = L"object_detector_yolov5.py";
  wchar_t* argv[] = {programname, nullptr};
  PySys_SetArgv(1, argv);

  // Load the yolov5 model.
  try {
    // TODO(choi0330): get a correct path to not depend on the terminal pwd.
    // TODO(choi0330): remove all possible dependencies and make a small
    // directory for yolov5.
    python_module = bp::import("detect");
    python_class = python_module.attr("DetectorYoloV5");
    yolov5_net = python_class(0.25, 0.45);
  } catch (boost::python::error_already_set const&) {
    std::cout << "Exception in Python:" << parse_python_exception()
              << std::endl;
    std::terminate();
  }

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
  // Convert the image to BGR8 which is needed for the yolov5 network.
  cv::Mat cv_image = cv::Mat(
      (*img_msg).height, (*img_msg).width, CV_8UC3,
      const_cast<uchar*>(&(*img_msg).data[0]), (*img_msg).step);

  // Run inference and get the detection output.
  std::vector<double> result_vector;
  int num_detections = 0;
  try {
    bp::object result =
        yolov5_net.attr("detect")((ConvertMatToNDArray(cv_image)));
    if (result) {
      np::ndarray nd_array = bp::extract<bp::numpy::ndarray>(result);
      num_detections = nd_array.shape(0);
      const int input_size = nd_array.shape(0) * nd_array.shape(1);
      double* input_ptr = reinterpret_cast<double*>(nd_array.get_data());
      for (int i = 0; i < input_size; ++i) {
        result_vector.push_back(*(input_ptr + i));
        std::cout << result_vector[i] << " ";
      }
    } else {
      num_detections = 0;
    }
  } catch (boost::python::error_already_set const&) {
    std::cout << "Exception in Python:" << parse_python_exception()
              << std::endl;
    std::terminate();
  }

  // Create a detection for each bounding box
  vision_msgs::Detection2DArray array_msg;

  // Publish the output
  // if objects were detected, send out message
  if (num_detections > 0) {
    ROS_INFO(
        "detected %i objects in %ux%u image", num_detections, img_msg->width,
        img_msg->height);

    for (int n = 0; n < num_detections; n++) {
      vision_msgs::ObjectHypothesisWithPose hyp;
      hyp.id = result_vector[n * 6];
      hyp.score = result_vector[n * 6 + 1];

      vision_msgs::Detection2D detMsg;
      detMsg.bbox.center.x = result_vector[n * 6 + 2];
      detMsg.bbox.center.y = result_vector[n * 6 + 3];
      detMsg.bbox.center.theta = 0.0f;
      detMsg.bbox.size_x = result_vector[n * 6 + 4];
      detMsg.bbox.size_y = result_vector[n * 6 + 5];

      detMsg.results.push_back(hyp);
      array_msg.detections.push_back(detMsg);
    }
  }

  // populate timestamp in header field
  array_msg.header.stamp = img_msg->header.stamp;

  // publish the detection message
  detection_pub_.publish(array_msg);

  // Generate the overlay image(if there are subscribers).
  // if (overlay_pub_.getNumSubscribers() > 0) {
  // }
}

}  // namespace object_detection_yolov5
