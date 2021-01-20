#include "object_detection_yolov5/detector_yolov5.h"

#include <cmath>
#include <cstdlib>

#include <boost/python/numpy.hpp>
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
    const sensor_msgs::ImageConstPtr& input_msg) {
  // TODO(choi0330): Boost python fails with cv_brdige, try to fix it.
  // Assume the input image as MONO8 and convert it to BGR8 which is needed for
  // the yolov5 network.
  cv::Mat cv_image_raw = cv::Mat(
      (*input_msg).height, (*input_msg).width, CV_8UC1,
      const_cast<uchar*>(&(*input_msg).data[0]), (*input_msg).step);
  cv::Mat cv_image;
  cv::cvtColor(cv_image_raw, cv_image, cv::COLOR_GRAY2BGR);

  // Run inference and get the detection output.
  std::vector<double> result_vector;
  int num_detections = 0;
  try {
    bp::object result =
        yolov5_net.attr("detect")((ConvertMatToNDArray(cv_image)));
    np::ndarray nd_array = bp::extract<bp::numpy::ndarray>(result);
    num_detections = nd_array.shape(0);
    if (num_detections > 0) {
      const int input_size = nd_array.shape(0) * nd_array.shape(1);
      double* input_ptr = reinterpret_cast<double*>(nd_array.get_data());
      for (int i = 0; i < input_size; ++i) {
        result_vector.push_back(*(input_ptr + i));
      }
    }
  } catch (boost::python::error_already_set const&) {
    std::cout << "Exception in Python:" << parse_python_exception()
              << std::endl;
    std::terminate();
  }

  // Create a detection for each bounding box.
  vision_msgs::Detection2DArray array_msg;

  // Publish the output.
  if (num_detections > 0) {
    for (int n = 0; n < num_detections; n++) {
      vision_msgs::ObjectHypothesisWithPose hyp;
      static constexpr int kIdIndex = 0;
      static constexpr int kConfidenceIndex = 1;
      static constexpr int kXIndex = 2;
      static constexpr int kYIndex = 3;
      static constexpr int kWIndex = 4;
      static constexpr int kHIndex = 5;
      hyp.id =
          std::to_string(static_cast<int>(result_vector[n * 6 + kIdIndex]));
      hyp.score = result_vector[n * 6 + kConfidenceIndex];

      vision_msgs::Detection2D detMsg;
      detMsg.bbox.center.x = result_vector[n * 6 + kXIndex];
      detMsg.bbox.center.y = result_vector[n * 6 + kYIndex];
      detMsg.bbox.center.theta = 0.0f;
      detMsg.bbox.size_x = result_vector[n * 6 + kWIndex];
      detMsg.bbox.size_y = result_vector[n * 6 + kHIndex];

      // Generate the overlay image.
      if (overlay_pub_.getNumSubscribers() > 0) {
        cv::Point min_point(
            detMsg.bbox.center.x - 0.5 * detMsg.bbox.size_x,
            detMsg.bbox.center.y - 0.5 * detMsg.bbox.size_y);
        cv::Point max_point(
            detMsg.bbox.center.x + 0.5 * detMsg.bbox.size_x,
            detMsg.bbox.center.y + 0.5 * detMsg.bbox.size_y);
        // Dark blue color.
        cv::Scalar color = cv::Scalar(135, 74, 32);
        cv::rectangle(
            cv_image, min_point, max_point, color, 2, cv::LineTypes::LINE_AA);
        std::string class_name;
        if (hyp.id == "0") {
          class_name = "Human";
        } else {
          class_name = hyp.id;
        }
        const std::string confidence = std::to_string(hyp.score);
        const std::string rounded =
            confidence.substr(0, confidence.find(".") + 3);
        cv::String label = class_name + std::string("  ") + rounded;
        cv::putText(
            cv_image, label, cv::Point(min_point.x, max_point.y - 2), 0, 0.5,
            cv::Scalar(225, 255, 255), 1.5, cv::LineTypes::LINE_AA);
      }
      detMsg.results.push_back(hyp);
      array_msg.detections.push_back(detMsg);
    }
  }
  array_msg.header = input_msg->header;

  // Publish the detection message.
  detection_pub_.publish(array_msg);

  if (overlay_pub_.getNumSubscribers() > 0) {
    sensor_msgs::Image image_msg;
    image_msg.header = input_msg->header;
    image_msg.height = cv_image.rows;
    image_msg.width = cv_image.cols;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.step = cv_image.cols * cv_image.elemSize();
    const size_t size = image_msg.step * cv_image.rows;
    image_msg.data.resize(size);
    memcpy(reinterpret_cast<char*>(&image_msg.data[0]), cv_image.data, size);

    ROS_DEBUG("publishing %ux%u overlay image", cv_image.rows, cv_image.cols);
    overlay_pub_.publish(image_msg);
  }
}

}  // namespace object_detection_yolov5
