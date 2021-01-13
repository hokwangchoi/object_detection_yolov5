#include "object_detection_yolov5/detector_yolov5.h"

#include <boost/python.hpp>
#include <cv_bridge/cv_bridge.h>
#include <python3.8/Python.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

namespace object_detection_yolov5 {

std::string parse_python_exception() {
  PyObject *type_ptr = NULL, *value_ptr = NULL, *traceback_ptr = NULL;
  PyErr_Fetch(&type_ptr, &value_ptr, &traceback_ptr);
  std::string ret("Unfetchable Python error");
  if (type_ptr != NULL) {
    bp::handle<> h_type(type_ptr);
    bp::str type_pstr(h_type);
    bp::extract<std::string> e_type_pstr(type_pstr);
    if (e_type_pstr.check())
      ret = e_type_pstr();
    else
      ret = "Unknown exception type";
  }
  if (value_ptr != NULL) {
    bp::handle<> h_val(value_ptr);
    bp::str a(h_val);
    bp::extract<std::string> returned(a);
    if (returned.check())
      ret += ": " + returned();
    else
      ret += std::string(": Unparseable Python error: ");
  }
  if (traceback_ptr != NULL) {
    bp::handle<> h_tb(traceback_ptr);
    bp::object tb(bp::import("traceback"));
    bp::object fmt_tb(tb.attr("format_tb"));
    bp::object tb_list(fmt_tb(h_tb));
    bp::object tb_str(bp::str("\n").join(tb_list));
    bp::extract<std::string> returned(tb_str);
    if (returned.check())
      ret += ": " + returned();
    else
      ret += std::string(": Unparseable Python traceback");
  }
  return ret;
}

ObjectDetectorYolov5::ObjectDetectorYolov5(const std::string& topic) {
  // Initialize the python instance.
  Py_Initialize();

  // Define a python program.
  wchar_t programname[] = L"object_detector_yolov5.py";
  wchar_t* argv[] = {programname, nullptr};
  PySys_SetArgv(1, argv);

  // Load the yolov5 model.
  try {
    const bp::object module(bp::import("detect"));
    yolov5_net = module.attr("detect");
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
  // Convert the image
  // Run inference (python code) and get the correct output.
  yolov5_net();

  // Create a detection for each bounding box
  vision_msgs::Detection2DArray array_msg;

  // Publish the output
  // // if objects were detected, send out message
  // if (numDetections > 0) {
  //   ROS_INFO(
  //       "detected %i objects in %ux%u image", numDetections,
  //       input_msg->width, input_msg->height);

  //   for (int n = 0; n < numDetections; n++) {
  //     detectNet::Detection* det = *detections_dptr.get() + n;

  //     ROS_INFO(
  //         "object %i class #%u (%s)  confidence=%f", n, det->ClassID,
  //         net_->GetClassDesc(det->ClassID), det->Confidence);
  //     ROS_INFO(
  //         "object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n,
  //         det->Left, det->Top, det->Right, det->Bottom, det->Width(),
  //         det->Height());

  //     // create a detection sub-message
  //     vision_msgs::Detection2D detMsg;

  //     detMsg.bbox.size_x = det->Width();
  //     detMsg.bbox.size_y = det->Height();

  //     float cx, cy;
  //     det->Center(&cx, &cy);

  //     detMsg.bbox.center.x = cx;
  //     detMsg.bbox.center.y = cy;

  //     detMsg.bbox.center.theta = 0.0f;

  //     // create classification hypothesis
  //     vision_msgs::ObjectHypothesisWithPose hyp;

  //     hyp.id = det->ClassID;
  //     hyp.score = det->Confidence;

  //     detMsg.results.push_back(hyp);
  //     array_msg.detections.push_back(detMsg);
  //   }
  // }

  // // populate timestamp in header field
  // array_msg.header.stamp = input_msg->header.stamp;

  // // publish the detection message
  // detection_pub_.publish(array_msg);

  // // generate the overlay (if there are subscribers)
  // if (overlay_pub_.getNumSubscribers() > 0) {
  //   ROS_DEBUG("overlay image being processed.");
  //   // get the image dimensions
  //   const uint32_t width = input_cvt_->getWidth();
  //   const uint32_t height = input_cvt_->getHeight();

  //   // assure correct image size
  //   if (!overlay_cvt_->resize(width, height,
  //   ImageConverter::rosOutputFormat)) {
  //     ROS_ERROR(
  //         "failed to resize overlay image converter on %ux%u image", width,
  //         height);
  //   }

  //   // generate the overlay with bounding box, label and confidence
  //   if (!net_->Overlay(
  //           input_cvt_->imageGPU().get(), overlay_cvt_->imageGPU().get(),
  //           width, height, ImageConverter::internalFormat,
  //           *detections_dptr.get(), numDetections, overlay_flags)) {
  //     ROS_ERROR("failed to overlay the image on %ux%u image", width, height);
  //   }

  //   // populate the message
  //   sensor_msgs::Image image_msg;
  //   if (!overlay_cvt_->convert(image_msg, ImageConverter::rosOutputFormat)) {
  //     ROS_ERROR(
  //         "failed to convert the image for ros output %ux%u image", width,
  //         height);
  //   }

  //   // populate timestamp in header field
  //   image_msg.header.stamp = input_msg->header.stamp;

  //   // publish the message
  //   overlay_pub_.publish(image_msg);
  //   ROS_DEBUG("publishing %ux%u overlay image", width, height);
  // }
}

}  // namespace object_detection_yolov5
