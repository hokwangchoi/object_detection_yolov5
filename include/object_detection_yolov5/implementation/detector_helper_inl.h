#ifndef OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_
#define OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_

#include <string>

#include <boost/python/numpy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <python3.8/Python.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace object_detection_yolov5 {

namespace bp = boost::python;
namespace np = boost::python::numpy;

inline cv::Mat ConvertRosImageMsgToCV(
    const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImagePtr cv_ptr_rgb;
  try {
    cv_ptr_rgb =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    std::terminate();
  }
  return cv_ptr_rgb->image;
}

inline np::ndarray ConvertMatToNDArray(const cv::Mat& mat) {
  bp::tuple shape = bp::make_tuple(mat.rows, mat.cols, mat.channels());
  bp::tuple stride = bp::make_tuple(
      mat.channels() * mat.cols * sizeof(uchar), mat.channels() * sizeof(uchar),
      sizeof(uchar));
  np::dtype dt = np::dtype::get_builtin<uchar>();
  np::ndarray ndImg = np::from_data(mat.data, dt, shape, stride, bp::object());

  return ndImg;
}

inline std::string parse_python_exception() {
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

}  // namespace object_detection_yolov5

#endif  // OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_
