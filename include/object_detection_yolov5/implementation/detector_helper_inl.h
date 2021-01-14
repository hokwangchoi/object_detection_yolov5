#ifndef OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_
#define OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_

#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace object_detection_yolov5 {

inline cv::Mat ConvertRosImageMsgToCV(
    const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImagePtr cv_ptr_rgb;
  try {
    cv_ptr_rgb =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    std::terminate();
  }
  return cv_ptr_rgb->image;
}

}  // namespace object_detection_yolov5

#endif  // OBJECT_DETECTION_YOLOV5_IMPLEMENTATION_DETECTOR_HELPER_INL_H_
