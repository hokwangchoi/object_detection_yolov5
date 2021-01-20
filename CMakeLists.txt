cmake_minimum_required(VERSION 3.5)
project(object_detection_yolov5)

variable_watch(PYTHON_LIBRARY)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.8.so)
set(PYTHON_INCLUDE_DIR /usr/include/python3.8)

add_python_export_library(${PROJECT_NAME}_py_lib python/object_detection_yolov5
  src/detector_yolov5.cc
)

# TODO(choi0330): remove it if the numpy component is automatically added by add_python_export_library.
find_package(Boost REQUIRED COMPONENTS numpy)

cs_add_executable(detector_node_yolov5 src/nodes/detector_node_yolov5.cc)
target_link_libraries(detector_node_yolov5 ${PROJECT_NAME}_py_lib ${Boost_LIBRARIES})

cs_install()
cs_export()
