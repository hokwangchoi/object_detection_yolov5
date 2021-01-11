cmake_minimum_required(VERSION 3.5)
project(object_detection_yolov5)

variable_watch(PYTHON_LIBRARY)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# CUDA is required
find_package(CUDA)

# find_package(PythonInterp)
# find_package(PythonLibs)

set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.8.so)
set(PYTHON_INCLUDE_DIR /usr/include/python3.8)

add_python_export_library(${PROJECT_NAME} python/object_detection_yolov5
  src/nodes/test_python.cc
)

# cs_add_library(${PROJECT_NAME}_lib
#   src/object_detection_yolov5/detector_yolov5.cc
# )
# cs_add_executable(detector_node_yolov5 src/nodes/detector_node_yolov5.cc)
# target_link_libraries(detector_node_yolov5 ${PROJECT_NAME}_lib)

cs_add_executable(test_python src/nodes/test_python.cc)
target_link_libraries(test_python ${PROJECT_NAME})

# cs_add_executable(test_boost src/nodes/test_boost.cc)

cs_install()
cs_export()
