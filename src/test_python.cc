#include <iostream>

#include <boost/python/numpy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <python3.8/Python.h>

namespace bp = boost::python;
namespace np = boost::python::numpy;

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

static bool termineAlreadyInfiltrated;
void pythonFriendlyTerminate() {
  auto e = std::current_exception();
  if (e) {
    try {
      std::rethrow_exception(e);
    } catch (boost::python::error_already_set const&) {
      std::cout << "Exception in Python:" << parse_python_exception();
    }
  }
  std::terminate();
}

np::ndarray ConvertMatToNDArray(const cv::Mat& mat) {
  bp::tuple shape = bp::make_tuple(mat.rows, mat.cols, mat.channels());
  bp::tuple stride = bp::make_tuple(
      mat.channels() * mat.cols * sizeof(uchar), mat.channels() * sizeof(uchar),
      sizeof(uchar));
  np::dtype dt = np::dtype::get_builtin<uchar>();
  np::ndarray ndImg = np::from_data(mat.data, dt, shape, stride, bp::object());

  return ndImg;
}

int main() {
  if (!termineAlreadyInfiltrated) {
    std::set_terminate(pythonFriendlyTerminate);
    termineAlreadyInfiltrated = true;
  }

  // Initialize the python instance
  Py_Initialize();
  np::initialize();

  wchar_t programname[] = L"yolov5_node.py";
  wchar_t* argv[] = {programname, nullptr};
  PySys_SetArgv(1, argv);

  try {
    bp::object main = bp::import("__main__");
    bp::object globals = main.attr("__dict__");

    // Run a simple string (verify the python version).
    bp::exec(
        "import sys\n"
        "print(sys.version)",
        globals);

    bp::object module = bp::import("detect");
    bp::object detector_class = module.attr("DetectorYoloV5");
    bp::object detector_instance = detector_class(0.25, 0.45);
    std::string image_path =
        "/hokwang/data/bags/ghost_bag_cam0/frame000000.png";
    cv::Mat mat = cv::imread(image_path, cv::IMREAD_COLOR);
    bp::object result =
        detector_instance.attr("detect")(ConvertMatToNDArray(mat));

    np::ndarray nd_array = bp::extract<bp::numpy::ndarray>(result);
    std::cout << "py_array: " << bp::extract<char const*>(bp::str(nd_array))
              << std::endl;

    int input_size = nd_array.shape(0) * nd_array.shape(1);
    double* input_ptr = reinterpret_cast<double*>(nd_array.get_data());
    std::vector<double> v(input_size);
    for (int i = 0; i < input_size; ++i) {
      v[i] = *(input_ptr + i);
      std::cout << v[i] << " ";
    }
    std::cout << std::endl;
  } catch (boost::python::error_already_set const&) {
    std::cout << "Exception in Python:" << parse_python_exception()
              << std::endl;
  }
}
