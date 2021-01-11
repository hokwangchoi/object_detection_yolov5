#include <iostream>

#include <boost/python.hpp>
#include <python3.8/Python.h>

int main() {
  // Initialize the python instance
  Py_Initialize();

  // Run a simple string (test).
  PyRun_SimpleString(
      "import sys\n"
      "print(sys.version)");

  // Run a simple file
  // TODO(choi0330): Try to run python3.8 with python/C API or python boost.
  // FILE* PScriptFile = fopen("detect.py", "r");
  // if (PScriptFile) {
  //   PyRun_SimpleFile(PScriptFile, "detect.py");
  //   fclose(PScriptFile);
  // }

  // Close the python instance
  // Py_Finalize();
}
