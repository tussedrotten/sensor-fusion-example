#pragma once

#include "Python.h"
#include <unordered_map>
#include <vector>

#if PY_MAJOR_VERSION >= 3
#  define PyString_FromString PyUnicode_FromString
#endif

namespace plot
{

/// \brief Loads and manages Python objects.
/// Assumes that it is the only Python interpreter.
class PythonObjectManager
{
public:
  PythonObjectManager();

  ~PythonObjectManager();

  /// \brief Add a python module with the specified name.
  void addModule(const std::string& module_name);

  /// \brief Add a python function with the specified module and function name.
  void addFunction(const std::string& module_name, const std::string& func_name);

  /// \brief Provides a pointer to an empty python tuple object for convenience.
  PyObject* emptyTuple();

  /// \brief Provides a pointer to a python module with the specified name.
  /// Module must first be loaded using addModule().
  PyObject* getModule(const std::string& module_name);

  /// \brief Provides a pointer to a python function with the specified module and function name.
  /// Module and function must first be loaded using addModule() and addFunction().
  PyObject* getFunction(const std::string& module_name, const std::string& func_name);

  /// \brief Provides a pointer to a python function with the specified module and function name.
  /// Name is given by "module_name.function_name".
  /// Module and function must first be loaded using addModule() and addFunction().
  PyObject* getFunction(const std::string& module_func_name);

private:
  std::unordered_map<std::string, PyObject*> py_modules_;
  std::unordered_map<std::string, PyObject*> py_functions_;
  PyObject* empty_tuple_;
};

}
