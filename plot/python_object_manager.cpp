#include "python_object_manager.h"


namespace plot
{

PythonObjectManager::PythonObjectManager()
{
  // Initialize Python
  Py_Initialize();

  // Create empty tuple for convenience.
  empty_tuple_ = PyTuple_New(0);
}

PythonObjectManager::~PythonObjectManager()
{
  Py_DECREF(empty_tuple_);

  for (const auto& funcs : py_functions_)
  {
    Py_DECREF(funcs.second);
  }

  for (const auto& mods : py_modules_)
  {
    Py_DECREF(mods.second);
  }

  Py_Finalize();
}

void PythonObjectManager::addModule(const std::string& module_name)
{
  auto py_module_name = PyString_FromString(module_name.c_str());
  if (!py_module_name)
  { throw std::runtime_error("Could not create PyString"); }

  auto module = PyImport_Import(py_module_name);
  Py_DECREF(py_module_name);

  if (!module)
  { throw std::runtime_error("Could not load module: " + module_name); }

  if (py_modules_.count(module_name) > 0)
  {
    Py_DECREF(py_modules_[module_name]);
  }

  py_modules_[module_name] = module;
}

void PythonObjectManager::addFunction(const std::string& module_name, const std::string& func_name)
{
  auto py_function = PyObject_GetAttrString(getModule(module_name), func_name.c_str());
  if (!py_function)
  { throw std::runtime_error("Could not find function: " + func_name); }
  if (!PyFunction_Check(py_function))
  { throw std::runtime_error("Python object is not a function: " + func_name); }

  std::string module_func_name = module_name + "." + func_name;
  if (py_functions_.count(module_func_name) > 0)
  {
    Py_DECREF(py_functions_[module_func_name]);
  }

  py_functions_[module_func_name] = py_function;
}

PyObject* PythonObjectManager::emptyTuple()
{
  return empty_tuple_;
}

PyObject* PythonObjectManager::getModule(const std::string& module_name)
{
  if (py_modules_.count(module_name) == 0)
  {
    throw std::out_of_range("Module \"" + module_name + "\" has not been loaded");
  }

  return py_modules_[module_name];
}

PyObject* PythonObjectManager::getFunction(const std::string& module_name, const std::string& func_name)
{
  return getFunction(module_name + "." + func_name);
}

PyObject* PythonObjectManager::getFunction(const std::string& module_func_name)
{
  if (py_functions_.count(module_func_name) == 0)
  {
    throw std::out_of_range("Function \"" + module_func_name + "\" has not been loaded");
  }

  return py_functions_[module_func_name];
}

}
