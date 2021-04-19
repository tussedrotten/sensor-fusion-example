#include "plot.h"


namespace plot
{

Plot::Plot()
{
  // Load modules.
  python_.addModule("matplotlib");
  python_.addModule("matplotlib.pyplot");
  python_.addModule("matplotlib.style");
  python_.addModule("pylab");

  // Load functions.
  python_.addFunction("matplotlib.pyplot", "annotate");
  python_.addFunction("matplotlib.pyplot", "axis");
  python_.addFunction("matplotlib.pyplot", "clf");
  python_.addFunction("matplotlib.pyplot", "close");
  python_.addFunction("matplotlib.pyplot", "draw");
  python_.addFunction("matplotlib.pyplot", "figure");
  python_.addFunction("matplotlib.pyplot", "grid");
  python_.addFunction("matplotlib.pyplot", "hist");
  python_.addFunction("matplotlib.pyplot", "ion");
  python_.addFunction("matplotlib.pyplot", "ioff");
  python_.addFunction("matplotlib.pyplot", "legend");
  python_.addFunction("matplotlib.pyplot", "pause");
  python_.addFunction("matplotlib.pyplot", "plot");
  python_.addFunction("matplotlib.pyplot", "fill_between");
  python_.addFunction("matplotlib.pyplot", "quiver");
  python_.addFunction("matplotlib.pyplot", "show");
  python_.addFunction("matplotlib.pyplot", "subplot");
  python_.addFunction("matplotlib.pyplot", "tight_layout");
  python_.addFunction("matplotlib.pyplot", "title");
  python_.addFunction("matplotlib.pyplot", "xkcd");
  python_.addFunction("matplotlib.pyplot", "xlabel");
  python_.addFunction("matplotlib.pyplot", "xlim");
  python_.addFunction("matplotlib.pyplot", "ylabel");
  python_.addFunction("matplotlib.pyplot", "ylim");

  python_.addFunction("matplotlib.style", "use");

  python_.addFunction("pylab", "savefig");
}

Plot& Plot::instance()
{
  static Plot the_plot;
  return the_plot;
}

void Plot::annotate(const std::string& text, double x, double y)
{
  PyObject* xy = PyTuple_New(2);
  PyObject* str = PyString_FromString(text.c_str());

  PyTuple_SetItem(xy, 0, PyFloat_FromDouble(x));
  PyTuple_SetItem(xy, 1, PyFloat_FromDouble(y));

  PyObject* kwargs = PyDict_New();
  PyDict_SetItemString(kwargs, "xy", xy);

  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, str);

  PyObject* res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.annotate"), args, kwargs);
  if (!res)
  { throw std::runtime_error("Call to annotate() failed"); }

  Py_DECREF(args);
  Py_DECREF(kwargs);
  Py_DECREF(res);
}

void Plot::axis(const std::string& axis_string)
{
  PyObject* str = PyString_FromString(axis_string.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, str);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.axis"), args);
  if (!res)
  { throw std::runtime_error("Call to axis() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::clf()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.clf"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to clf() failed"); }

  Py_DECREF(res);
}

void Plot::close()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.close"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to close() failed"); }

  Py_DECREF(res);
}

void Plot::draw()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.draw"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to draw() failed"); }

  Py_DECREF(res);
}

void Plot::figure()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.figure"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to figure() failed"); }

  Py_DECREF(res);
}

void Plot::figure(
  const size_t num,
  const double width,
  const double height,
  const double dpi
)
{
  PyObject* pynum = PyInt_FromSize_t(num);
  Py_INCREF(pynum);

  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pynum);

  PyObject* kwargs = PyDict_New();

  {
    PyObject* fig_size = PyTuple_New(2);

    PyTuple_SetItem(fig_size, 0, PyFloat_FromDouble(width));
    PyTuple_SetItem(fig_size, 1, PyFloat_FromDouble(height));

    PyDict_SetItemString(kwargs, "figsize", fig_size);
  }

  PyDict_SetItemString(kwargs, "dpi", PyFloat_FromDouble(dpi));

  PyObject* res = PyObject_Call(
    instance().python_.getFunction("matplotlib.pyplot.figure"),
    args,
    kwargs
  );

  if (!res)
  { throw std::runtime_error("Call to figure() failed"); }

  Py_DECREF(res);
  Py_DECREF(args);
  Py_DECREF(kwargs);
}

void Plot::grid(bool show_grid)
{
  PyObject* pyflag = show_grid ? Py_True : Py_False;
  Py_INCREF(pyflag);

  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pyflag);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.grid"), args);
  if (!res)
  { throw std::runtime_error("Call to grid() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::ioff()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.ioff"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to ioff() failed"); }

  Py_DECREF(res);
}

void Plot::ion()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.ion"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to ion() failed"); }

  Py_DECREF(res);
}

void Plot::legend()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.legend"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to legend() failed"); }

  Py_DECREF(res);
}

void Plot::pause(double seconds)
{
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, PyFloat_FromDouble(seconds));

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.pause"), args);
  Py_DECREF(args);

  if (!res)
  { throw std::runtime_error("Call to pause() failed"); }

  Py_DECREF(res);
}

void Plot::savefig(const std::string& filename)
{
  PyObject* pyfilename = PyString_FromString(filename.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pyfilename);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("pylab.savefig"), args);
  if (!res)
  { throw std::runtime_error("Call to savefig() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::show(bool block)
{
  PyObject* res;

  if (block)
  {
    res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.show"),
                              instance().python_.emptyTuple());
  } else
  {
    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "block", Py_False);
    res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.show"), instance().python_.emptyTuple(),
                        kwargs);
    Py_DECREF(kwargs);
  }

  if (!res)
  { throw std::runtime_error("Call to show() failed"); }

  Py_DECREF(res);
}

void Plot::styleUse(const std::string& style)
{
  PyObject* pystring = PyString_FromString(style.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pystring);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.style.use"), args);
  Py_DECREF(args);

  if (!res)
  { throw std::runtime_error("Call to matplotlib.style.use() failed"); }

  Py_DECREF(res);
}

void Plot::subplot(long num_rows, long num_cols, long index)
{
  PyObject* args = PyTuple_New(3);
  PyTuple_SetItem(args, 0, PyLong_FromLong(num_rows));
  PyTuple_SetItem(args, 1, PyLong_FromLong(num_cols));
  PyTuple_SetItem(args, 2, PyLong_FromLong(index));

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.subplot"), args);
  if (!res)
  { throw std::runtime_error("Call to subplot() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::tightLayout()
{
  PyObject* res = PyObject_CallObject(
      instance().python_.getFunction("matplotlib.pyplot.tight_layout"),
      instance().python_.emptyTuple());

  if (!res)
  { throw std::runtime_error("Call to tight_layout() failed"); }

  Py_DECREF(res);
}

void Plot::title(const std::string& title)
{
  PyObject* pytitle = PyString_FromString(title.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pytitle);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.title"), args);
  if (!res)
  { throw std::runtime_error("Call to title() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::xkcd(double scale, double length, double randomness)
{
  PyObject* kwargs = PyDict_New();
  PyDict_SetItemString(kwargs, "scale", PyFloat_FromDouble(scale));
  PyDict_SetItemString(kwargs, "length", PyFloat_FromDouble(length));
  PyDict_SetItemString(kwargs, "randomness", PyFloat_FromDouble(randomness));

  PyObject* res;
  res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.xkcd"),
                      instance().python_.emptyTuple(),
                      kwargs);

  Py_DECREF(kwargs);

  if (!res)
  { throw std::runtime_error("Call to xkcd() failed."); }

  Py_DECREF(res);
}

void Plot::xlabel(const std::string& label)
{
  PyObject* pylabel = PyString_FromString(label.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pylabel);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.xlabel"), args);
  if (!res)
  { throw std::runtime_error("Call to xlabel() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::xlim(double x_min, double x_max)
{
  PyObject* list = PyList_New(2);
  PyList_SetItem(list, 0, PyFloat_FromDouble(x_min));
  PyList_SetItem(list, 1, PyFloat_FromDouble(x_max));

  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, list);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.xlim"), args);
  if (!res)
  { throw std::runtime_error("Call to xlim() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::ylabel(const std::string& label)
{
  PyObject* pylabel = PyString_FromString(label.c_str());
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, pylabel);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.ylabel"), args);
  if (!res)
  { throw std::runtime_error("Call to ylabel() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

void Plot::ylim(double y_min, double y_max)
{
  PyObject* list = PyList_New(2);
  PyList_SetItem(list, 0, PyFloat_FromDouble(y_min));
  PyList_SetItem(list, 1, PyFloat_FromDouble(y_max));

  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, list);

  PyObject* res = PyObject_CallObject(instance().python_.getFunction("matplotlib.pyplot.ylim"), args);
  if (!res)
  { throw std::runtime_error("Call to ylim() failed"); }

  Py_DECREF(args);
  Py_DECREF(res);
}

}
