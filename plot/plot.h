#pragma once

#include "python_object_manager.h"
#include "Python.h"

#include <map>
#include <numeric>
#include <vector>
#include <variant>

#if PY_MAJOR_VERSION >= 3
#  define PyString_FromString PyUnicode_FromString
#endif

#pragma once

#include <type_traits>

namespace is_callable_detail
    {
/// \brief Checks if non-class is function or class if function object.
        template<bool is_class, typename T>
        struct is_callable_impl;

/// \brief A class is callable iff it defines operator().
        template<typename T>
        struct is_callable_impl<true, T>
        {
        private:
            struct Fallback
            {
                void operator()();
            };

            struct Derived : T, Fallback
            {
            };

            template<typename U, U>
            struct Check;

            // Use a variadic function to make sure
            // (1) it accepts everything and
            // (2) it's always the worst match
            template<typename U>
            static std::true_type test(...);

            template<typename U>
            static std::false_type test(Check<void (Fallback::*)(), &U::operator()>*);

        public:
            typedef decltype(test<Derived>(nullptr)) type;
        };

/// \brief A non-class is callable iff it is a function.
        template<typename T>
        struct is_callable_impl<false, T> : std::is_function<T>
        { };
    }

/// \brief Checks if type T is callable.
template<typename T>
struct is_callable : is_callable_detail::is_callable_impl<std::is_class<T>::value, T>::type
{ };



namespace plot
{
/// \brief Provides a simplified interface to the python matplotlib API.
/// Assumes that it is the only Python interpreter. See tests for example usages.
/// This module is inspired by [matplotlib-cpp](https://github.com/lava/matplotlib-cpp).
/// \see https://matplotlib.org/
class Plot
{
public:
  using Property = std::variant<std::string, double, int>;
  using Properties = std::map<std::string, Property>;

  /// \brief Annotate the point (x, y) with the text.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.annotate.html
  static void annotate(const std::string& text, double x, double y);

  /// \brief Set axis properties.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.axis.html
  static void axis(const std::string& axis_string);

  /// \brief Clear the current figure.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.clf.html
  static void clf();

  /// \brief Close the current figure.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.close.html
  static void close();

  /// \brief Redraw the current figure.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.draw.html
  static void draw();

  /// \brief Create a new figure.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.figure.html
  static void figure();

  /// \brief Activates figure with given id. Creates new figure if does not
  /// already exist.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.figure.html
  static void figure(
    size_t num,
    double width = 8,
    double height = 6,
    double dpi = 80
  );

  /// \brief Turn the axes grids on or off.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.grid.html
  static void grid(bool show_grid);

  /// \brief Plot a histogram.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.hist.html
  template<typename Scalar>
  static void hist(const std::vector<Scalar>& y,
                   long bins = 10,
                   double alpha = 1.0,
                   const Properties& properties = {});

  /// \brief Plot a histogram with label.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.hist.html
  template<typename Scalar>
  static void histLabeled(const std::vector<Scalar>& y, long bins, double alpha, const std::string& label);

  /// \brief Turn interactive mode off.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.ioff.html
  static void ioff();

  /// \brief Turn interactive mode on.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.ion.html
  static void ion();

  /// \brief Place a legend on the axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.legend.html
  static void legend();

  /// \brief Pause for the given amount of seconds.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.pause.html
  static void pause(double seconds);

  /// \brief Plot y using x as index array [0, N-1].
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  static void plot(const std::vector<double>& y,
                   const std::string& format = "",
                   const Properties& properties = {});

  /// \brief Plot y using x as index array [0, N-1].
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename Scalar>
  static void plot(const std::vector<Scalar>& y,
                   const std::string& format = "",
                   const Properties& properties = {});

  /// \brief Plot y versus x.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  static void plot(const std::vector<double>& x,
                   const std::vector<double>& y,
                   const std::string& format = "",
                   const Properties& properties = {});

  /// \brief Plot y versus x.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename ScalarX, typename ScalarY>
  static void plot(const std::vector<ScalarX>& x,
                   const std::vector<ScalarY>& y,
                   const std::string& format = "",
                   const Properties& properties = {});

  /// \brief Plot tuples of (x, y, format_string).
  ///  Here, x is an iterable and y is either an iterable or a callable.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename A, typename B, typename... Args>
  static void plot(const A& x, const B& y, const std::string& format, Args... args);

  /// \brief Plot tuples of (x, y, format_string, properties).
  ///  Here, x is an iterable and y is either an iterable or a callable.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename A, typename B, typename... Args>
  static void plot(const A& x,
                   const B& y,
                   const std::string& format,
                   const Properties& properties,
                   Args... args);

  /// \brief Recursion stop for variadic templates above. Does nothing.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename... Args>
  static void plot();

  template<typename Scalar>
  static void fillBetween(
    const std::vector<Scalar>& y1,
    const std::vector<Scalar>& y2,
    const std::string& color
  );

  template<typename Scalar>
  static void fillBetween(
    const std::vector<Scalar>& x,
    const std::vector<Scalar>& y1,
    const std::vector<Scalar>& y2,
    const std::string& color
  );

  /// \brief Plot y using x as index array [0, N-1] with label.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename Scalar>
  static void plotLabeled(const std::vector<Scalar>& y,
                          const std::string& format,
                          const std::string& label);

  /// \brief Plot y versus x with label.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename ScalarX, typename ScalarY>
  static void plotLabeled(const std::vector<ScalarX>&,
                          const std::vector<ScalarY>&,
                          const std::string& format,
                          const std::string& label);

  /// \brief Plot y versus x where both are iterables.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.plot.htm
  template<typename IterableX, typename IterableY>
  static void plotIterables(const IterableX& x,
                            const IterableY& y,
                            const std::string& format = "",
                            const Properties& properties = {});

  /// \brief Plot arrows.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.quiver.htm
  template<typename Scalar>
  static void quiver(const std::vector<Scalar>& x,
                     const std::vector<Scalar>& y,
                     const std::vector<Scalar>& u,
                     const std::vector<Scalar>& v,
                     const std::string& color = "");

  /// \brief Plot arrows.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.quiver.htm
  static void quiver(const std::vector<double>& x,
                     const std::vector<double>& y,
                     const std::vector<double>& u,
                     const std::vector<double>& v,
                     const std::string& color = "");

  /// \brief Save figure to image file.
  static void savefig(const std::string& filename);

  /// \brief Display a figure with the specified blocking behavior.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.show.html
  static void show(bool block = true);

  /// \brief Use matplotlib style settings from a style specification.
  /// \see https://matplotlib.org/api/style_api.html#matplotlib.style.use
  static void styleUse(const std::string& style);

  /// \brief Create a subplot axis at the given grid position.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.subplot.html
  static void subplot(long num_rows, long num_cols, long index);

  /// \brief Automatically adjust subplot parameters for padding.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.tight_layout.html
  static void tightLayout();

  /// \brief Set a title for the current axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.title.html
  static void title(const std::string& title);

  /// \brief Turns on xkcd drawing mode.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.xkcd.html
  static void xkcd(double scale = 1.0, double length = 100.0, double randomness = 2.0);

  /// \brief Set the x-axis label of the current axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.xlabel.html
  static void xlabel(const std::string& label);

  /// \brief Set the x-limits of the current axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.xlim.html
  static void xlim(double x_min, double x_max);

  /// \brief Set the y-axis label of the current axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.ylabel.html
  static void ylabel(const std::string& label);

  /// \brief Set the y-limits of the current axes.
  /// \see https://matplotlib.org/api/_as_gen/matplotlib.pyplot.ylim.html
  static void ylim(double y_min, double y_max);

  Plot(const Plot&) = delete;

  Plot& operator=(const Plot&) = delete;

  Plot(Plot&&) = delete;

  Plot& operator=(Plot&&) = delete;

private:
  Plot();

  static Plot& instance();

  ~Plot() = default;

  PythonObjectManager python_;
};


// ----------- Implementation -----------
namespace plot_detail
{
using Property = Plot::Property;
using Properties = Plot::Properties;

PyObject* asPyObject(double val);

PyObject* asPyObject(int val);

PyObject* asPyObject(const std::string& str);

PyObject* asPyObject(const Property& property);
}

template<typename IterableX, typename IterableY>
inline void Plot::plotIterables(const IterableX& x,
                                const IterableY& y,
                                const std::string& format,
                                const Properties& properties)
{
  auto x_length = std::distance(std::begin(x), std::end(x));
  auto y_length = std::distance(std::begin(y), std::end(y));
  if (x_length != y_length)
  { throw std::runtime_error("x and y must have the same number of elements"); }

  PyObject* xlist = PyList_New(x_length);
  PyObject* ylist = PyList_New(y_length);
  PyObject* pystring = PyString_FromString(format.c_str());

  auto itx = std::begin(x);
  auto ity = std::begin(y);
  for (long i = 0; i < x_length; ++i)
  {
    PyList_SetItem(xlist, i, PyFloat_FromDouble(*itx++));
    PyList_SetItem(ylist, i, PyFloat_FromDouble(*ity++));
  }

  // construct properties args
  PyObject* kwargs = PyDict_New();
  for (const auto& props : properties)
  {
    PyDict_SetItemString(kwargs, props.first.c_str(), plot_detail::asPyObject(props.second));
  }

  PyObject* plot_args = PyTuple_New(3);
  PyTuple_SetItem(plot_args, 0, xlist);
  PyTuple_SetItem(plot_args, 1, ylist);
  PyTuple_SetItem(plot_args, 2, pystring);

  PyObject* res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.plot"), plot_args, kwargs);

  Py_DECREF(plot_args);
  Py_DECREF(kwargs);
  Py_XDECREF(res);
}

namespace plot_detail
{
template<typename IsYDataCallable>
struct plot_impl
{
};

template<>
struct plot_impl<std::false_type>
{
  template<typename IterableX, typename IterableY>
  void operator()(const IterableX& x, const IterableY& y, const std::string& format,
                  const Properties& properties = {})
  {
    Plot::plotIterables(x, y, format, properties);
  }
};

template<>
struct plot_impl<std::true_type>
{
  template<typename Iterable, typename Callable>
  void operator()(const Iterable& ticks, const Callable& f, const std::string& format,
                  const Properties& properties = {})
  {
    if (std::begin(ticks) == std::end(ticks)) return;

    std::vector<double> y;
    for (auto x : ticks) y.push_back(f(x));

    Plot::plotIterables(ticks, y, format, properties);
  }
};
}

template<typename Scalar>
inline void
Plot::hist(const std::vector<Scalar>& y, long bins, double alpha, const Properties& properties)
{
  PyObject* ylist = PyList_New(y.size());
  for (size_t i = 0; i < y.size(); ++i)
  {
    PyList_SetItem(ylist, i, PyFloat_FromDouble(y[i]));
  }

  // construct properties args
  PyObject* kwargs = PyDict_New();
  PyDict_SetItemString(kwargs, "bins", PyLong_FromLong(bins));
  PyDict_SetItemString(kwargs, "alpha", PyFloat_FromDouble(alpha));
  for (const auto& props : properties)
  {
    PyDict_SetItemString(kwargs, props.first.c_str(), plot_detail::asPyObject(props.second));
  }

  PyObject* plot_args = PyTuple_New(1);
  PyTuple_SetItem(plot_args, 0, ylist);

  PyObject* res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.hist"), plot_args, kwargs);

  Py_DECREF(plot_args);
  Py_DECREF(kwargs);
  Py_XDECREF(res);
}

template<typename Scalar>
inline void Plot::histLabeled(const std::vector<Scalar>& y, long bins, double alpha, const std::string& label)
{
  hist(y, bins, alpha, {{"label", label}});
}

// Specialization to allow initializer-list construction of vector.
inline void Plot::plot(const std::vector<double>& y,
                       const std::string& format,
                       const Properties& properties)
{
  plot<double>(y, format, properties);
}

template<typename Scalar>
inline void Plot::plot(const std::vector<Scalar>& y,
                       const std::string& format,
                       const Properties& properties)
{
  std::vector<double> x(y.size());
  std::iota(x.begin(), x.end(), 0.0);

  plot(x, y, format, properties);
}

template<typename Scalar>
inline void Plot::fillBetween(
  const std::vector<Scalar>& y1,
  const std::vector<Scalar>& y2,
  const std::string& color
)
{
  std::vector<double> x(y1.size());
  std::iota(x.begin(), x.end(), 0.0);

  fillBetween(x, y1, y2, color);
}

template<typename Scalar>
inline void Plot::fillBetween(
  const std::vector<Scalar>& x,
  const std::vector<Scalar>& y1,
  const std::vector<Scalar>& y2,
  const std::string& color
)
{
  const auto length = y1.size();

  PyObject* xlist = PyList_New(length);
  PyObject* y1list = PyList_New(length);
  PyObject* y2list = PyList_New(length);

  auto itx = std::begin(x);
  auto ity1 = std::begin(y1);
  auto ity2 = std::begin(y2);

  for (size_t i = 0; i < length; ++i)
  {
    PyList_SetItem(xlist, i, PyFloat_FromDouble(*itx++));
    PyList_SetItem(y1list, i, PyFloat_FromDouble(*ity1++));
    PyList_SetItem(y2list, i, PyFloat_FromDouble(*ity2++));
  }

  // construct properties args
  PyObject* kwargs = PyDict_New();
  PyDict_SetItemString(kwargs, "color", plot_detail::asPyObject(color));

  PyObject* plot_args = PyTuple_New(3);
  PyTuple_SetItem(plot_args, 0, xlist);
  PyTuple_SetItem(plot_args, 1, y1list);
  PyTuple_SetItem(plot_args, 2, y2list);

  PyObject* res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.fill_between"), plot_args, kwargs);

  Py_DECREF(plot_args);
  Py_DECREF(kwargs);
  Py_XDECREF(res);
}

template<typename Scalar>
inline void Plot::plotLabeled(const std::vector<Scalar>& y,
                              const std::string& format,
                              const std::string& label)
{
  std::vector<double> x(y.size());
  std::iota(x.begin(), x.end(), 0.0);

  plotLabeled(x, y, format, label);
}

// Specialization to allow initializer-list construction of vector.
inline void Plot::plot(const std::vector<double>& x,
                       const std::vector<double>& y,
                       const std::string& format,
                       const Properties& properties)
{
  plot<double, double>(x, y, format, properties);
}

template<typename ScalarX, typename ScalarY>
inline void Plot::plot(const std::vector<ScalarX>& x,
                       const std::vector<ScalarY>& y,
                       const std::string& format,
                       const Properties& properties)
{
  plotIterables(x, y, format, properties);
}

template<typename ScalarX, typename ScalarY>
inline void Plot::plotLabeled(const std::vector<ScalarX>& x,
                              const std::vector<ScalarY>& y,
                              const std::string& format,
                              const std::string& label)
{
  plotIterables(x, y, format, {{"Label", label}});
}

template<typename A, typename B, typename... Args>
inline void
Plot::plot(const A& x,
           const B& y,
           const std::string& format,
           const Properties& properties,
           Args... args)
{
  plot_detail::plot_impl<typename is_callable<B>::type>()(x, y, format, properties);
  plot(args...);
}

template<typename A, typename B, typename... Args>
inline void Plot::plot(const A& x, const B& y, const std::string& format, Args... args)
{
  plot_detail::plot_impl<typename is_callable<B>::type>()(x, y, format);
  plot(args...);
}

template<typename... Args>
inline void Plot::plot()
{}

template<typename Scalar>
inline void Plot::quiver(const std::vector<Scalar>& x,
                         const std::vector<Scalar>& y,
                         const std::vector<Scalar>& u,
                         const std::vector<Scalar>& v,
                         const std::string& color)
{
  const auto num_elem = x.size();

  if (num_elem != y.size() || num_elem != u.size() || num_elem != v.size())
  { throw std::runtime_error("x, y, u and v must have the same number of elements"); }

  PyObject* xlist = PyList_New(num_elem);
  PyObject* ylist = PyList_New(num_elem);
  PyObject* ulist = PyList_New(num_elem);
  PyObject* vlist = PyList_New(num_elem);

  for (size_t i = 0; i < num_elem; ++i)
  {
    PyList_SetItem(xlist, i, PyFloat_FromDouble(x[i]));
    PyList_SetItem(ylist, i, PyFloat_FromDouble(y[i]));
    PyList_SetItem(ulist, i, PyFloat_FromDouble(u[i]));
    PyList_SetItem(vlist, i, PyFloat_FromDouble(v[i]));
  }

  PyObject* plot_args = PyTuple_New(4);
  PyTuple_SetItem(plot_args, 0, xlist);
  PyTuple_SetItem(plot_args, 1, ylist);
  PyTuple_SetItem(plot_args, 2, ulist);
  PyTuple_SetItem(plot_args, 3, vlist);

  PyObject* kwargs = PyDict_New();
  if (!color.empty())
  {
    PyDict_SetItemString(kwargs, "color", PyString_FromString(color.c_str()));
  }

  PyObject* res = PyObject_Call(instance().python_.getFunction("matplotlib.pyplot.quiver"), plot_args, kwargs);

  Py_DECREF(plot_args);
  Py_DECREF(kwargs);
  Py_XDECREF(res);
}

// Specialization to allow initializer-list construction of vector.
inline void Plot::quiver(const std::vector<double>& x,
                         const std::vector<double>& y,
                         const std::vector<double>& u,
                         const std::vector<double>& v,
                         const std::string& color)
{
  Plot::quiver<double>(x, y, u, v, color);
}

namespace plot_detail
{
inline PyObject* asPyObject(const double val)
{
  return PyFloat_FromDouble(val);
}

inline PyObject* asPyObject(const int val)
{
  return PyInt_FromLong(val);
}

inline PyObject* asPyObject(const std::string& str)
{
  return PyString_FromString(str.c_str());
}

inline PyObject* asPyObject(const Property& property)
{
  return std::visit(
    [](const auto& val)
    {
      return asPyObject(val);
    },
    property
  );
}
}
}

