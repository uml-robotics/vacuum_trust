#include <boost/python.hpp>

boost::python::object f_callback;

void set_callback(boost::python::object f)
{
  f_callback = f;
}

void call_f(int a, int b)  //note might cause an issue w/ threading 
{
  if (f_callback)
    f_callback(a, b);
  else
    return; // can have some error here
}

BOOST_PYTHON_MODULE(callback_test)
{
    boost::python::def("set_callback", set_callback);
    boost::python::def("call_f", call_f);
}
