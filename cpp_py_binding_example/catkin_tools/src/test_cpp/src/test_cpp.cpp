#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>


bool destruct;
int count = 0;

boost::thread a;

bool m_pause; // initialise to false in constructor!
boost::mutex m_pause_mutex;
boost::condition_variable m_pause_changed;

void block_while_paused()
{
    boost::unique_lock<boost::mutex> lock(m_pause_mutex);
    while(m_pause)
    {
        m_pause_changed.wait(lock);
    }
}

void set_paused(bool new_value)
{
    {
        boost::unique_lock<boost::mutex> lock(m_pause_mutex);
        m_pause = new_value;
    }

    m_pause_changed.notify_all();
}


void wait(int seconds)
{
  boost::this_thread::sleep_for(boost::chrono::seconds(seconds));
}

void c_thread()
{
  while (!destruct)
  {
    wait(1);
    count++;
    block_while_paused();
  }
}


void pauseff()
{
  set_paused(true);
}

void resumef()
{
  set_paused(false);
}

void initialize()
{
  resumef();
  destruct = false;
  a = boost::thread(c_thread);
}

void dispose()
{
  destruct = true; //thread should exist on next loop
  resumef(); //need to unblock the lock to terminate thread properly
  a.join(); //waits til the thread actually exits
}
int get_count()
{
  return count;
}


char const* greet()
{
   return "hello, world";
}

#include <boost/python.hpp>
 
BOOST_PYTHON_MODULE(test_cpp)
{
    boost::python::def("greet", greet);
    boost::python::def("initialize", initialize);
    boost::python::def("pause", pauseff);
    boost::python::def("resume", resumef);
    boost::python::def("get_count", get_count);
    boost::python::def("dispose", dispose);
}
