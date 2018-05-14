#include <boost/thread.hpp> //threading
#include <boost/chrono.hpp> //?
#include <boost/asio.hpp> //serial interface 
#include <boost/python.hpp>
#include <boost/algorithm/string.hpp>
#include <string>
#include <iostream>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;
typedef boost::python::list pylist;

serial_port_ptr _port;
std::string devicePath;
boost::asio::io_service _io;

int dbg;

/*
std::string* get_lines(int lines)
{
  std::string* lines = malloc(lines * sizeof(char));
  for (int i=0; i<lines; i++)
  {
    std::istream is(
    std::getline(
  }
}
*/



void initialize(std::string dpath)
{
  std::cout << "Device path: " << dpath << "\n" << std::flush; 
  devicePath = dpath;
  _port = serial_port_ptr(new boost::asio::serial_port(_io));
  _port->open(devicePath);
}

int read_num(char delim)
{
  char buffer[20];
  int size=0;
  do
  {
    boost::asio::read(*_port,boost::asio::buffer(&buffer[size],1));
    size++;  
  } while (buffer[size-1] != delim);
  //buffer should contain a reversed number from index 0 to size-2
  int number = buffer[0] - '0';
  for (int i=1; i<size-1; i++)
  {
    number *= 10;
    number += (buffer[i] - '0');
  }
  return number;
}

boost::python::object readMotorCommands()
{
  if(dbg)
  {
    std::cout << "readMotorCommands: called\n" << std::flush;
  }
  char byte_array[165];
  std::string command = "getmotors\r";
  boost::asio::write(*_port,boost::asio::buffer(command.c_str(),command.size())); 
  usleep(2000); //TODO
  boost::asio::streambuf response;
  
  if(dbg)
  {
 //   std::cout << "readMotorCommands: looking for 'LeftWheel_Encoder,'\n" << std::flush;
/*    int a = boost::asio::read_until(*_port, response, ",");
    std::cout << "finished internal test\n" << std::flush;
    std::string s( (std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>() );
    std::cout << "$" << s << "$" << "\nreturned value: " << a << "\n" << std::flush;
    a = boost::asio::read_until(*_port, response, ",");
    s = std::string( (std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>() );
    std::cout << "$" << s << "$" << "\nreturned value: " << a << "\n" << std::flush;
  */   
  }
/*  boost::asio::read_until(*_port, response, "LeftWheel_Encoder,"); 
  int left = read_num('\n');
  if(dbg)
  {
    std::cout << "readMotorCommands: looking for 'RightWheel_Encoder,'\n" << std::flush;
  }
  boost::asio::read_until(*_port, response, "RightWheel_Encoder,");
  int right = read_num('\n');
  if(dbg)
  {
    std::cout << "readMotorCommands: looking for 'Charger_mAH, '\n" << std::flush;
  }
  */
  boost::asio::read_until(*_port, response, "Charger_mAH, ");
  if(dbg)
  {
    std::cout << "readMotorCommands: Got all, trying to flush buffer\n" << std::flush;
  }
//  _port->read_some(boost::asio::buffer(byte_array, 164));
  pylist list;
  //list.append(left);
//  list.append(right);
  if (dbg)
  {
    int a;
    std::cout << "readMotorCommands: exiting\n" << std::flush;

//    std::cout << "$" << s << "$" << "\nreturned value: " << a << "\n" << std::flush;
  }    
  std::string s( (std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>() );
  std::string::size_type i = s.find("getmotor");
  if (i != std::string::npos)
  {
    s.erase(0,i);
  }
/*  std::string torem = "\n\n";
  std::string::size_type i = s.find(torem);
  if (i != std::string::npos)
  {
    s.erase(i, torem.length());
  }*/

  std::vector<std::string> strs;
  boost::split(strs,s,boost::is_any_of("\n"));
  // LeftWheel_Encoder,<num> line = 15
  // RightWheel_Encoder,<num> line = 21
  if (dbg)
  {
    std::cout << "left line: " << strs[15] << "\n" << std::flush;
    std::cout << "right line: " << strs[21] <<"\n" <<  std::flush;
  }
  std::string leftl = strs[15].substr(18);
  std::string rightl = strs[21].substr(19);
  int left = std::stoi(leftl);
  int right = std::stoi(rightl);
  if (dbg)
  {
    std::cout << "left number: " << left << "\n" << std::flush;
    std::cout << "right number: " << right << "\n" << std::flush;
  }
  list.append(left); list.append(right);
  return list; 
}

void set_debug(int debug)
{
  dbg = debug;
}


boost::python::object readLidarData()
{
  //   serial_port_ptr _port;
  //   std::string devicePath  = "/dev/ttyACM1";
  //   boost::asio::io_service _io;

  //   _port = serial_port_ptr(new boost::asio::serial_port(_io));
  // _port->open(devicePath);
  char byte_array [501];
  //lock lidar_scan
  std::string command = "\ngetLDSScan\r";
  char c;
  std::string output =  "";
  boost::asio::write(*_port,boost::asio::buffer(command.c_str(),command.size()));
  usleep(2000);
  //_port->read_some(boost::asio::buffer(byte_array, 20));//flush input string
  int a = 0;
  /*  while (a > 10)
      {
      boost::asio::read(*_port,boost::asio::buffer(&c,1));
      if (a == 0 && c == 'g') a++;
      else if (a == 1 && c == 'e') a++;
      else if (a == 2 && c == 't') a++;
      else if (a == 3 && c == 'L') a++;
      else if (a == 4 && c == 'D') a++;
      else if (a == 5 && c == 'S') a++;
      else if (a == 6 && c == 'S') a++;
      else if (a == 7 && c == 'c') a++;
      else if (a == 8 && c == 'a') a++;
      else if (a == 9 && c == 'n') a++;
      else if (a == 10 && c == '\r') a++;
      else a = 0;
      }
   */

  //_port->read_some(boost::asio::buffer(byte_array, 48));//flush column titles
  for (int i=0; i<60; i++)
  {
    boost::asio::read(*_port,boost::asio::buffer(&c,1));
    byte_array[i] = c;
  }
  byte_array[60] = '\0';
  std::cout << "Sh: " << byte_array<< "\n";


  pylist outer_list;
  pylist inner_lists[360];
  for(int x = 0; x < 360; ++x)
  {
    while(c != ',')
    {
      boost::asio::read(*_port,boost::asio::buffer(&c,1));//flush characters until first ','
    }

    //std::string dist = "";
    char buffer[20];
    int size=0;
    do
    {
      boost::asio::read(*_port,boost::asio::buffer(&buffer[size],1));
      size++;  
    } while (buffer[size-1] != ',');
    //buffer should contain a reversed number from index 0 to size-2
    int number = buffer[0] - '0';
    for (int i=1; i<size-1; i++)
    {
      number *= 10;
      number += (buffer[i] - '0');
    }
    inner_lists[x].append(number);

    size = 0;
    do
    {
      boost::asio::read(*_port,boost::asio::buffer(&buffer[size],1));
      size++;
    } while (buffer[size-1] != ',');
    number = buffer[0] - '0';
    for (int i=1; i<size-1; i++)
    {
      number *= 10;
      number += (buffer[i] - '0');
    }
    inner_lists[x].append(number);
    outer_list.append(inner_lists[x]);

    while(c != '\n')  // || c != '\r')
    {
      boost::asio::read(*_port,boost::asio::buffer(&c,1));
    }
    // output += "dist: " + dist + " intens: "  + intens + "\n";
  } 
  //   _port->close();   
  _port->read_some(boost::asio::buffer(byte_array, 164));//flush input string
  /*  while(_port->read_some(boost::asio::buffer(byte_array, 16)) != 0)
      {
      std::cout << "yo im stuck " << std::flush;
      }*/
  /*    while (boost::asio::read(*_port,boost::asio::buffer(&c,1)) != 0)
	{
	std::cout << "." << std::flush;
	}*/
  return outer_list;

}


void dispose()
{
  _port->close();
}

BOOST_PYTHON_MODULE(neato_cpp)
{
  boost::python::def("readLidarData", readLidarData);
  boost::python::def("readMotorCommands", readMotorCommands);
  boost::python::def("set_debug", set_debug);
  boost::python::def("init", initialize);
  boost::python::def("dispose", dispose);
}
