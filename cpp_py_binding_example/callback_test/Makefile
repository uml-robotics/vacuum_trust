# location of the Python header files
 
PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
 
# location of the Boost Python include files and library
 
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib/x86_64-linux-gnu
PROJ = callback_test
 
$(PROJ).so: $(PROJ).o
	g++ $(PROJ).o -shared -Wl,--export-dynamic -L$(BOOST_LIB) -lboost_python-py27 -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION) -o $(PROJ).so

$(PROJ).o: $(PROJ).cpp
	g++ -c -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -fPIC $(PROJ).cpp

clean:
	rm $(PROJ).o $(PROJ).so
