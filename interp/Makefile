CXX=g++
CXXFLAGS=-std=c++14 -O -Wall -g -ggdb
INCLUDE=-I../program_option
LDFLAGS=-lgmp -lgmpxx

TRAJEDY_INCLUDE=$(INCLUDE)
TRAJEDY_LDFLAGS=$(LDFLAGS)

TRAJEDEBUG_PKG_DEPS=gtkmm-3.0 cairomm-1.0
# These are recursive variables. Hence we will only check
# pkg-config if we're trying to build the graphical binaries.
TRAJEDEBUG_INCLUDE=$(INCLUDE) $(or $(shell pkg-config --cflags $(TRAJEDEBUG_PKG_DEPS)),$(error pkg-config lookup failed))
TRAJEDEBUG_LDFLAGS=$(LDFLAGS) $(or $(shell pkg-config --libs $(TRAJEDEBUG_PKG_DEPS)),$(error pkg-config lookup failed))

all: trajedy trajedebug

trajedy.o: trajedy.cpp v2.hpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(TRAJEDY_INCLUDE)

trajedy: trajedy.o interp.o
	$(CXX) $^ -o $@ $(TRAJEDY_LDFLAGS)

interp.o: interp.cpp v2.hpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(INCLUDE)

trajedebug.o: trajedebug.cpp v2.hpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(TRAJEDEBUG_INCLUDE)

trajedebug: trajedebug.o interp.o
	$(CXX) $^ -o $@ $(TRAJEDEBUG_LDFLAGS)

clean:
	rm -f trajedy trajedy.o interp.o trajedebug trajedebug.o

.PHONY: all clean
