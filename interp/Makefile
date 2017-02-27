CXX=g++
CXXFLAGS=-std=c++14 -O -Wall -g -ggdb
INCLUDE=-I../program_option
LDFLAGS=-lgmp -lgmpxx

TRAJEDY_INCLUDE=$(INCLUDE)
TRAJEDY_LDFLAGS=$(LDFLAGS)

TRAJEDEBUG_PKG_DEPS=gtkmm-3.0 cairomm-1.0
TRAJEDEBUG_INCLUDE=$(INCLUDE) $(shell pkg-config --cflags $(TRAJEDEBUG_PKG_DEPS))
TRAJEDEBUG_LDFLAGS=$(LDFLAGS) $(shell pkg-config --libs $(TRAJEDEBUG_PKG_DEPS))

all: trajedy trajedebug

trajedy.o: trajedy.cpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(TRAJEDY_INCLUDE)

trajedy: trajedy.o
	$(CXX) $< -o $@ $(TRAJEDY_LDFLAGS)

trajedebug.o: trajedebug.cpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(TRAJEDEBUG_INCLUDE)

trajedebug: trajedebug.o
	$(CXX) $< -o $@ $(TRAJEDEBUG_LDFLAGS)

clean:
	rm -f trajedy trajedy.o trajedebug trajedebug.o

.PHONY: all clean
