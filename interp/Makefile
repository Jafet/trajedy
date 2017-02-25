CXXFLAGS=-std=c++14 -O -Wall -g -ggdb
INCLUDE=-I../program_option
LDFLAGS=-lgmp -lgmpxx
CXX=g++

all: trajedy

trajedy.o: trajedy.cpp
	$(CXX) $< -c -o $@ $(CXXFLAGS) $(INCLUDE)

trajedy: trajedy.o
	$(CXX) $< -o $@ $(LDFLAGS)

clean:
	rm trajedy trajedy.o

.PHONY: all clean
