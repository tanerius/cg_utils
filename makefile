# Declare a variable for the compliler which we are going to use
CXX=g++
# Declare another variable with the flags which we are going to use
CXXFLAGS=-g -Wall -std=c++11 

OBJDIR=build

default: testing

testing: clean init cg_utils.o
	$(CXX) $(CXXFLAGS) tests.cpp -o $(OBJDIR)/tests $(OBJDIR)/cg_utils.o

cg_utils.o: cg_utils.hpp
	$(CXX) $(CXXFLAGS) -c cg_utils.cpp -o $(OBJDIR)/cg_utils.o

init: 
	- mkdir -p $(OBJDIR)

clean:
	- rm -rf $(OBJDIR)