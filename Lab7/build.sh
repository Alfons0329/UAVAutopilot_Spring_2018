#!/bin/bash
g++ -std=c++11 prob2.cpp `pkg-config --cflags --libs opencv`
./a.out 1.jpg
./a.out 2.jpg
./a.out 3.jpg    
