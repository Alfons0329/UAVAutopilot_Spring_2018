#!/bin/bash
g++ -std=c++11 bgr2rgb_template.cpp `pkg-config --cflags --libs opencv`
./a.out kobe.jpg
