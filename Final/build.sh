#!/bin/bash
cp PID/src/pid.cpp ~/cvdrone/build/unix
cp PID/src/pid.hpp ~/cvdrone/build/unix
# cp PID/build/pid.yaml ~/cvdrone/build/unix
cp PID/build/makefile ~/cvdrone/build/unix/makefile
cp main.cpp ~/cvdrone/src
cp haarcascade_frontalface_alt.xml ~/cvdrone/build/unix
cp haarcascade_frontalface_alt2.xml ~/cvdrone/build/unix
cd ~
cd cvdrone/build/unix
make -j8
./test.a config.xml
