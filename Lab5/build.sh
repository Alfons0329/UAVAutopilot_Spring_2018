#!/bin/bash
cp main.cpp ~/cvdrone/src
cd ~
cd cvdrone/build/unix
make -j8
./test.a down_camera.xml
