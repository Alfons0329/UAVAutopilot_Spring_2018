#!/bin/bash
cp main.cpp ~/cvdrone/src
cd ~
cd cvdrone/build/unix
make
./test.a
