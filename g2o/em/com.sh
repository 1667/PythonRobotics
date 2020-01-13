#!/bin/sh
g++ em-g2o.cpp -I /usr/include/eigen3/ `pkg-config --cflags --libs opencv` -lg2o_core -lg2o_stuff
