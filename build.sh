#!/bin/sh

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                                           #
#                     KeypointTrackerCpp Build Script                       #
#                                                                           #
# Uses pkg-config to find appropriate directory for dependencies and calls  #
# g++ accordingly.  This should eventually be replaced with a Makefile.     #
#                                                                           #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                                           #
# Copyright (c) 2014-2015, Brandon "Breakthrough" Castellano.               #
#                                                                           #
# KeypointTrackerCpp is licensed under the BSD 2-Clause License;            #
# see the included LICENSE file, or visit the following URL for details:    #
# < https://github.com/Breakthrough/KeypointTrackerCpp>                     #
#                                                                           #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

rm main
g++ -ggdb `pkg-config --cflags opencv` -o main main.cpp `pkg-config --libs opencv`;
