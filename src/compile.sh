#!/usr/bin/env bash

set -eu

g++ --std=c++17 Capture.cc -o capture `pkg-config --cflags realsense2` `pkg-config --libs realsense2` -lygor

