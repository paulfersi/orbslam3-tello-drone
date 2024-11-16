#!/bin/bash

rm -rf install/
rm -rf build/

colcon build

source install/setup.sh: