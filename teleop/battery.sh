#!/bin/bash

ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'battery?'}"