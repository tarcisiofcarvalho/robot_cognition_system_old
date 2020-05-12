#!/bin/bash

clear

# Open catkin source
cd ../../

# Cleaning up the current build data
rm -rf build/robot_cognition_system

# Creating make files
catkin_make robot_cognition_system

# Installing make files
catkin_make install robot_cognition_system