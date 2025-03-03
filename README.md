# LineTracker
![Python](https://img.shields.io/badge/Python-3.x-blue) 
![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)

A ROS node that uses OpenCV to detect and track a line of a decided color and contols a TurtleSim with published command velocities.

## Overview
This ROS node:
1. Thresholds color ranges set by the user to enable precise detection of differntly colored lines.
2. Calculates command velocities based on the live video feed.
3. Publishes calcutated command velocities to a TurtleSim subscriber.
4. Forwards the velocities to the TurtleSim robot to control it.
5. Allows for easy recalibration and interchanging of tracked colors.

## Directions
1. Put the files into a catkin workspace package.
2. Launch the launch file.
3. Calibrate the color ranges using the thresholding script in your desired color space (HSV/BGR).
4. Enter desired color to track into the terminal prompt.
5. Watch the turtle go.

**Extras**
- After the script has been run once, you can skip the calibration step and select a color range from the previous data.
- Pres 'c' on the "Line Tracking" window and select a different color in the terminal prompt to change the color being tracked without restarting the program. 
