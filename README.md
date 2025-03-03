Uses OpenCV to threshold, track lines, and publish command velocities to ROS TurtleSim. 
Upon running "line_tracker.launch" for the first time, TurtleSim will open as well as the color thresholding calibration.
During calibration you will be prompted to threshold for each color listed (you can swap between an HSV or BGR color space by pressing 's'). Press space-bar after you are done thresholding for each color.
Once calibration is complete, you will be prombeted to select a color to track.
(If this package has been run before and calibration was completed, you will be given the option to use the previous thresholding data and skip calibration or calibrate again.)
After the desired color has been selected, "ROS_line_tracker.py" will calculate and publish linear and angular command velocities to the "/cmd_vel" ROS topic.
"turtle_sub.py" will subscribe to the "/cmd_vel" topic and forawrd the data to TurtleSim.

While program is running, you can change the color you want tracked by pressing 'c' (must click on "line_tracking" window first) then enter the new color into the prompt on the terminal.

If you want to calibrate more colors, add them into line 160 then edit line 189 and 232 accordingly.
