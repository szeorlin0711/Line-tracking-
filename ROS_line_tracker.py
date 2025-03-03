#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rft_hsvrgb as rf  # Import your custom range_finder module for trackbar-based HSV calibration
import imutils as imut
import os
import rospy
from geometry_msgs.msg import Twist

CALIBRATION_FILE = "calibration_data.npy"

def save_calibration_data(color_ranges, color_space='HSV'):
    """save the calibrated color ranges to a file"""
    data = {'color_space': color_space, 'ranges': color_ranges}
    np.save(CALIBRATION_FILE, data)
    print(f"Calibration data saved in {color_space} space.")

def load_calibration_data():
    """load color ranges from the saved calibration file."""
    try:
        data = np.load(CALIBRATION_FILE, allow_pickle=True).item()
        return data.get('ranges', {}), data.get('color_space', 'HSV') #Default to HSV
    except FileNotFoundError:
        return None, None

def process_frame(frame, lower, upper, range_filter='HSV'):
    """
    Process the given frame to detect a colored line, choose the endpoint closest to the top
    of the image, and compute control commands (linear and angular velocities) based on a vector
    drawn from the center of the screen to that endpoint.
    
    Args:
        frame: The original BGR frame.
        lower: NumPy array with lower HSV bounds.
        upper: NumPy array with upper HSV bounds.
        range_filter: The color space used for thresholding ('HSV' is expected).
    
    Returns:
        The processed frame with the detected line and the control vector drawn.
    """

    # Convert to the chosen color space.
    if range_filter.upper() == 'HSV':
        frame_to_thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    elif range_filter.upper() == 'BGR':
        frame_to_thresh = frame
    else:
        print(f"Error: Unsupported color space {range_filter}")
        return frame

    # Apply the thresholding.
    mask = cv2.inRange(frame_to_thresh, lower, upper)
    
    # (Optional) Clean up the mask with morphological operations.
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Detect edges using Canny.
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    
    # Use the probabilistic Hough Transform to detect line segments.
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                            minLineLength=50, maxLineGap=10)

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    v, w = 0, 0 #default values for velocites

    if lines is not None:
        # Select the longest line segment.
        longest_line = None
        max_length = 0
        for line in lines:
            for x1, y1, x2, y2 in line:
                length = math.hypot(x2 - x1, y2 - y1)
                if length > max_length:
                    max_length = length
                    longest_line = (x1, y1, x2, y2)
                    
        if longest_line is not None:
            x1, y1, x2, y2 = longest_line

            # Draw the detected line (green) for visualization.
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
            # Choose the endpoint that is closest to the top (i.e. smallest y value).
            if y1 < y2:
                target_point = (x1, y1)
            else:
                target_point = (x2, y2)

            tx, ty = target_point

            # Draw a circle at the selected target point.
            cv2.circle(frame, target_point, 5, (0, 0, 255), -1)

            # Draw a vector (arrow) from the center of the image to the target point.
            cv2.arrowedLine(frame, (center_x, center_y), target_point, (255, 0, 0), 2)

            # Compute the differences relative to the center.
            error_x = tx - center_x       # horizontal difference (for turning)
            error_y = center_y - ty       # vertical difference (for forward/reverse)
                                         # (Note: if ty is above center, error_y is positive)

            # Compute control commands:
            # - Linear velocity: proportional to how far the target is above the center.
            #   (If the target is below the center, this value becomes negative for reverse.)
            k_linear = 0.005  # Tune this constant as needed.
            v = k_linear * error_y

            # - Angular velocity: proportional to the horizontal offset.
            k_turn = 0.005    # Tune this constant as needed.
            w = k_turn * error_x

            print(f"Target point: {target_point}, error_x: {error_x}, error_y: {error_y}")
            print(f"Command velocities: linear = {v:.2f}, angular = {w:.2f}")

    else:
        print("No line detected.")

    return frame, v, w

def main():
    rospy.init_node("line_follower") #initialize ROS node
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #ROS publisher

    use_saved = 'n'

    # Open the video capture.
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Load saved calibration data
    color_ranges, saved_color_space = load_calibration_data()
    
    if color_ranges:
        use_saved = input("Calibration data found. Use previous calibration? (y/n): ").strip().lower()
        if use_saved == 'y':
            range_filter = saved_color_space
            print(f"Using saved calibration in {range_filter} space.")
        else:
            range_filter = input("Select color space (HSV/BGR): ").strip().upper()
    else:
        range_filter = input("No saved Calibration. Select color space (HSV/BGR): ").strip().upper()

    if range_filter not in ['HSV', 'BGR']:
        print("Invalid color space. Exiting.")
        return

    if use_saved == 'n':
        # --- CALIBRATION USING THE range_finder MODULE ---
        print(f"Calibrating in {range_filter} color space...")
        color_ranges = {}
        for color in ['red', 'blue', 'green']:
            print(f"Calibrating {color.capitalize()} color...")
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                return
            track_vals = rf.process_live_feed(cap, preview=True, imut=False, frame_width=640)  # Pass the camera (cap), not the frame
            lower = np.array(track_vals[:3])
            upper = np.array(track_vals[3:])
            color_ranges[color] = (lower, upper)  # Correctly assigning values to a dictionary
            print(f"{color.capitalize()} color range complete: lower = {lower}, upper = {upper}")

        save_calibration_data(color_ranges, color_space=range_filter)

    if not color_ranges:
        print("No calibration data available. Exiting.")
        return

    # Prompt user to select one of the color ranges
    color_names = list(color_ranges.keys())
    print("\nChoose a color range:")
    for i, color in enumerate(color_names, 1):
        lower, upper = color_ranges[color]
        print(f"{i}: {color.capitalize()} -> {lower} to {upper}")

    while True:
        try:
            selection = int(input(f"Enter the number (1 for Red, 2 for Blue, 3 for Green) of the color range to use: ").strip())
            if 1 <= selection <= len(color_names):
                selected_color = color_names[selection - 1]
                lower, upper = color_ranges[selected_color]
                print(f"Using {selected_color.capitalize()} color range: Lower = {lower}, Upper = {upper}")
                break
            else:
                print("Invalid selection. Please enter a valid number.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    # --- MAIN LOOP FOR LINE TRACKING ---
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        processed_frame, v, w= process_frame(frame, lower, upper, range_filter)
        resized_frame = imut.resize(processed_frame, width=720)
        cv2.imshow("Line Tracking", resized_frame)
        
        #compute Velocities
        twist_msg = Twist()
        twist_msg.linear.x = v #forward movement
        twist_msg.angular.z = w #turn based on detected line

        cmd_vel_pub.publish(twist_msg)
        print(f"Published Twist: linear={v}, angular={w}")

        # Press 'q' to quit.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
