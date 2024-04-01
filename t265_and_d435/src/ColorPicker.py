#! /usr/bin/env python
import numpy as np
import cv2
import pyrealsense2 as rs


def empty(a):
    pass

# Create a pipeline
pipeline = rs.pipeline()

# Create a configuration object
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# Create a window
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)

# Create trackbars for HSV
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
cv2.createTrackbar("Saturation Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Saturation Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Value Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Value Max", "TrackBars", 255, 255, empty)

try:
    while True:
        # Wait for a frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert the color image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image to HSV
        img_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Get trackbar values
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Saturation Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Saturation Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Value Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Value Max", "TrackBars")

        # Create lower and upper bounds for the mask
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Create a mask using the lower and upper bounds
        mask = cv2.inRange(img_hsv, lower_bound, upper_bound)

        # Apply the mask to the original image
        result = cv2.bitwise_and(color_image, color_image, mask=mask)

        # Display the original image and result
        cv2.imshow("Original Image", color_image)
        cv2.imshow("Result", result)

        # Exit when 'q' is pressed
        
	if cv2.waitKey(1) & 0xFF == ord('q'):
            break



finally:
    pipeline.stop()
    cv2.destroyAllWindows()



