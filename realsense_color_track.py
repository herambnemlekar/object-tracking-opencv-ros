## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np  # np is used for array functions
import imutils  # image processing functions

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
count = 0
try:

	while True:
		# This call waits until a new coherent set of frames is available on a device
		# Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
		frames = pipeline.wait_for_frames()
		depth = frames.get_depth_frame()
		if not depth:
			continue

		lower = np.array([0, 180, 20])
		upper = np.array([6, 255, 110])
		colors = np.array([0, 0, 250])

		color_frame = frames.get_color_frame()
		frame = np.asanyarray(color_frame.get_data())
		frame = imutils.resize(frame, width=700)  # frame size
		blurred = cv2.GaussianBlur(frame, (7, 7), 0)  # removes noise #(7,7) is the size on which blur is applied at a time
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # change input frame to HSV

		kernel = np.ones((9, 9), np.uint8)
		mask = cv2.inRange(hsv, lower, upper)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # erosion followed by dilation
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # dilation followed by erosion

		# RETR_EXTERNAL is the mode, CHAIN_APPROX_SIMPLE is the method, [-2] is the offset
		contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		if len(contour)>0:
			c = max(contour, key=cv2.contourArea)
			(x,y), radius = cv2.minEnclosingCircle(c)
			print (x,y)

			# Moments
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# Specify min size of object to be detected
			if radius > 1:
				cv2.circle(frame, (int(x), int(y)), int(radius), colors, 2)
				cv2.putText(frame, "OBJECT", (int(x - (radius+1)), int(y - (radius+1))), cv2.FONT_HERSHEY_COMPLEX, 0.8, colors, 2)

		cv2.imshow("Frame", frame)
		cv2.waitKey(1)

finally:
	# Stop streaming
	pipeline.stop()
