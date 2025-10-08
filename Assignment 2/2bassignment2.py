#SeedLab EENG 350
#Jason Huang
# 9/19/25
# The following code uses the camera to take an image of a pdf and detects a green shape
#Start 2b



from time import sleep
import numpy as np
import cv2
import os

#Stop a error from popping up, just makes it pretty
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

#Start camera video
camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

sleep(1)
#Take a frame from camera
ret, colors = camera.read()

#Error check
if not ret:
	print("Could not capture image from camera!")
	quit()

#Convert to HSV
colorsHSV = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)

#Set bounds for detecting green shape
lowergreen = np.array([35, 40, 40])
uppergreen = np.array([90, 255, 255])

#Create mask 
mask = cv2.inRange(colorsHSV,lowergreen,uppergreen)
result = cv2.bitwise_and(colors,colors, mask=mask)

#Set a kernel to move across image
kernel = np.ones((5,5), np.uint8)
#Run a dual morphology, running erosion and dilation in conjunction to improve the image without significantly degrading the final image
opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations = 3)

# Draw out a shape around the detected shape
contours, hierarchy = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
output = colors.copy()
#Display the image
cv2.drawContours(output, contours, -1, (255,0,0), 2)
cv2.imshow("yay", output)
cv2.waitKey(0)
camera.release()
cv2.destroyAllWindows()
quit()

	
