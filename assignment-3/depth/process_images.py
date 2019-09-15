#!/usr/bin/env python
# Example script to load the data, visualize the images, and capture clicks to identify the book
# Run the script as: ./process_images.py images.npz

import sys
import cv2
import numpy as np
from extract_coordinates import extract_coordinates

# -------------------------------------------------------------------------------
# get input path
images_file = sys.argv[1]

# load data
print("Loading {}".format(images_file))
data = np.load(images_file)

gray=data['gray']              # gray image captured by the camera
depth=data['depth']            # aligned depth image (transformed to match the gray image)
width=data['width']            # images width
height=data['height']          # images height
K=np.reshape(data['K'], (3,3)) # camera intrinsic parameters

# -------------------------------------------------------------------------------
# display the depth image
# but first normalize it so that all parts are visible and not too dark...
scaled_depth = 255 * depth / depth.max() # normalize data to 0 to 255
depth_as_int = scaled_depth.astype(np.uint8)

cv2.imshow("Depth", depth_as_int)
cv2.moveWindow("Depth", 20,20);

# display the gray image
cv2.imshow("Gray", gray)
cv2.moveWindow("Gray", 20 + width, 20)

print("You should now see the depth and gray image. Press any key to continue...")
cv2.waitKey(0)

# close all windows
cv2.destroyAllWindows() 

# -------------------------------------------------------------------------------
# get coordinates of book in the gray image
image_coordinates = []
image = gray 

cv2.namedWindow('select')
cv2.setMouseCallback('select', extract_coordinates, param=(image_coordinates, image))
cv2.imshow("select", gray)

print("Identify the object of interest by clicking the top-left corner of the front cover of the book with the left mouse button, holding the press until the mouse is in the bottom-right corner of the book, and then releasing the left button...")

while len(image_coordinates) < 2:
    cv2.waitKey(1) # wait until the region of interest is selected on the image...

cv2.destroyAllWindows()

print("Top-left: {}".format(image_coordinates[0]))
print("Bottom-right: {}".format(image_coordinates[1]))

# -------------------------------------------------------------------------------
# Implement your code donw below...



