import cv2
import copy

def extract_coordinates(event, x, y, flags, parameters):
    image_coordinates, image = parameters 
    
    # Record starting (x,y) coordinates on left mouse button click
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(image_coordinates) < 1:
            image_coordinates.append((x,y))
        else:
            print("selecting new region")
            image_coordinates = [(x,y)]
        
    # Record ending (x,y) coordintes on left mouse bottom release
    elif event == cv2.EVENT_LBUTTONUP:
        if len(image_coordinates) < 1:
            print("Ignoring mouse up since no top-left corner has been selected")
            return
        image_coordinates.append((x,y))
    
    # Clear drawing boxes on right mouse button click
    elif event == cv2.EVENT_RBUTTONDOWN:
        cv2.imshow("select", image) 

    # Draw rectangle around ROI
    if len(image_coordinates) > 0:
        image_copy = copy.copy(image)
        if len(image_copy.shape) < 3:
            image_copy = cv2.cvtColor(image_copy,cv2.COLOR_GRAY2RGB)
        cv2.rectangle(image_copy, image_coordinates[0], (x,y), (0,255,0), 2)
        cv2.imshow("select", image_copy) 
