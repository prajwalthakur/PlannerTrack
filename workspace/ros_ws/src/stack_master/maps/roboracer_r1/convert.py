import cv2
import numpy as np

# Load grayscale map
img = cv2.imread("/home/prajwalthakur24/ws/src/race_stack/stack_master/maps/roboracer_r1/roboracer_r1_ros.pgm", cv2.IMREAD_GRAYSCALE)

# ROS map convention usually:
# white (254–255) -> free space
# gray  (~205)    -> unknown
# black (0)       -> occupied

# Keep only drivable area (free space) as white
# Everything else (unknown + occupied) becomes black

drivable = np.where(img > 250, 255, 0).astype(np.uint8)

cv2.imwrite("/home/prajwalthakur24/ws/src/race_stack/stack_master/maps/roboracer_r1/roboracer_r1.pgm", drivable)
print("Saved as track_bw.pgm")