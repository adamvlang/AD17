import whitelinedetecton
import cv2

image = cv2.imread("/home/nvidia/advanced-lane-detection/test_images/test1.jpg", cv2.IMREAD_COLOR)
#cv2.imshow('img', image)
#cv2.waitKey(0)
whitelinedetecton.pipeline(image)
