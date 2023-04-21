import cv2
import numpy as np

# Read the image
img = cv2.imread('C:/Users/navan/Downloads/image.png')

crop_img = img[550:900,60:]
print(crop_img.shape)

cv2.imshow("cropped", crop_img)
cv2.waitKey(0)