# Question 6 : Use OpenCV to do a bilateral filter to an image, modify from question6.py, you may use your
# favorite image, visualize the images before and after the filtering using matplotlib.

import cv2
import matplotlib.pyplot as plt

img = cv2.imread('you_image.jpg')

bilateral = cv2.bilateralFilter(img, d=9, sigmaColor=75, sigmaSpace=75)

plt.subplot(121), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Original Input Image')
plt.subplot(122), plt.imshow(cv2.cvtColor(bilateral, cv2.COLOR_BGR2RGB)), plt.title('Bilateral Filtered Image')

cv2.imwrite('bilateral.jpg', bilateral)

plt.show()
