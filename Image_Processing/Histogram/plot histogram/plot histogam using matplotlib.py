import cv2
import numpy as np
from matplotlib import pyplot as plt

img = np.zeros((200,200),np.uint8)
cv2.rectangle(img,(0,100),(200,200),(255),-1)
cv2.imshow('img',img)

#plot histogram using matplotlib
plt.hist(img.ravel(),256,[0,256])
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()
