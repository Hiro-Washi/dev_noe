import cv2
img = cv2.imread('images/femail_1.png')

# B & W
#img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Inverse
#img = cv2.flip(img,1)

# Resizeing
#size = 224
#img = cv2.resize(img, (size,size))

# Combine, Overlay, Synthesize
#img1 = cv2.imread('images/femail_1.png')
#img2 = cv2.imread('images/femail_1.png')

cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
