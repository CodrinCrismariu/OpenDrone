import cv2
import time

cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(2)

while True:

    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break

    cv2.imshow("frame0", frame0);
    cv2.imshow("frame1", frame1);

    
