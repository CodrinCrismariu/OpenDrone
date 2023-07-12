import cv2
import time

cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(2)

cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cnt = 1

while True:

    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    cv2.imwrite('cam0/' + str(cnt) + '.png', frame0)
    cv2.imwrite('cam1/' + str(cnt) + '.png', frame1)

    cnt += 1
        
    cv2.imshow('frame0', frame0);
    cv2.imshow('frame1', frame1);
    
    time.sleep(1)

    
