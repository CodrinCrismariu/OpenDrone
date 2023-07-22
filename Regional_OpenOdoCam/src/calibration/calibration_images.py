import cv2

cap = cv2.VideoCapture(6)
cap2 = cv2.VideoCapture(4)

num = 0

while cap.isOpened():

    succes1, img = cap.read()
    succes2, img2 = cap2.read()

    grayL = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # retL, cornersL = cv2.findChessboardCorners(grayL, (7, 7), None)
    # retR, cornersR = cv2.findChessboardCorners(grayR, (7, 7), None)

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', img)
        cv2.imwrite('images/stereoRight/imageR' + str(num) + '.png', img2)
        print("images saved!")
        num += 1

    cv2.imshow('Img 1',img)
    cv2.imshow('Img 2',img2)

# Release and destroy all windows before termination
cap.release()
cap2.release()

cv2.destroyAllWindows()
