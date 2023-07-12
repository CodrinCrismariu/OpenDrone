import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)

num = 0

chessboardSize = (7, 7)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

while cap.isOpened():

    succes1, img = cap.read()
    succes2, img2 = cap2.read()
    grayL = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)


    k = cv2.waitKey(5)

    # Find the chess board corners
    retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, None)

    if k == 27:
        break
    elif retL and retR:
        cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', img)
        cv2.imwrite('images/stereoRight/imageR' + str(num) + '.png', img2)
        print("images saved!")
        num += 1

    # If found, add object points, image points (after refining them)
    if retL and retR:

        objpoints.append(objp)

        cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, cornersL, retL)
        cv2.drawChessboardCorners(img2, chessboardSize, cornersR, retR)
        cv2.waitKey(200)

    cv2.imshow('Img 1',img)
    cv2.imshow('Img 2',img2)

