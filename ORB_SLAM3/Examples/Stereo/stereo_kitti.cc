/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void getTranslationAndRotation(pangolin::OpenGlMatrix, cv::Vec3f&, cv::Mat&);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat);
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f);

int main(int argc, char **argv) {
    
    cout << argv[1] << ' ' << argv[2] << '\n';
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imLeft, imRight;
    cv::VideoCapture leftCap = cv::VideoCapture(0), rightCap = cv::VideoCapture(2);
    cv::Mat rotation = cv::Mat::zeros(3, 3, CV_32F);
    cv::Vec3f translation = cv::Vec3f();

    while (true) {
        leftCap.read(imLeft);
        rightCap.read(imRight);

        cv::cvtColor(imLeft, imLeft, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imRight, imRight, cv::COLOR_BGR2GRAY);

        if(imageScale != 1.f) {
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        SLAM.TrackStereo(imLeft, imRight, 0);

        getTranslationAndRotation(SLAM.getCurrentPose(), translation, rotation);

    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}


void getTranslationAndRotation(pangolin::OpenGlMatrix pose, cv::Vec3f &translation, cv::Mat &rotation) {

    rotation = (cv::Mat_<float>(3, 3) << pose.m[0], pose.m[4], pose.m[8], pose.m[1], pose.m[5], pose.m[9], pose.m[2], pose.m[6], pose.m[10]);
    translation = cv::Vec3f(pose.m[12], pose.m[13], pose.m[14]);

}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
 
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
     
}

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
 
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
 
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
 
    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
 
    return R;
 
}
