#include<iostream>
#include<algorithm>
#include<iomanip>
#include<string>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<CppLinuxSerial/SerialPort.hpp>

using namespace std;
using namespace mn::CppLinuxSerial;

void getTranslationAndRotation(pangolin::OpenGlMatrix, cv::Vec3f&, cv::Vec3f&);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f);

int main(int argc, char **argv) {

    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::ON, SoftwareFlowControl::OFF);

    serialPort.Open();
    
    cout << argv[1] << ' ' << argv[2] << '\n';
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    float imageScale = SLAM.GetImageScale();


    cv::Mat imLeft, imRight;
    cv::VideoCapture leftCap = cv::VideoCapture(2), rightCap = cv::VideoCapture(0);
    cv::Vec3f rotation = cv::Vec3f();
    cv::Vec3f translation = cv::Vec3f();
    
    leftCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    leftCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    rightCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    rightCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    while (true) {
        leftCap.read(imLeft);
        rightCap.read(imRight);

//        cv::cvtColor(imLeft, imLeft, cv::COLOR_BGR2GRAY);
//        cv::cvtColor(imRight, imRight, cv::COLOR_BGR2GRAY);
//
//       cv::resize(imLeft, imLeft, cv::Size(640, 480));
//        cv::resize(imRight, imRight, cv::Size(640, 480));

        SLAM.TrackStereo(imLeft, imRight, 0);

        getTranslationAndRotation(SLAM.getCurrentPose(), translation, rotation);
	
		cout << "x: " << translation[0] << "y: " << translation[1] << "z: " << translation[2] << '\n';
	    
        serialPort.Write(to_string(translation[0]) + " " + to_string(translation[1]) + " " + to_string(translation[2]) + " " + to_string(rotation[0]) + " " + to_string(rotation[1]) + " " + to_string(rotation[2]) + "\n");
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}


void getTranslationAndRotation(pangolin::OpenGlMatrix pose, cv::Vec3f &translation, cv::Vec3f &rotation) {

    cv::Mat rotationMatrix(3, 3, CV_64F);
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) 
			rotationMatrix.at<double>(i, j) = pose.m[j * 4 + i];
	}

	cv::Mat rotationVec;
	cv::Rodrigues(rotationMatrix, rotationVec);

	for(int i = 0; i < 3; i++) {
		rotation[i] = rotationVec.at<double>(i, 0);
	}

    translation = cv::Vec3f(pose.m[12], pose.m[13], pose.m[14]);

}
