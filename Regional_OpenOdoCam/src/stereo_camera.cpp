// See example codes at https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/capture.c.html
// and https://gist.github.com/mike168m/6dd4eb42b2ec906e064d

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <string.h>
#include <fstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

#include "stereo_camera.h"

StereoCamera::StereoCamera () 
    : CameraBase ()
{
    m_frameCount      = 0;
    init_device ();
}

StereoCamera::~StereoCamera () {
    // deconstructor
}

int StereoCamera::init_device () {

    std::cout << "INITIALISE CAMERA" << std::endl;

    leftCap = cv::VideoCapture(4);
    rightCap = cv::VideoCapture(6);

    leftCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    leftCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    rightCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    rightCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    capture_frame();

    std::cout << "DEVICE INITIALISED" << std::endl;

    return 0;
}

int StereoCamera::capture_frame () {

    usleep(100000);
    leftCap.read(leftCached);
    usleep(100000);
    rightCap.read(rightCached);
    cv::cvtColor(leftCached, leftCached, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightCached, rightCached, cv::COLOR_BGR2GRAY);

    return 0;
}

void StereoCamera::getLRFrames (cv::Mat &left_rect, cv::Mat &right_rect) {
    capture_frame();

    incrementFrameCount ();

    left_rect = leftCached;
    right_rect = rightCached;

    saveFrame (leftCached, rightCached);
}

void StereoCamera::getLRFrames (unsigned char *left_rect, unsigned char *right_rect) {
    capture_frame();

    saveFrame (leftCached, rightCached);

    incrementFrameCount ();
}

#ifdef STANDALONE_TEST
int main(int argc, char *argv[]) {
    StereoCamera intelObject;
    for (int i = 0; i < 20; i++)
        intelObject.capture_frame ();
}
#endif
