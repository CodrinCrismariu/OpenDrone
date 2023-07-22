#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include <opencv2/opencv.hpp>
#include "camera_object.h"

struct v4l2_buffer;

class StereoCamera : public CameraBase {
    public:
                        StereoCamera      ();
        virtual        ~StereoCamera      ();
                int     init_device     ();
                int     capture_frame   ();
        virtual void    getLRFrames     (cv::Mat &left_rect, cv::Mat &right_rect);
	    virtual void    getLRFrames     (unsigned char *left_rect, unsigned char *right_rect);

    private:
        int             m_frameCount;

        cv::VideoCapture leftCap;
        cv::VideoCapture rightCap;

        cv::Mat leftCached;
        cv::Mat rightCached;
};

#endif
