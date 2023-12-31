#ifndef UTILS_H
#define UTILS_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <bits/stdc++.h>
#include <ctime>

#include "feature.h"
#include "matrix.h"

// --------------------------------
// Visualization
// --------------------------------
void drawFeaturePoints(cv::Mat image, std::vector<cv::Point2f>& points);

void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose);



// --------------------------------
// Transformation
// --------------------------------
void integrateOdometryStereo(int frame_id, cv::Mat& rigid_body_transformation, cv::Mat& frame_pose, const cv::Mat& rotation, 
                            const cv::Mat& translation_stereo);

bool isRotationMatrix(cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta);

#endif
