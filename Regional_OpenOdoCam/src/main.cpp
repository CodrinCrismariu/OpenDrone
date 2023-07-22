#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/viz/viz3d.hpp"
#include "opencv2/core/ocl.hpp"

#include <bits/stdc++.h>
#include <ctime>

#include "feature.h"
#include "utils.h"
#include "evaluate_odometry.h"
#include "visualOdometry.h"
#include "Frame.h"

#include "camera_object.h"
#include "stereo_camera.h"

using namespace std;

int main(int argc, char **argv)
{
    cv::viz::Viz3d window; //creating a Viz window
    //Displaying the Coordinate Origin (0,0,0)
    cv::viz::WCoordinateSystem system = cv::viz::WCoordinateSystem(10);
    window.showWidget("coordinate", system);
    //Displaying the 3D points in green
    // window.showWidget("points", cv::viz::WCloud(pts3d, cv::viz::Color::green()));
    // window.spin();

    // Camera calibration
    string strSettingPath = string(argv[1]);
    cout << "Calibration Filepath: " << strSettingPath << endl;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float left_fx = fSettings["LeftCamera.fx"];
    float left_fy = fSettings["LeftCamera.fy"];
    float left_cx = fSettings["LeftCamera.cx"];
    float left_cy = fSettings["LeftCamera.cy"];

    float right_fx = fSettings["RightCamera.fx"];
    float right_fy = fSettings["RightCamera.fy"];
    float right_cx = fSettings["RightCamera.cx"];
    float right_cy = fSettings["RightCamera.cy"];

    float bf = (float)fSettings["stereo_baseline"] * right_fx;

    float COMPUTATION_TIME = fSettings["COMPUTATION_TIME"];
    const int NUM_THREADS = fSettings["NUM_THREADS"];

    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << left_fx, 0., left_cx, 0., 0., left_fy, left_cy, 0., 0,  0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << right_fx, 0., right_cx, bf, 0., right_fy, right_cy, 0., 0,  0., 1., 0.);
    // cout << "P_left: " << endl << projMatrl << endl;
    // cout << "P_right: " << endl << projMatrr << endl;

    // Initialize variables
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    cv::Vec3f total_rotation = rotationMatrixToEulerAngles(rotation);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat frame_pose32 = cv::Mat::eye(4, 4, CV_32F);

    // cout << "frame_pose " << frame_pose << endl;
    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    FeatureSet currentVOFeatures[NUM_THREADS];
    cv::Mat points4D, points3D;
    int init_frame_id = 0;

    // Load first images
    
    cv::Mat imageRight_Cached_t0;
    cv::Mat imageLeft_Cached_t0;

    cv::Mat imageRight_Cached_t1;
    cv::Mat imageLeft_Cached_t1;

    CameraBase *camera  = new StereoCamera;
    for (int throw_frames = 10 ; throw_frames >=0 ; throw_frames--) {

        imageLeft_Cached_t0 = imageLeft_Cached_t1;
        imageRight_Cached_t0 = imageRight_Cached_t1;

        camera->getLRFrames(imageLeft_Cached_t1,imageRight_Cached_t1);

    }

    long long frame_id = 0;

    std::thread threads[NUM_THREADS];

    auto start_job = [&](int id) {
        float start_of_job = clock();
        frame_id++;

        try {

            cout << endl << "frame id " << frame_id << endl;

            // Load images
            cv::Mat imageRight_t0 = imageRight_Cached_t0;
            cv::Mat imageLeft_t0 = imageLeft_Cached_t0;
            cv::Mat imageRight_t1 = imageRight_Cached_t1;
            cv::Mat imageLeft_t1 = imageLeft_Cached_t1;

            int grid_height = 1;
            int grid_width = 2;

            vector < cv::Mat > imgRight_t0 = createGrid(imageRight_t0, grid_height, grid_width);
            vector < cv::Mat > imgLeft_t0 = createGrid(imageLeft_t0, grid_height, grid_width);
            vector < cv::Mat > imgRight_t1 = createGrid(imageRight_t1, grid_height, grid_width);
            vector < cv::Mat > imgLeft_t1 = createGrid(imageLeft_t1, grid_height, grid_width);

            vector < vector<cv::Point2f> > vec_pointsLeft_t0(NUM_THREADS);
            vector < vector<cv::Point2f> > vec_pointsRight_t0(NUM_THREADS);
            vector < vector<cv::Point2f> > vec_pointsLeft_t1(NUM_THREADS);
            vector < vector<cv::Point2f> > vec_pointsRight_t1(NUM_THREADS);  
            
            cv::Size s = imageRight_t0.size();

            double HEIGHT_RES = s.height;
            double WIDTH_RES = s.width;

            double height_ratio = HEIGHT_RES / grid_height;
            double width_ratio = WIDTH_RES / grid_width;

            auto match_features = [&](int id) {

                try {

                    matchingFeatures( imgLeft_t0[id], imgRight_t0[id],
                                    imgLeft_t1[id], imgRight_t1[id], 
                                    currentVOFeatures[id],
                                    vec_pointsLeft_t0[id], 
                                    vec_pointsRight_t0[id], 
                                    vec_pointsLeft_t1[id], 
                                    vec_pointsRight_t1[id]);  

                    int y_0 = std::max(height_ratio * (id / grid_width) - HEIGHT_RES * 0.05, 0.0);
                    // int x_1 = std::min(height_ratio * (i + 1) + HEIGHT_RES * 0.05, HEIGHT_RES);

                    int x_0 = std::max(width_ratio * (id % grid_width) - WIDTH_RES * 0.05, 0.0);
                    // int y_1 = std::min(width_ratio * (j + 1) + WIDTH_RES * 0.05, WIDTH_RES);

                    for(int i = 0; i < vec_pointsLeft_t0[id].size(); i++) {
                        vec_pointsLeft_t0[id][i].x += x_0;
                        vec_pointsLeft_t0[id][i].y += y_0;
                    }

                    for(int i = 0; i < vec_pointsLeft_t1[id].size(); i++) {
                        vec_pointsLeft_t1[id][i].x += x_0;
                        vec_pointsLeft_t1[id][i].y += y_0;
                    }

                    for(int i = 0; i < vec_pointsRight_t0[id].size(); i++) {
                        vec_pointsRight_t0[id][i].x += x_0;
                        vec_pointsRight_t0[id][i].y += y_0;
                    }

                    for(int i = 0; i < vec_pointsRight_t1[id].size(); i++) {
                        vec_pointsRight_t1[id][i].x += x_0;
                        vec_pointsRight_t1[id][i].y += y_0;
                    }

                } catch(cv::Exception e) { cout << "LOST FEATURES" << endl; }
            };

            for(int i = 0; i < NUM_THREADS; i++) threads[i] = std::thread(match_features, i);
            for(int i = 0; i < NUM_THREADS; i++) threads[i].join();

            vector<cv::Point2f> pointsLeft_t0;
            vector<cv::Point2f> pointsRight_t0;
            vector<cv::Point2f> pointsLeft_t1;
            vector<cv::Point2f> pointsRight_t1;

            for(int i = 0; i < NUM_THREADS; i++) {
                pointsLeft_t0.insert(
                    pointsLeft_t0.end(),
                    std::make_move_iterator(vec_pointsLeft_t0[i].begin()),
                    std::make_move_iterator(vec_pointsLeft_t0[i].end())
                );
            }

            for(int i = 0; i < NUM_THREADS; i++) {
                pointsRight_t0.insert(
                    pointsRight_t0.end(),
                    std::make_move_iterator(vec_pointsRight_t0[i].begin()),
                    std::make_move_iterator(vec_pointsRight_t0[i].end())
                );
            }

            for(int i = 0; i < NUM_THREADS; i++) {
                pointsLeft_t1.insert(
                    pointsLeft_t1.end(),
                    std::make_move_iterator(vec_pointsLeft_t1[i].begin()),
                    std::make_move_iterator(vec_pointsLeft_t1[i].end())
                );
            }

            for(int i = 0; i < NUM_THREADS; i++) {
                pointsRight_t1.insert(
                    pointsRight_t1.end(),
                    std::make_move_iterator(vec_pointsRight_t1[i].begin()),
                    std::make_move_iterator(vec_pointsRight_t1[i].end())
                );
            }

            // Triangulate 3D Points
            cv::Mat points3D_t0, points4D_t0;
            cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t0,  pointsRight_t0,  points4D_t0);
            cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

            // Tracking transfomation
            trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation);
            displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1, "left");

            // Intergrating and display
            cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

            cv::Mat rigid_body_transformation;

            if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1) {

                integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation);
                total_rotation += rotation_euler;

            } else cout << "Too large rotation or too fast movement"  << endl;

            // cout << "total rotation: " << total_rotation << endl;

            cv::Mat xyz = frame_pose.col(3).clone();
            display(frame_id, trajectory, xyz);

            cout << "TIME OF JOB: " << float(clock() - start_of_job) / CLOCKS_PER_SEC * 1000 << '\n';

            window.spinOnce(1, true);
            translation = cv::Mat::zeros(3, 1, CV_64F);

            cv::Mat pose = frame_pose.col(3).clone();

            float x = -pose.at<double>(0);
            float y = -pose.at<double>(1);
            float z = pose.at<double>(2); 

            cv::Vec3f XYZ{x, y, z};
            system.setPose(cv::Affine3d(eulerAnglesToRotationMatrix(total_rotation), XYZ));

        } catch(cv::Exception e) { cout << "LOST FEATURES" << endl; }

    };

    clock_t lastUpdateTime = clock();

    while(true) {
        imageLeft_Cached_t0 = imageLeft_Cached_t1;
        imageRight_Cached_t0 = imageRight_Cached_t1;

        camera->getLRFrames(imageLeft_Cached_t1,imageRight_Cached_t1);
        start_job(0);
            
        cv::waitKey(1);
    }

    delete camera;

    return 0;
}

