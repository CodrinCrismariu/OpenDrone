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

int main(int argc, char **argv)
{

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imLeft, imRight;
    while (b_continue_session)
    {
        leftCap = cv::VideoCapture(4);
        rightCap = cv::VideoCapture(6);

        usleep(100000);
        leftCap.read(imLeft);
        usleep(100000);
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
    }

    pipe.stop();

    // Stop all threads
    SLAM.Shutdown();

    reeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee

    return 0;
}
