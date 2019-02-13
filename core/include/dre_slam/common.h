// This file is part of dre_slam - Dynamic RGB-D Encoder SLAM for Differential-Drive Robot.
//
// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)
//
// dre_slam is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// dre_slam is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef COMMON_H
#define COMMON_H

#include <sophus/se2.h>
#include <sophus/se3.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

namespace dre_slam{

double normAngle ( double angle );
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
std::vector<cv::Mat> CvMat2DescriptorVector(const cv::Mat &Descriptors);
Eigen::Matrix4d AngleAxisTrans2EigenT(cv::Vec3d rvec, cv::Vec3d tvec);
Sophus::SE2 EigenT2Pose2d(Eigen::Matrix4d& T);

} ; // namespace dre_slam

#endif
