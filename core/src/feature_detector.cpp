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

#include <dre_slam/feature_detector.h>

namespace dre_slam
{

FeatureDetector::FeatureDetector ( int n_features, float scale_factor, int n_levels, int ini_th, int min_th )
{
    orb_extractor_ = new ORBextractor ( n_features, scale_factor, n_levels, ini_th, min_th );
}

void FeatureDetector::detect ( const cv::Mat& img,  std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors )
{
    keypoints.clear();
    ( *orb_extractor_ ) ( img, cv::Mat(), keypoints, descriptors );
} // detect

} //namespace dre_slam
