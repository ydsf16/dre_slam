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

#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <dre_slam/ORBextractor.h>

namespace dre_slam
{

class FeatureDetector
{
public:
    FeatureDetector(int n_features, float scale_factor, int n_levels, int ini_th, int min_th);
    void detect(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    
    ~FeatureDetector(){
       delete orb_extractor_;
    }
    
private:
    ORBextractor* orb_extractor_;
}; // class FeatureDetector

} // namespace dre_slam

#endif
