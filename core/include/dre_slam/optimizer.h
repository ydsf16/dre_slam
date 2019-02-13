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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <sophus/se2.h>

#include <dre_slam/config.h>
#include <dre_slam/map.h>
#include <dre_slam/frame.h>

namespace dre_slam
{

class Optimizer
{
public:
    Optimizer(Map* map, Config* cfg):map_(map), cfg_(cfg){}
    
    // Tracking
    void motionOnlyBA( Frame* frame, KeyFrame* ref_kf, EncoderIntegration& encoder_kf2f_);
    
    // Local Mapping. Sliding window local BA.
    void localBA(std::vector<KeyFrame*>& opt_kfs );
  
    // Loop closure.
    void motionOnlyBA(std::vector<cv::KeyPoint>& kps, std::vector<MapPoint*>& mpts, Sophus::SE2& pose);
	void poseGraphOptimization(  KeyFrame* loop_KF  );
    
private:
    Config* cfg_;
    Map* map_;
    Eigen::Matrix2d sqrtMatrix2d(const Eigen::Matrix2d& TT);
    
    template <typename T>
    T sqrtMatrix(const T& TT);
    
    Eigen::Matrix3d sqrtMatrix3d(const Eigen::Matrix3d& TT);
    
};//class Optimizer

} //namespace dre_slam

#endif

