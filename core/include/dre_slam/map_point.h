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

#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <dre_slam/config.h>
#include <dre_slam/keyframe.h>
#include <mutex>

namespace dre_slam{

class KeyFrame;    
    
class MapPoint{
public:
    MapPoint(const Eigen::Vector3d& ptw, const cv::Mat& des, Config* cfg_); 
	
    void setPosition( const Eigen::Vector3d& ptw);
    Eigen::Vector3d getPosition();
	
    void addObservation(KeyFrame* kf, int idx);
    cv::Mat& getDescriptor();
	
	// Graph 
 	std::map<KeyFrame*, int> ob_kfs_; 
 	KeyFrame* first_ob_kf_; 
	
	// Used for optimization.
	void cvtEigen2Double();
	void cvtDouble2Eigen();	
	double ptwd_[3]; 
	
	// used for match.
	bool is_selected_;
	int des_dist_; 
private:
    Eigen::Vector3d ptw_; 
    cv::Mat des_; 
    // Thread safe.
	std::mutex mutex_position_;
	std::mutex mutex_des_;

}; // class MapPoint
    
} //namespace dre_slam

#endif