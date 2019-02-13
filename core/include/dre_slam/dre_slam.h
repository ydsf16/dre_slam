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

#ifndef DRE_SLAM_H
#define DRE_SLAM_H

#include <dre_slam/config.h>
#include <dre_slam/vocabulary.h>
#include <dre_slam/camera.h>
#include <dre_slam/sub_octomap_construction.h>
#include <dre_slam/octomap_fusion.h>
#include <dre_slam/tracking.h>
#include <dre_slam/dynamic_pixel_culling.h>
#include <dre_slam/local_mapping.h>
#include <dre_slam/loop_closing.h>
#include <dre_slam/optimizer.h>

namespace dre_slam{

class DRE_SLAM{
public:
	DRE_SLAM(  ros::NodeHandle& nh, Config* cfg, const std::string& orbvoc_dir, const std::string& yolov3_classes_dir, const std::string& yolov3_model_dir, const std::string& yolov3_weights_dir );
	
	void addRGBDImage(const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp);
	void addEncoder(const double& enc_l, const double& enc_r, const double& timestamp);
	
	// Save Results
	void saveMapPoints(const std::string& dir);
	void saveKeyFrames(const std::string& dir);
	void saveFrames(const std::string& dir);
	void saveOctoMap(const std::string& dir);
	
private:
	Config* cfg_;
 	Camera* cam_;
 	
 	Map* map_;
 	Tracking* tracking_;
 	DynamicPixelCulling* dynamic_pixel_culling_;
	LocalMapping* local_mapping_;
	LoopClosing* loop_closing_;
	SubOctoMapConstruction* sub_octomap_construction_;
	OctoMapFusion* octomap_fusion_;
	Optimizer* optimizer_;
	RosPuber* ros_puber_;
	
	Vocabulary* orb_voc_;
 	ObjectDetector* objector_detector_;
	
}; // class DRE_SLAM
	
	
} // namespace dre_slam


#endif // DRE_SLAM_H