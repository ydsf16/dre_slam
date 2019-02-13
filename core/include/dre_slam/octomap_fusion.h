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

#ifndef OCTOMAP_FUSION
#define OCTOMAP_FUSION
#include <dre_slam/sub_octomap.h>
#include <dre_slam/map.h>
#include <dre_slam/ros_puber.h>
#include <thread>
#include <mutex>

namespace dre_slam {
class SubOctoMapConstruction;

class OctoMapFusion{
public:
	OctoMapFusion( RosPuber* ros_puber, Map* map, Config* cfg );
	void insertSubMap( SubOctomap* submap );
	void insertOneScan2FullMapAndPub( KeyFrame* kf, octomap::Pointcloud& point_cloud_c );
	void fusionAndPub();  
	void insertSubMap2NewTree(SubOctomap* submap, octomap::OcTree* new_tree);
	void processing();
	void setLoopFlag();
	bool getLoopFlag();
	void saveOctoMap( const std::string& dir );
	
private: 
	void transformTree(octomap::OcTree* src_tree, Sophus::SE3& Twc, octomap::OcTree* dst_tree);
	
	RosPuber* ros_puber_;	
	Map* map_;
	Config* cfg_;
	octomap::OcTree* full_map_;
	std::list<SubOctomap*> submaps_;
	
	std::thread* th_octomap_fusion_;
	std::mutex mutex_submaps_;
	std::mutex mutex_full_map_;
	
	std::mutex mutex_loop_flag_;
	bool loop_flag_ = false;
	KeyFrame* ckf_; 
	
	double clamping_max_;
	double clamping_min_;

}; // class OctoMapFusion
	
} // namespace dre_slam

#endif