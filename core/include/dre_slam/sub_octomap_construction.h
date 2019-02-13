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

#ifndef SUB_OCTOMAPPING_H
#define SUB_OCTOMAPPING_H

#include <dre_slam/keyframe.h>
#include <dre_slam/config.h>
#include <octomap/octomap.h>
#include <dre_slam/sub_octomap.h>
#include <dre_slam/octomap_fusion.h>

#include <thread>

namespace dre_slam{
class OctoMapFusion;	

class SubOctoMapConstruction{
public:
	SubOctoMapConstruction( OctoMapFusion* octomap_fusion, Config* cfg );
	
	// insert new frame to
	void insertKeyFrame(KeyFrame* kf);
	void processing();
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
	
	// create points.
	void createPointCloud( KeyFrame* kf, octomap::Pointcloud& point_cloud );
	void createCleanCloud( KeyFrame* kf, octomap::Pointcloud& point_cloud );
	
	void insertAllData2OctoMapFusion();
private:
	OctoMapFusion* octomap_fusion_;
	Config* cfg_;
	
	// Thread.
	std::thread* sub_octomap_construction_thread_;
	std::queue<KeyFrame*> kfs_queue_;
	std::mutex mutex_kfs_queue_;
	
	// current sub map.
	SubOctomap* cur_sub_map_; 
	int nkf_passed_;
	
	// requestAllData
	std::mutex mutex_all_data_;
	
}; // class SubOctoMapConstruction
	
} // namespace dre_slam


#endif