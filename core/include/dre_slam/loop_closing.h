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

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <dre_slam/camera.h>
#include <dre_slam/config.h>
#include <dre_slam/map.h>
#include <dre_slam/optimizer.h>
#include <dre_slam/vocabulary.h>
#include <dre_slam/keyframe.h>
#include <dre_slam/frame.h>
#include <thread>
#include <mutex>
#include <dre_slam/sub_octomap_construction.h>
#include <dre_slam/octomap_fusion.h>
#include <dre_slam/ros_puber.h>

namespace dre_slam
{

class LoopClosing
{
public:
	LoopClosing( SubOctoMapConstruction* sub_octomap_construction, OctoMapFusion* octomap_fusion, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Vocabulary* voc, Config* cfg);
	void insertKeyFrame(KeyFrame* kf); 
	
private:	
	bool checkLoop( KeyFrame* kf, KeyFrame*& lkf ); 
	bool computeTrans2KF(KeyFrame* rkf, KeyFrame* lkf, Sophus::SE2& delta_pose);
	int matchByBow(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint *> &vpMatches12);
	int matchNnearKfs(KeyFrame* kf,  std::vector<KeyFrame*>& lkfs, vector<MapPoint *> &vpMatches12);
	void processing();
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
 
private:
	SubOctoMapConstruction* sub_octomap_construction_;
	OctoMapFusion* octomap_fusion_;
	RosPuber* ros_puber_;
	Optimizer* optimizer_;
	Map* map_;
	Camera* cam_;
	Vocabulary* voc_;
	Config* cfg_;
     
	long int last_loop_kfid_;
	std::thread* th_loop_;
	std::queue<KeyFrame*> kfs_queue_;
	std::mutex mutex_kfs_queue_;
}; //class LoopClosing

}//namespace dre_slam

#endif
