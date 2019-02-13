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

#ifndef LOCAL_MAPPING_H
#define LOCAL_MAPPING_H

#include <thread>
#include <mutex>

#include<dre_slam/camera.h>
#include <dre_slam/config.h>
#include <dre_slam/map.h>
#include <dre_slam/loop_closing.h>
#include <dre_slam/optimizer.h>
#include <dre_slam/keyframe.h>
#include <dre_slam/sub_octomap_construction.h>
#include <dre_slam/ros_puber.h>

namespace dre_slam
{
class LocalMapping
{
public:

 	LocalMapping( LoopClosing* loop_closing, SubOctoMapConstruction* sub_OctoMap_construction, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Config* cfg );
	
    void processing();
    int addNewMpts(KeyFrame* kf);
    void addVisualEdges(KeyFrame* kf);
	void insertKeyFrame(KeyFrame* kf);
    bool checkNewFrame();
    KeyFrame* getNewKeyFrame();

	
private:
    LoopClosing* loop_closing_;
	SubOctoMapConstruction* sub_OctoMap_construction_;
	RosPuber* ros_puber_; 
    Optimizer* optimizer_;
    Map* map_;
    Camera* cam_;
    Config* cfg_;

    std::mutex mutex_kfs_queue_;
    std::queue<KeyFrame*> kfs_queue_; 
    std::thread* th_local_mapping_;

}; //class LocalMapping
} // namespace dre_slam

#endif
