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

#ifndef DYNAMIC_PIXEL_CULLING_H
#define DYNAMIC_PIXEL_CULLING_H

#include <dre_slam/local_mapping.h>
#include <dre_slam/dynamic_pixel_detector.h>
#include <dre_slam/ros_puber.h>

namespace dre_slam {
	
class DynamicPixelCulling{
	
public:
	DynamicPixelCulling( LocalMapping* local_mapping, ObjectDetector* object_detector, RosPuber* ros_puber, Map* map, Camera* cam, Config* cfg );
	void processing();
	
	void insertKeyFrame(KeyFrame* kf);
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
	
private:
	LocalMapping* local_mapping_;
	RosPuber* ros_puber_;
	Map* map_;
	Camera* cam_;
	Config* cfg_;
	
	// thread.
	std::mutex mutex_kfs_queue_;
	std::queue<KeyFrame*> kfs_queue_; 
	std::thread* th_dynamic_pixel_culling_;
	
	DynamicPixelDetector* dynamic_pixel_detector_;
	
	// Passed KFs
	std::queue<KeyFrame*> passed_kfs_queue_;

}; // class DynamicPixelCulling	
	
} //namespace dre_slam

#endif
