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

#ifndef ROS_PUBER_H
#define ROS_PUBER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <dre_slam/map.h>

namespace dre_slam {

class RosPuber{
public:
	RosPuber( ros::NodeHandle& nh );
	
	void pubCurrentFrame( Frame* frame );
	void pubDynamicPixelCullingResults( KeyFrame* kf);
	void pubSparseMap( Map* map);
	void pubOctoMap( octomap::OcTree* octree );
	
private:
	ros::NodeHandle nh_;
	
	// Info of the current frame.
	ros::Publisher puber_robot_pose_; 
	image_transport::Publisher puber_img_match_;
	
	// Dynamic pixel detection results.
	image_transport::Publisher puber_dpc_img_objects_;
	image_transport::Publisher puber_dpc_img_clusters_;
	image_transport::Publisher puber_dpc_img_mask_;
	
	
	// KFs and pose graph. Sparse Map
	ros::Publisher puber_mappoints_;
	ros::Publisher puber_kfs_puber_; 
	ros::Publisher puber_encoder_graph_;        
	ros::Publisher puber_loop_graph_;        
	ros::Publisher puber_visual_graph_;
	
	// OctoMap.
	ros::Publisher puber_octomap_;
}; // RosPuber
	
} // namespace dre_slam


#endif
