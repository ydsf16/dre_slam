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

#include <dre_slam/octomap_fusion.h>
#include <dre_slam/run_timer.h>
#include <dre_slam/sub_octomap_construction.h>

namespace dre_slam
{
OctoMapFusion::OctoMapFusion ( RosPuber* ros_puber, Map* map, Config* cfg ) : ros_puber_(ros_puber), map_ ( map ), cfg_ ( cfg )
{
    th_octomap_fusion_ = new std::thread ( &OctoMapFusion::processing, this );
    full_map_ = new octomap::OcTree ( cfg_->oc_voxel_size_ );
    full_map_->setOccupancyThres ( cfg_->oc_occ_th_ );
    full_map_->setProbHit ( cfg_->oc_prob_hit_ );
    full_map_->setProbMiss ( cfg_->oc_prob_miss_ );

    clamping_max_ = full_map_->getClampingThresMaxLog();
    clamping_min_ = full_map_->getClampingThresMinLog();
} // OctomapMerging

void OctoMapFusion::insertSubMap ( SubOctomap* submap )
{
    std::unique_lock<mutex> lock ( mutex_submaps_ );
    submaps_.push_back ( submap );
} // insertSubMap

void OctoMapFusion::insertOneScan2FullMapAndPub ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c )
{
    // Convert point cloud to world coordinate.
    octomap::Pointcloud point_cloud_w;
    Sophus::SE3 Twc = kf->getSE3Pose() * cfg_->Trc_;
    for ( size_t i = 0; i < point_cloud_c.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_c[i];
        Eigen::Vector3d ptc ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptw = Twc * ptc;
		
		// Delete error points. 
		if(ptw[2] > 6.0 || ptw[2] < -2.0)
			continue;
		
        point_cloud_w.push_back ( octomap::point3d ( ptw[0], ptw[1], ptw[2] ) );
    }

    std::unique_lock<mutex> lock ( mutex_full_map_ );
    // update the map.
    Eigen::Vector3d& pt_o = Twc.translation();
    full_map_->insertPointCloud ( point_cloud_w, octomap::point3d ( pt_o[0],pt_o[1],pt_o[2] ), -1, true, true );
    full_map_->updateInnerOccupancy();
	
	// publish OctoMap.
	ros_puber_->pubOctoMap(full_map_);
} // insertOneScan2FullMap


void OctoMapFusion::fusionAndPub()
{
    std::unique_lock<mutex> lock_sub ( mutex_submaps_ ); 
	std::list<SubOctomap*> tmp_submaps = submaps_;
	
	if ( tmp_submaps.size() == 0 ) {
        return;
    }

    // Create new tree.
    octomap::OcTree* new_tree = new octomap::OcTree ( cfg_->oc_voxel_size_ );
    new_tree->setOccupancyThres ( cfg_->oc_occ_th_ );
    new_tree->setProbHit ( cfg_->oc_prob_hit_ );
    new_tree->setProbMiss ( cfg_->oc_prob_miss_ );

    // insert sub tree to new tree.
	for ( SubOctomap* submap: tmp_submaps ) {
        insertSubMap2NewTree ( submap, new_tree );
    }

    std::unique_lock<mutex> lock_full ( mutex_full_map_ );
    delete full_map_;
    full_map_ = new_tree;
	
	// publish OctoMap.
	ros_puber_->pubOctoMap(full_map_);
} // fuse the sub octrees.


void OctoMapFusion::insertSubMap2NewTree ( SubOctomap* submap, octomap::OcTree* new_tree )
{
    Sophus::SE3 Twc = submap->kf_base_->getSE3Pose() * cfg_->Trc_;
    transformTree ( submap->sub_octree_, Twc, new_tree );
} // insertSubMap2NewTree


void OctoMapFusion::transformTree ( octomap::OcTree* src_tree, Sophus::SE3& Twc, octomap::OcTree* dst_tree )
{
    for ( octomap::OcTree::leaf_iterator it = src_tree->begin_leafs(); it != src_tree->end_leafs(); ++it ) {
        // src.
        Eigen::Vector3d pt_src ( it.getX(), it.getY(), it.getZ() );
		octomap::OcTreeNode* node_src = src_tree->search ( pt_src ( 0 ), pt_src ( 1 ), pt_src ( 2 ) );
        if ( node_src == NULL ) {
            continue;
        }
        double prob_log = node_src->getLogOdds();

        // dest.
        Eigen::Vector3d pt_dst = Twc * pt_src;

		octomap::OcTreeNode* node_dst = dst_tree->search ( pt_dst ( 0 ), pt_dst ( 1 ), pt_dst ( 2 ) );
        if ( node_dst == NULL ) {
            node_dst = dst_tree->updateNode ( pt_dst ( 0 ), pt_dst ( 1 ), pt_dst ( 2 ), true );
			prob_log = std::max<double> ( std::min<double> ( prob_log, clamping_max_ ),  clamping_min_ );
		
            node_dst->setLogOdds ( prob_log );
        } else {
            double prob_log_dst = node_dst->getLogOdds();
            double sum_log = prob_log_dst + prob_log;
  
			sum_log = std::max<double> ( std::min<double> ( sum_log, clamping_max_ ),  clamping_min_ );
            node_dst->setLogOdds ( sum_log );
        }
    } // for all leafs.
    dst_tree->updateInnerOccupancy();
} // transformTree


void OctoMapFusion::processing()
{
    static long int kf_num = 0;
    while ( true ) {
        if ( getLoopFlag () == true ) {
			// fusion & publish octomap.
			fusionAndPub();
        }
        usleep ( 5000 ); // sleep 5 ms.
    }
} // processing


bool OctoMapFusion::getLoopFlag ()
{
    std::unique_lock<mutex> lock ( mutex_loop_flag_ );
    if ( loop_flag_ == true ) {
        loop_flag_ = false;
        return true;
    }
    return false;
}

void OctoMapFusion::setLoopFlag ()
{
    std::unique_lock<mutex> lock ( mutex_loop_flag_ );
    loop_flag_ = true;
}

void OctoMapFusion::saveOctoMap ( const string& dir )
{
	std::unique_lock<mutex> lock_full ( mutex_full_map_ );
	full_map_->writeBinary( dir );
} // saveOctoMap

} //namespace dre_slam

