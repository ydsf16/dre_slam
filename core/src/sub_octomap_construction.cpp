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

#include <dre_slam/sub_octomap_construction.h>
#include <dre_slam/run_timer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace dre_slam
{

SubOctoMapConstruction::SubOctoMapConstruction ( OctoMapFusion* octomap_fusion, Config* cfg ) : octomap_fusion_ ( octomap_fusion ), cfg_ ( cfg )
{
    sub_octomap_construction_thread_ = new std::thread ( &SubOctoMapConstruction::processing, this );
    nkf_passed_ = 0;
} // SubOctomapping


void SubOctoMapConstruction::processing()
{
    while ( true ) {

        if ( checkNewFrame() == true ) {
            KeyFrame* kf = getNewKeyFrame();

			/*std::unique_lock<mutex> lock ( mutex_all_data_ );*/ 
			
            // construct sub map.
            if ( nkf_passed_ == 0 ) {
                cur_sub_map_ = new SubOctomap ( cfg_ );
            }

            // Construct 3D point cloud in the camera frame.
            octomap::Pointcloud point_cloud_c;
            //createCleanCloud ( kf, point_cloud_c ); // voxel_grid filtered
			createPointCloud( kf, point_cloud_c );

			// Insert one scan to full map.
			octomap_fusion_->insertOneScan2FullMapAndPub ( kf, point_cloud_c );
             
			// Insert one scan to cur sub map.
            cur_sub_map_->insertKeyFrame ( kf, point_cloud_c );

            // Check if need to construct new submap.
            nkf_passed_ ++;
            if ( nkf_passed_ > cfg_->oc_submap_size_ ) {
                nkf_passed_ = 0;
                SubOctomap* new_sub_map = cur_sub_map_;
				
				// insert one submap to fullmap.
				octomap_fusion_->insertSubMap ( new_sub_map );
            } // if have to insert submap.

        } // if have new keyframe.
        usleep ( 5000 );
    }// while true.
} // processing new keyframe.


void SubOctoMapConstruction::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
}


bool SubOctoMapConstruction::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
} // checkNewFrame

KeyFrame* SubOctoMapConstruction::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
} // getNewKeyFrame


void SubOctoMapConstruction::createPointCloud ( KeyFrame* kf, octomap::Pointcloud& point_cloud )
{
    cv::Mat& depth = kf->depth_;
    Camera* camera = kf->cam_;

    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {

            float dp = depth.at<float> ( v,u );
            if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
                Eigen::Vector3d ptc = camera->img2Cam ( Eigen::Vector2d ( u, v ), dp );
                point_cloud.push_back ( ptc[0], ptc[1], ptc[2] );
            } // if is good point.
        } // for all pixels.
    } //  for all pixels.

} // createPointCloud

void SubOctoMapConstruction::createCleanCloud ( KeyFrame* kf, octomap::Pointcloud& point_cloud )
{
    cv::Mat& depth = kf->depth_;
    Camera* camera = kf->cam_;

    // Create point cloud from the depth image.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ( new pcl::PointCloud<pcl::PointXYZ> );
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {

            float dp = depth.at<float> ( v,u );
            if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
                Eigen::Vector3d ptc = camera->img2Cam ( Eigen::Vector2d ( u, v ), dp );

                cloud->points.push_back ( pcl::PointXYZ ( ptc[0], ptc[1], ptc[2] ) );
            } // if is good point.
        } // for all pixels.
    } //  for all pixels.


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud ( cloud );
	sor.setLeafSize ( 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_ );
	sor.filter ( *cloud_filtered );

	// Assigned octomap cloud
	for ( size_t i = 0; i < cloud_filtered->size(); i ++ ) {
		pcl::PointXYZ pt = cloud_filtered->points.at ( i );
		point_cloud.push_back ( pt.x, pt.y, pt.z );
	} 
} // createCleanCloud


void SubOctoMapConstruction::insertAllData2OctoMapFusion()
{
// 	std::unique_lock<mutex> lock ( mutex_all_data_ );
	SubOctomap* new_sub_map = cur_sub_map_;
	if ( new_sub_map->kfs_.size() == 0 ) {
		return;
	}
	octomap_fusion_->insertSubMap ( new_sub_map );
	nkf_passed_ = 0;
} // requestAllData


} // namespace dre_slam
