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

#include <dre_slam/frame.h>
#include <dre_slam/run_timer.h>
#include <dre_slam/common.h>

namespace dre_slam
{

long int Frame::N_FRAMES = 0;

Frame::Frame ( const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp, FeatureDetector* feature_detector, Camera* cam, Config* cfg ) :
    rgb_ ( rgb.clone() ), timestamp_ ( timestamp ), feature_detector_ ( feature_detector ), cam_ ( cam ), cfg_ ( cfg )
{
    /* set frame id */
    id_ = N_FRAMES;
    N_FRAMES ++;

    /* Convert color to gray and scale depth image */
    cv::cvtColor ( rgb, gray_, cv::COLOR_RGB2GRAY );
    depth.convertTo ( depth_, CV_32F, cfg_->cam_depth_factor_ );

    /* set init pose*/
    setPose ( Sophus::SE2() );
} //  Frame

void Frame::extractFeatures()
{
    // Extract features.
    std::vector<cv::KeyPoint> dist_kps;
    feature_detector_->detect ( gray_, dist_kps, des_ );

    // Undistor the keypoints.
    cam_->undistortKeyPoints ( dist_kps, kps_ );

    // Assign the feature number.
    n_features_ = kps_.size();

    // Assign a map point to each keypoint.
    mpts_ = std::vector<MapPoint*> ( n_features_, NULL );

    // Construct KD-Tree for all keypoints.
    constructKDTree ( kps_, kd_tree_kps_ );

    // Each keypoint is assigned a flag to indicate whether it is static or dynamic.
    static_flags_ = std::vector<bool> ( n_features_, true );
} // extractFeatures

bool Frame::getDepth ( const double& u, const double& v, double& dp )
{
    dp = depth_.at<float> ( v,u );

    // Limit the depth to suitable range.
    if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
        return true;
    }

    return false;
} // getDepth

void Frame::getDepths()
{
    for ( size_t i = 0; i < kps_.size(); i ++ ) {
        cv::KeyPoint& kp = kps_[i];
        double dp;
        bool ok = getDepth ( kp.pt.x, kp.pt.y, dp );
        if ( ok ) {
            dps_.push_back ( dp );
        } else {
            dps_.push_back ( 0.0 );
        }
    } // for all kps_
} // getDepths

void Frame::setPose ( const Sophus::SE2& Twr )
{
    Twr2_ = Twr;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.block ( 0,0, 2, 2 ) = Twr.matrix();

    Eigen::Vector3d t3;
    Eigen::Vector2d t2 = Twr.translation();
    t3 << t2, 0.0;

    Twr3_ = Sophus::SE3 ( R, t3 );
} // setPose

void Frame::constructKDTree ( const std::vector<cv::KeyPoint>& kps, pcl::KdTreeFLANN<pcl::PointXY>& kd_tree )
{
    pcl::PointCloud<pcl::PointXY>::Ptr cloud ( new pcl::PointCloud<pcl::PointXY> );

    for ( size_t i = 0; i < kps.size(); ++i ) {
        pcl::PointXY pt;
        const cv::KeyPoint& kp = kps.at ( i );
        pt.x = kp.pt.x;
        pt.y = kp.pt.y;
        cloud->push_back ( pt );
    }

    kd_tree.setInputCloud ( cloud );
} // constructKDTree


int Frame::radiusSearch ( const pcl::KdTreeFLANN< pcl::PointXY >& kd_tree, const Eigen::Vector2d& u, const double& radius, std::vector< int >& ids )
{
    pcl::PointXY search_point;
    search_point.x = u ( 0 );
    search_point.y = u ( 1 );
    std::vector<float> k_sqr_distances;
    return kd_tree.radiusSearch ( search_point, radius, ids, k_sqr_distances );
}


bool Frame::radiusSearchBestMatchKps ( const Eigen::Vector2d& u, const double& radius, const cv::Mat& mpt_des, cv::KeyPoint& kp, int& des_dist, int& idx )
{
    std::vector<int> ids;
    int n = radiusSearch ( kd_tree_kps_, u, radius, ids );

    if ( n == 0 ) {
        return false;
    }

    // get best match
    int min_dist = 1e8;
    int min_idx = -1;
    for ( size_t i = 0; i < n; i ++ ) {
        int dist = DescriptorDistance ( mpt_des, des_.row ( ids[i] ) );
        if ( dist < min_dist ) {
            min_dist = dist;
            min_idx = ids[i];
        }//
    } //

    kp = kps_.at ( min_idx );
    des_dist = min_dist;
    idx = min_idx;

    return true;
} // radiusSearchBestMatchTrackKps


bool Frame::radiusSearchBestMatchKpWithDistLimit ( const Eigen::Vector2d& u, const double& radius, const Eigen::Vector3d& mpt_ptw, const cv::Mat& mpt_des, cv::KeyPoint& kp, int& des_dist, int& idx )
{
    std::vector<int> ids;
    int n = radiusSearch ( kd_tree_kps_, u, radius, ids );

    if ( n == 0 ) {
        return false;
    }

    // get best match
    const int maxint = 1e8;
    int min_dist = 1e8;
    int min_idx = -1;
    for ( size_t i = 0; i < n; i ++ ) {

        // calculate geo_distance.
        double& dp = dps_.at ( ids[i] );

        if ( dp != 0 ) {

            cv::KeyPoint& mkp = kps_.at ( ids[i] );

            Eigen::Vector2d uv ( mkp.pt.x, mkp.pt.y );
            Eigen::Vector3d ptc = cam_->img2Cam ( uv, dp );
            Eigen::Vector3d ptw = getSE3Pose() * cfg_->Trc_ * ptc;
			double geo_dist = ( ptw - mpt_ptw ).norm();

            // Distance threshold. Using the depth image data to remove erroneous matches.
            if ( geo_dist > ( cfg_->ret_tk_db_ + cfg_->ret_tk_kd_ * dp ) ) { 
                continue;
            }
        }

        int dist = DescriptorDistance ( mpt_des, des_.row ( ids[i] ) );
        if ( dist < min_dist ) {
            min_dist = dist;
            min_idx = ids[i];
        }
    } 

    if ( min_dist == maxint ) {
        return false;
    }

    kp = kps_.at ( min_idx );
    des_dist = min_dist;
    idx = min_idx;

    return true;
} //



void Frame::setEncoders ( const std::vector<Encoder> &encs )
{
    encoders_ = encs;
}

std::vector<Encoder>& Frame::getEncoders()
{
    return encoders_;
}

Sophus::SE2 Frame::getSE2Pose()
{
    std::unique_lock<std::mutex> lock ( mutex_pose_ );
    return Twr2_;
}
Sophus::SE3 Frame::getSE3Pose()
{
    std::unique_lock<std::mutex> lock ( mutex_pose_ );
    return Twr3_;
}

// Set Reference KeyFrame
void Frame::setPoseRefKF ( KeyFrame *ref_kf )
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    ref_kf_ = ref_kf;
    T_rkf_f = ref_kf_->getSE2Pose().inverse() * Twr2_;
}

Sophus::SE2 Frame::getSE2PoseRefKF()
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    return ref_kf_->getSE2Pose() * T_rkf_f;
}

Sophus::SE3 Frame::getSE3PoseRefKF()
{

    Sophus::SE2 T = getSE2PoseRefKF();

    std::unique_lock<mutex> lock ( mutex_pose_ );

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.block ( 0,0, 2, 2 ) = T.matrix();

    Eigen::Vector3d t3;
    Eigen::Vector2d t2 = T.translation();
    t3 << t2, 0.0;

    return Sophus::SE3 ( R, t3 );
}

void Frame::freeMemory()
{
    kps_.clear();
	kps_.shrink_to_fit();
	
    static_flags_.clear();
	static_flags_.shrink_to_fit();
	
    des_ = cv::Mat();
	des_.release();
    
    dps_.clear(); 
	dps_.shrink_to_fit();
    mpts_.clear();
	mpts_.shrink_to_fit();
    
    encoders_.clear(); 
	encoders_.shrink_to_fit();

    rgb_ = cv::Mat();
    gray_ = cv::Mat();
    depth_ = cv::Mat();
	img_match_ = cv::Mat();
	
	rgb_.release();
	gray_.release();
	depth_.release();
	img_match_.release();
}

} // namespace dre_slam
