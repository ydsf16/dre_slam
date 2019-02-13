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

#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>
#include <sophus/se2.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <dre_slam/config.h>
#include <dre_slam/camera.h>
#include <dre_slam/feature_detector.h>
#include <dre_slam/map_point.h>
#include <dre_slam/encoder.h>
#include <mutex>

namespace dre_slam{
class MapPoint;
class KeyFrame;


class Frame{
public:
    Frame(const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp, FeatureDetector* feature_detector, Camera* cam, Config* cfg);
    void extractFeatures(); 
    void getDepths();
    bool getDepth(const double& x, const double& y, double& dp); 
    void setPose(const Sophus::SE2& Twr);
    Sophus::SE2 getSE2Pose();
    Sophus::SE3 getSE3Pose();
    Sophus::SE2 getSE2PoseRefKF();
    Sophus::SE3 getSE3PoseRefKF();
    void setPoseRefKF(KeyFrame* kf);
    void freeMemory();

	void constructKDTree( const std::vector<cv::KeyPoint>& kps, pcl::KdTreeFLANN<pcl::PointXY>& kd_tree);
	int radiusSearch( const pcl::KdTreeFLANN<pcl::PointXY>& kd_tree, const Eigen::Vector2d& u,  const double& radius,std::vector<int>& ids);
	bool radiusSearchBestMatchKps( const Eigen::Vector2d& u, const double& radius, const cv::Mat& mpt_des, cv::KeyPoint& kp, int& des_dist, int& idx );
	bool radiusSearchBestMatchKpWithDistLimit( const Eigen::Vector2d& u, const double& radius, const Eigen::Vector3d& mpt_ptw, const cv::Mat& mpt_des, cv::KeyPoint& kp, int& des_dist, int& idx );
	
	void setEncoders(const std::vector<Encoder>& encs);
	std::vector<Encoder>& getEncoders();

public:
    static long int N_FRAMES;
    long int id_;
    double timestamp_;

	std::vector<cv::KeyPoint> kps_;  // keypoints.
	std::vector<bool> static_flags_; // Static flag with each keypoint.
	cv::Mat des_;				  // Descriptor with each keypoint.
	std::vector<double> dps_;     // Depth value with each keypoint.
	std::vector<MapPoint*> mpts_; // Mappoint with each keypoint.
    int n_features_, n_matched_;  // Number of features and the Number of matches.
	
    cv::Mat rgb_, gray_, depth_; // Raw images
    cv::Mat img_match_;
    
    Camera* cam_;
	Config* cfg_;
	
private:
    std::vector<Encoder> encoders_; 
	FeatureDetector* feature_detector_;
	
	// Frame pose
	std::mutex mutex_pose_;
	Sophus::SE2 T_rkf_f; 
	KeyFrame* ref_kf_; 
	Sophus::SE2 Twr2_; 
	Sophus::SE3 Twr3_; 
	
	pcl::KdTreeFLANN<pcl::PointXY> kd_tree_kps_; 
}; //class Frame    


} // namespace dre_slam

#endif
