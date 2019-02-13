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

#include <dre_slam/config.h>

namespace dre_slam{
	
Config::Config ( const std::string& cfg_dir)
{

	cv::FileStorage fs ( cfg_dir, cv::FileStorage::READ );
	
	/**** Camera ****/
	fs["cam_rgb_topic_"] >> cam_rgb_topic_;
	fs["cam_depth_topic_"] >> cam_depth_topic_;
	
	fs["cam_fx_"] >> cam_fx_;
	fs["cam_fy_"] >> cam_fy_;
	fs["cam_cx_"] >> cam_cx_;
	fs["cam_cy_"] >> cam_cy_;
	fs["cam_k1_"] >> cam_k1_;
	fs["cam_k2_"] >> cam_k2_;
	fs["cam_p1_"] >> cam_p1_;
	fs["cam_p2_"] >> cam_p2_;
	fs["cam_k3_"] >> cam_k3_;
	fs["cam_height_"] >> cam_height_;
	fs["cam_width_"] >> cam_width_;	
	fs["cam_depth_factor_"] >> cam_depth_factor_; 	// Depth scale factor.
	fs["cam_dmax_"] >> cam_dmax_;	// Max depth value to be used.
	fs["cam_dmin_"] >> cam_dmin_;	// Min depth value to be used.
	fs["cam_fps_"] >> cam_fps_;		// Camera FPs.
	
	/**** Robot intrinsic and extrinsic ****/
	fs["encoder_topic_"] >> encoder_topic_;
	
	fs["odom_kl_"] >> odom_kl_; // left wheel factor
	fs["odom_kr_"] >> odom_kr_; // right wheel factor
	fs["odom_b_"] >> odom_b_; 	// wheel space
	fs["odom_K_"] >> odom_K_; 	// Noise factor.
	
	/** robot extrinsic **/
	cv::Mat cvTrc;
	fs["Trc_"] >> cvTrc;
	Eigen::Matrix3d R;
	R << cvTrc.at<double> ( 0, 0 ), cvTrc.at<double> ( 0, 1 ), cvTrc.at<double> ( 0, 2 ),
	cvTrc.at<double> ( 1, 0 ), cvTrc.at<double> ( 1, 1 ), cvTrc.at<double> ( 1, 2 ),
	cvTrc.at<double> ( 2, 0 ), cvTrc.at<double> ( 2, 1 ), cvTrc.at<double> ( 2, 2 );
	Eigen::Vector3d t;
	t << cvTrc.at<double> ( 0, 3 ), cvTrc.at<double> ( 1, 3 ), cvTrc.at<double> ( 2, 3 );
	Trc_ = Sophus::SE3 ( R, t );
	
	
	/**** RGB-D Encoder Tracking ****/
	/** ORB feature **/
	fs["ret_ft_n_features_"] >>  ret_ft_n_features_; // Number of ORB features per frame.

	/** Tracking **/
	fs["ret_tk_dist_th_"] >> ret_tk_dist_th_; // Local map search radius.
	fs["ret_tk_angle_th_"] >> ret_tk_angle_th_; // Local map search angle.
	fs["ret_tk_db_"] >> ret_tk_db_; // The base threshold for erroneous match discard.
	fs["ret_tk_kd_"] >> ret_tk_kd_; // The scale factor of the threshold for erroneous match discard.
	
	/** Keyframe decision **/
	// Condation 1. 
	fs["ret_kd_fps_factor_"] >> ret_kd_fps_factor_; 
	// Condation 2: 
	fs["ret_kd_dist_th_"] >> ret_kd_dist_th_;   // Max distance 
	fs["ret_kd_angle_th_"] >> ret_kd_angle_th_; // Max angle 
	
	
	
	/**** Dynamic Pixels Culling ****/
	fs["dpc_n_near_kfs_"] >> dpc_n_near_kfs_;
	fs["dpc_npts_per_cluster_"] >> dpc_npts_per_cluster_; // Number of points per cluster.
	fs["dpc_n_sel_pts_per_cluster_"] >> dpc_n_sel_pts_per_cluster_; // Number of points per cluster to be selected for dynamic cluster decision.
	fs["dpc_search_square_size_"] >> dpc_search_square_size_;
	
	
	/**** Sparse Mapping ****/
	/** Local Mapping **/
	fs["sm_lm_window_size_"] >> sm_lm_window_size_; // local BA window size: sp_lm_window_size_ KFs.
	
	/**** OctoMap Construction ****/
	fs["oc_voxel_size_"] >> oc_voxel_size_;   // Voxel size of the OctoMap (m).
	fs["oc_submap_size_"] >> oc_submap_size_; // Sub-OctoMap size (KFs)
	
	fs.release();
} // Config

} // namespace dre_slam
