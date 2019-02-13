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

#include <dre_slam/dre_slam.h>

namespace dre_slam
{

DRE_SLAM::DRE_SLAM ( ros::NodeHandle& nh, Config* cfg, const std::string& orbvoc_dir, const std::string& yolov3_classes_dir, const std::string& yolov3_model_dir, const std::string& yolov3_weights_dir ) : cfg_ ( cfg )
{
    // Load ORB Vocabulary.
    orb_voc_ = new Vocabulary();
    bool is_ok = orb_voc_->loadFromBinaryFile ( orbvoc_dir );
    if ( !is_ok ) {
        cerr << "Load vocabulary error!" << endl;
        exit ( -1 );
    }

    // Creat a camera
    cam_ = new Camera ( cfg_->cam_width_, cfg_->cam_height_,
                        cfg_->cam_fx_, cfg_->cam_fy_, cfg_->cam_cx_, cfg_->cam_cy_,
                        cfg_->cam_k1_, cfg_->cam_k2_, cfg_->cam_p1_, cfg_->cam_p2_, cfg_->cam_k3_ );

	// Map
	map_ = new Map(cfg_);
	
    // objector detector.
    objector_detector_ = new ObjectDetector ( yolov3_classes_dir, yolov3_model_dir, yolov3_weights_dir );

    // optimizer
    optimizer_ = new Optimizer ( map_, cfg_ );

    // ros puber
    ros_puber_ = new RosPuber ( nh );

	// octomap_fusion_
	octomap_fusion_ = new OctoMapFusion( ros_puber_, map_, cfg_);
	
	// sub octomap
	sub_octomap_construction_ = new SubOctoMapConstruction(octomap_fusion_, cfg_);
	
    //  loop closure
	loop_closing_ = new LoopClosing(sub_octomap_construction_, octomap_fusion_, ros_puber_, optimizer_, map_, cam_, orb_voc_, cfg_);

    // local mapping
    local_mapping_ = new LocalMapping ( loop_closing_, sub_octomap_construction_, ros_puber_, optimizer_, map_, cam_, cfg_ );

	// dynamic_pixel_culling_
	dynamic_pixel_culling_ = new DynamicPixelCulling(local_mapping_, objector_detector_, ros_puber_, map_, cam_, cfg_);
	
    // tracking
    tracking_ = new Tracking ( dynamic_pixel_culling_, ros_puber_, optimizer_, map_,  cam_, orb_voc_, cfg_ );
} // DRE_SLAM


void DRE_SLAM::addEncoder ( const double& enc_l, const double& enc_r, const double& timestamp )
{
    tracking_->addEncoder ( enc_l, enc_r, timestamp );
} // addEncoder

void DRE_SLAM::addRGBDImage ( const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp )
{
    tracking_->addRGBD ( rgb, depth, timestamp );
} // addRGBDImage


// Save map points.
void DRE_SLAM::saveMapPoints ( const string& dir )
{
    std::ofstream   file;
    file.open ( dir );

    std::set<MapPoint*> mpts = map_->getAllMapPoints();
    for ( MapPoint* mpt: mpts ) {
        Eigen::Vector3d ptw = mpt->getPosition();
        file << std::left <<std::setw ( 12 ) << fixed << setprecision ( 4 ) << ptw ( 0 )
             << std::left <<std::setw ( 12 ) << fixed << setprecision ( 4 ) << ptw ( 1 )
             << std::left <<std::setw ( 12 ) << fixed << setprecision ( 4 ) << ptw ( 2 )
             << std::left <<std::setw ( 12 ) << fixed << setprecision ( 4 )
             << std::left <<std::setw ( 5 )  << std::endl;
    }
}


void DRE_SLAM::saveKeyFrames ( const string& dir )
{
    std::ofstream   file;
    file.open ( dir );
    std::map<long int, KeyFrame*>::iterator iter;
    std::map<long int, KeyFrame*> kfs = map_->getAllKeyFrames();

    for ( iter =  kfs.begin(); iter != kfs.end(); iter ++ ) {
        KeyFrame* kf = iter->second;

        Sophus::SE2 Twr = kf->getSE2Pose();
        double theta = Twr.so2().log();
        Eigen::Matrix3d R;
        R << cos ( theta ), -sin ( theta ), 0.0,
          sin ( theta ), cos ( theta ), 0.0,
          0.0, 0.0, 1.0;
        Eigen::Quaterniond q ( R );
        std::vector<float> v ( 4 );
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();
        file << std::fixed << setprecision ( 6 )  <<  kf->timestamp_ << " " <<  std::setprecision ( 9 ) << Twr.translation() ( 0 )  << " " <<Twr.translation() ( 1 ) << " " << 0.0 << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
}

void DRE_SLAM::saveFrames ( const string& dir )
{
    std::ofstream   file;
    file.open ( dir );
    file <<  std::fixed;

    std::vector<Frame*> frames = map_->getAllFrames();

    std::vector<Frame*>::iterator iter;
    for ( iter =  frames.begin(); iter != frames.end(); iter ++ ) {

        Sophus::SE2 pose = ( *iter )->getSE2PoseRefKF();
        double theta = pose.so2().log();
        Eigen::Matrix3d R;
        R << cos ( theta ), -sin ( theta ), 0.0,
          sin ( theta ), cos ( theta ), 0.0,
          0.0, 0.0, 1.0;
        Eigen::Quaterniond q ( R );
        std::vector<float> v ( 4 );
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();

        file << setprecision ( 6 )  << ( *iter )->timestamp_ << " " <<  std::setprecision ( 9 ) << pose.translation() ( 0 )  << " " <<pose.translation() ( 1 ) << " " << 0.0 << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
} //

void DRE_SLAM::saveOctoMap ( const string& dir )
{
	octomap_fusion_->saveOctoMap(dir);
} // saveOctoMap


} // namespace dre_slam
