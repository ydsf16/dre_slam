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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <dre_slam/dre_slam.h>
#include <sys/stat.h>

using namespace dre_slam;

class SensorGrabber
{
public:
	SensorGrabber ( DRE_SLAM* slam ) :slam_ ( slam ) {}
	
	void grabRGBD ( const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::ImageConstPtr& msg_depth ) {
		// Get images.
		cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare ( msg_rgb );
		cv_bridge::CvImageConstPtr cv_ptr_depth  = cv_bridge::toCvShare ( msg_depth );
		
		// Add RGB-D images.
		slam_->addRGBDImage ( cv_ptr_rgb->image, cv_ptr_depth->image, cv_ptr_rgb->header.stamp.toSec() );
		
	}// grabRGBD
	
	void grabEncoder ( const geometry_msgs::QuaternionStamped::ConstPtr& en_ptr ) {
		
		// Extract left and right encoder measurements.
		double enl1 = en_ptr->quaternion.x;
		double enl2 = en_ptr->quaternion.y;
		double enr1 = en_ptr->quaternion.z;
		double enr2 = en_ptr->quaternion.w;
		
		// Calculate left and right encoder.
		double enl = 0.5* ( enl1 + enl2 );
		double enr = 0.5* ( enr1 + enr2 );
		double  ts= en_ptr->header.stamp.toSec();
		
		// Check bad data.
		{
			
			if ( last_enl_ == 0 && last_enr_ == 0 ) {
				last_enl_ = enl;
				last_enr_ = enr;
				return;
			}
			
			double delta_enl = fabs ( enl - last_enl_ );
			double delta_enr = fabs ( enr - last_enr_ );
			
			const double delta_th = 4000;
			
			if ( delta_enl > delta_th || delta_enr > delta_th ) {
				std::cout << "\nJUMP\n";
				return;
			}
			
			last_enl_ = enl;
			last_enr_ = enr;
		}
		
		// Add encoder measurements.
		slam_->addEncoder ( enl, enr, ts );
	}// grabEncoder
	
private:
	DRE_SLAM* slam_;
	double last_enl_ = 0;
	double last_enr_ = 0;
};

int main ( int argc, char** argv )
{
    // Init ROS
    ros::init ( argc, argv, "DRE_SLAM" );
    ros::start();
    ros::NodeHandle nh;

    // Load parameters
	std::string dre_slam_cfg_dir, orbvoc_dir, yolov3_classes_dir, yolov3_model_dir, yolov3_weights_dir;
	std::string results_dir;
	if ( ! nh.getParam ( "/dre_slam_node/dre_slam_cfg_dir", dre_slam_cfg_dir ) ) {
		std::cout << "Read dre_slam_cfg_dir failure !\n";
        return -1;
    }
    
    if ( ! nh.getParam ( "/dre_slam_node/orbvoc_dir", orbvoc_dir ) ) {
		std::cout << "Read orbvoc_dir failure !\n";
		return -1;
	}
	
	if ( ! nh.getParam ( "/dre_slam_node/yolov3_classes_dir", yolov3_classes_dir ) ) {
		std::cout << "Read yolov3_classes_dir failure !\n";
		return -1;
	}
	if ( ! nh.getParam ( "/dre_slam_node/yolov3_model_dir", yolov3_model_dir ) ) {
		std::cout << "Read yolov3_model_dir failure !\n";
		return -1;
	}
	if ( ! nh.getParam ( "/dre_slam_node/yolov3_weights_dir", yolov3_weights_dir ) ) {
		std::cout << "Read yolov3_weights_dir failure !\n";
		return -1;
	}
	
	if ( ! nh.getParam ( "/dre_slam_node/results_dir", results_dir ) ) {
		std::cout << "Read results_dir failure !\n";
		return -1;
	}

	// Load system configure.
	Config* cfg = new Config( dre_slam_cfg_dir );
	
	// Init SLAM system.
	DRE_SLAM slam( nh, cfg, orbvoc_dir, yolov3_classes_dir, yolov3_model_dir, yolov3_weights_dir );
	
	// Sub topics.
	SensorGrabber sensors ( &slam );
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub ( nh, cfg->cam_rgb_topic_, 1 );
	message_filters::Subscriber<sensor_msgs::Image> depth_sub ( nh, cfg->cam_depth_topic_, 1 );
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync ( sync_pol ( 5 ), rgb_sub,depth_sub );
	sync.registerCallback ( boost::bind ( &SensorGrabber::grabRGBD,&sensors,_1,_2 ) ); //
	ros::Subscriber encoder_sub = nh.subscribe ( cfg->encoder_topic_, 1, &SensorGrabber::grabEncoder,&sensors );
	
	std::cout << "\n\nDRE-SLAM Started\n\n";
	
    ros::spin();

    // System Stoped.
    std::cout << "\n\nDRE-SLAM Stoped\n\n";

    // Save results.
	mkdir(results_dir.c_str(), S_IRWXU ); // Make a new folder.
	slam.saveFrames( results_dir +"/pose_frames.txt" );
	slam.saveKeyFrames( results_dir +"/pose_keyframes.txt" );
	slam.saveOctoMap( results_dir +"/octomap.bt" );
	
	std::cout << "Results have been saved to \"" + results_dir + "\".\n\n\n\n";
    return 0;
}


