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

#ifndef TRACKING_H
#define TRACKING_H

#include <mutex>
#include <thread>

#include <dre_slam/run_timer.h>
#include <dre_slam/optimizer.h>
#include <dre_slam/map.h>
#include <dre_slam/config.h>
#include <dre_slam/camera.h>
#include <dre_slam/frame.h>
#include <dre_slam/encoder.h>
#include <dre_slam/keyframe.h>
#include <dre_slam/encoder_integration.h>
#include <dre_slam/vocabulary.h>
#include <dre_slam/dynamic_pixel_culling.h>
#include <dre_slam/ros_puber.h>

namespace dre_slam
{

class Tracking
{
public:
    Tracking(DynamicPixelCulling* dynamic_pixel_culling, RosPuber* ros_puber, Optimizer* optimizer, Map* map,  Camera* cam, Vocabulary* voc, Config* cfg);

    void addRGBD(const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp);
    void addEncoder(const double& enc_l, const double& enc_r, const double& timestamp);

private:	
    void RGBDThread(); 
    void RGBDProcessing(); 

    bool initialization();
    int matchByProjection(const std::set<MapPoint*>& mpts, Frame* frame);

    bool checkNewFrame();
    Frame* getNewFrame();

	bool newKeyFrame();
	int discardOutliers(double th);
	void drawMatchedFeatures( cv::Mat& img_sum );
	
	double getSemiMajorAxisLength( const Eigen::Matrix2d& cov );
	

	bool inited_;

	DynamicPixelCulling* dynamic_pixel_culling_;
	RosPuber* ros_puber_;
	Optimizer* optimizer_;
	Map* map_;
	Camera* cam_;
	Vocabulary* voc_;
	Config* cfg_;
	FeatureDetector* feature_detector_;

	Frame* cur_frame_; // current frame

	std::mutex mutex_input_frames_;
	std::queue<Frame*> input_frames_; 
	std::thread* rgbd_thread_;

	long int last_frame_id_;

	// reference keyframe
	KeyFrame* ref_kf_;

	// raw encoder data.
	std::vector<Encoder> encoders_f2f_;

	// integrated encoder between the reference KF and the current frame.
	EncoderIntegration encoder_kf2f_;
   
}; //class Tracking

} //namespace dre_slam

#endif