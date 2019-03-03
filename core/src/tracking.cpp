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

#include <dre_slam/tracking.h>
#include <sstream>

namespace dre_slam
{

Tracking::Tracking ( DynamicPixelCulling* dynamic_pixel_culling, RosPuber* ros_puber, Optimizer* optimizer, Map* map,  Camera* cam, Vocabulary* voc, Config* cfg ) :
    dynamic_pixel_culling_ ( dynamic_pixel_culling ), ros_puber_ ( ros_puber ), optimizer_ ( optimizer ), map_ ( map ), cam_ ( cam ),  voc_ ( voc ), cfg_ ( cfg ), inited_ ( false ), last_frame_id_ ( 0 )
{
    // init feature detector.
    feature_detector_ = new FeatureDetector ( cfg_->ret_ft_n_features_, cfg_->ret_ft_scale_factor_, cfg_->ret_ft_n_levels_, cfg_->ret_ft_init_th_, cfg_->ret_ft_min_th_ );

    // Image processing thread.
    rgbd_thread_ = new std::thread ( &Tracking::RGBDThread, this );

} // Tracking

void Tracking::addRGBD ( const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp )
{
    // Time cost ~ 1 ms.
    Frame* frame = new Frame ( rgb, depth, timestamp, feature_detector_, cam_, cfg_ );
    frame->setEncoders ( encoders_f2f_ );
    encoders_f2f_.clear();

    std::unique_lock<mutex> lock ( mutex_input_frames_ );
    input_frames_.push ( frame );
} // addRGBD

void Tracking::addEncoder ( const double& enl, const double& enr, const double& timestamp )
{
    encoders_f2f_.push_back ( Encoder ( enl, enr, timestamp ) );
} // addEncoder


void Tracking::RGBDThread()
{
    while ( true ) {

        if ( checkNewFrame() ) {
            // get current frame.
            cur_frame_ = getNewFrame();
		
	    RunTimer t;
            t.start();
			
            // process current frame.
            RGBDProcessing();

            t.stop();
            std::cout << "Frame " << cur_frame_->id_ << " Tracking time: " << t.duration() << "\n";
        }// if new frame come in.

        usleep ( 3000 ); // sleep 3 ms.
    } // while true.
} //  RGBDThread


void Tracking::RGBDProcessing()
{
    if ( ! inited_ ) {

        initialization();
        inited_ = true;

        // Reset Encoder inegration from ref_keyframe to cur_frame.
        encoder_kf2f_ = EncoderIntegration ( cfg_->odom_kl_, cfg_->odom_kr_, cfg_->odom_b_, cfg_->odom_K_ );
        return;
    } // if not initialization

    encoder_kf2f_.addEncoders ( cur_frame_->getEncoders() );
    cur_frame_->setPose ( ref_kf_->getSE2Pose() * encoder_kf2f_.getTrr() ); // Set init pose

    // Extract new features and their depth values.
    cur_frame_->extractFeatures();
    cur_frame_->getDepths();

    /**** Track the local map ****/
    // Lock the map
    std::unique_lock<mutex> lock ( map_->update_mutex_ );

    // Get local map points
    std::set<MapPoint*> local_mpts;
    int nmpts = map_->getLocalMappoints ( cur_frame_, cfg_->ret_tk_dist_th_, cfg_->ret_tk_angle_th_, cfg_->ret_tk_max_local_mpts_, local_mpts );

    // match by projection.
    int nmatches = matchByProjection ( local_mpts, cur_frame_ );
    cur_frame_->n_matched_ = nmatches;

    // Refine
    if ( nmatches > 25 ) {
        optimizer_->motionOnlyBA ( cur_frame_, ref_kf_, encoder_kf2f_ );
        int n_good_match = discardOutliers ( 5 );

        cur_frame_->n_matched_ = n_good_match;
        if ( n_good_match > 20 ) {
            optimizer_->motionOnlyBA ( cur_frame_, ref_kf_, encoder_kf2f_ );
            n_good_match = discardOutliers ( 3 );
        }

        cur_frame_->n_matched_ = n_good_match;
    } else {
        // std::cout << "Less matches...\n";
    }

    // Draw tracked features.
    drawMatchedFeatures ( cur_frame_->img_match_ );

    // Save frame pose reference to the keyframe.
    cur_frame_->setPoseRefKF ( ref_kf_ );
    map_->addFrame ( cur_frame_ );

    /**** KeyFrame decision. ****/
    if ( newKeyFrame() ) {
        // Creat new keyframe.
        KeyFrame* kf = new KeyFrame ( cur_frame_, voc_ );
        kf->addLastKeyFrameEdge ( ref_kf_ );
        kf->Trr_ = encoder_kf2f_.getTrr();
        kf->covrr_= encoder_kf2f_.getCov();

        // Add the new keyframe to the map.
        map_->addKeyFrame ( kf );

        // Send the new keyframe to dynamic pixel culling module.
        dynamic_pixel_culling_->insertKeyFrame ( kf );

        ref_kf_ = kf;
        encoder_kf2f_ = EncoderIntegration ( cfg_->odom_kl_, cfg_->odom_kr_, cfg_->odom_b_, cfg_->odom_K_ );
    } // if is new kf

    // publish cur frame, pose. and image.
    ros_puber_->pubCurrentFrame ( cur_frame_ );

    // Free memory.
    cur_frame_->freeMemory();
} // RGBDProcessing


bool Tracking::initialization()
{
    // Set pose of current frame.
    cur_frame_->setPose ( Sophus::SE2() );

    // Extract new keyframes and get the depths.
    cur_frame_->extractFeatures();
    cur_frame_->getDepths();

    // Creat a new keyframe.
    KeyFrame* kf = new KeyFrame ( cur_frame_, voc_ );

    map_->addKeyFrame ( kf );
    dynamic_pixel_culling_->insertKeyFrame ( kf );

    ref_kf_ = kf;
    kf->addLastKeyFrameEdge ( ref_kf_ );
    cur_frame_->setPoseRefKF ( ref_kf_ );
    map_->addFrame ( cur_frame_ );

    // publish cur frame, pose. and image.
    ros_puber_->pubCurrentFrame ( cur_frame_ );

    // free memory of the current frame.
    cur_frame_->freeMemory();

    return true;
} // initialization


int Tracking::matchByProjection ( const std::set< MapPoint* >& mpts, Frame* frame )
{
    int n_matched = 0;

    Sophus::SE3 ref_kf_Twr = ref_kf_->getSE3Pose();
    Sophus::SE2 encoder_Trr = encoder_kf2f_.getTrr();
    Eigen::Matrix3d encoder_cov = encoder_kf2f_.getCov();

    for ( std::set<MapPoint*>::iterator it = mpts.begin(); it != mpts.end(); it ++ ) {
        MapPoint* mpt = *it;

        /* project to current frame */
        Eigen::Vector2d u;
        Eigen::Matrix2d cov;
        bool isok = cam_->projectWithCovariance ( cfg_->Trc_, ref_kf_Twr, encoder_Trr, encoder_cov,
                    mpt->getPosition(), cfg_->ret_tk_sigma_p_, u, cov );

        if ( !isok ) {
            continue;
        }

        // Policy 1. Search in ellipse. circle.
        double radius = getSemiMajorAxisLength ( cov );

        cv::KeyPoint kp;
        int des_dist;
        int idx;

        // Policy 2. distance threshold.
        bool mok = frame->radiusSearchBestMatchKpWithDistLimit ( u, radius, mpt->getPosition(), mpt->getDescriptor(), kp, des_dist,  idx );

        if ( !mok ) {
            continue;
        }

        if ( des_dist > 50 ) {
            continue;
        }

        // Get the best match for a keypoint.
        mpt->des_dist_ = des_dist;
        if ( frame->mpts_.at ( idx ) != NULL ) {

            if ( ( frame->mpts_.at ( idx )->des_dist_ > mpt->des_dist_ ) ) {
                frame->mpts_.at ( idx ) = mpt;
            }

        } else {
            frame->mpts_.at ( idx ) = mpt;
            n_matched ++;
        }
    } // for all mpts

    return n_matched;
} // matchByProjection

bool Tracking::newKeyFrame()
{
    // Condition 1.
    long int n_fms_passed = cur_frame_->id_ - last_frame_id_;


    static long int npassed_th = cfg_->ret_kd_fps_factor_ * cfg_->cam_fps_;
    if ( n_fms_passed < npassed_th ) {
        return false;
    }

    // Condition 2. Perhaps the of the variance can be used as a criterion.
    Sophus::SE2 Trr = encoder_kf2f_.getTrr();

	double dist = sqrt ( Trr.translation ()(0) *Trr.translation ()(0) + Trr.translation ()(1) *Trr.translation ()(1) );
    double angle = fabs ( Trr.so2().log() );

    if ( dist > cfg_->ret_kd_dist_th_ || angle > cfg_->ret_kd_angle_th_ ) {
        last_frame_id_ = cur_frame_->id_;
        return true;
    }

    return false;
} // newKeyFrame

int Tracking::discardOutliers ( double th )
{
	// TODO It is better to use the chi-square distribution.
    int n_good_mpts = 0;
    Sophus::SE3 cur_frame_Twr3 = cur_frame_->getSE3Pose();
    for ( size_t i = 0; i < cur_frame_->n_features_; i ++ ) {
        MapPoint* mpt = cur_frame_->mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }
        cv::KeyPoint& kp = cur_frame_->kps_.at ( i );

        // project to current frame and
        Eigen::Vector2d u;
        bool isok = cam_->projectWorldPoint2Img ( cfg_->Trc_, cur_frame_Twr3, mpt->getPosition(), u );
        if ( isok == false ) {
            cur_frame_->mpts_.at ( i ) = NULL;
            continue;
        }

        Eigen::Vector2d u_ob ( kp.pt.x, kp.pt.y );

        double dist = ( u_ob - u ).norm();

        if ( dist > th * ( pow<double> ( cfg_->ret_ft_scale_factor_, kp.octave ) ) ) { 
            cur_frame_->mpts_.at ( i ) = NULL;
            continue;
        }
        n_good_mpts ++;
    }// for all mpts

    return n_good_mpts;
} // discardOutliers


void Tracking::drawMatchedFeatures ( cv::Mat& img )
{
    img = cur_frame_->rgb_.clone();

    std::vector<cv::KeyPoint> kps;
    for ( size_t i = 0; i < cur_frame_->n_features_; i ++ ) {
        MapPoint * mpt = cur_frame_->mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }

        cv::KeyPoint& kp = cur_frame_->kps_.at ( i );
        cv::circle ( img, kp.pt, 4, cv::Scalar ( 255, 0, 0 ), -1 );
    } // for all features.
} // drawMatchedFeatures


bool Tracking::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_input_frames_ );
    return ( !input_frames_.empty() );
}

Frame* Tracking::getNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_input_frames_ );
    Frame* frame = input_frames_.front();
    input_frames_.pop();
    return  frame;
}


double Tracking::getSemiMajorAxisLength ( const Eigen::Matrix2d& cov )
{
    const static double Chi_Square_Probabilities = 10.597;

    cv::Mat cv_cov = ( cv::Mat_<double> ( 2,2 ) <<
                       cov ( 0,0 ), cov ( 0,1 ), cov ( 1,0 ), cov ( 1,1 ) );

    cv::Mat eigen_value, eigen_vector;
    cv::eigen ( cv_cov, eigen_value, eigen_vector );

    return sqrt ( eigen_value.at<double> ( 0,0 ) * Chi_Square_Probabilities ) ;
} // getSemiMajorAxisLength


} //namespace dre_slam

