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

#include <dre_slam/keyframe.h>
#include <dre_slam/common.h>

namespace dre_slam
{
long int KeyFrame::N_KFS = 0;

KeyFrame::KeyFrame ( Frame* frame , Vocabulary* voc ) :
    timestamp_ ( frame->timestamp_ ), kps_ ( frame->kps_ ), static_flags_ ( frame->static_flags_ ), des_ ( frame->des_.clone() ),
    n_features_ ( frame->n_features_ ), dps_ ( frame->dps_ ), mpts_ ( frame->mpts_ ),
    Twr2_ ( frame->getSE2Pose() ), Twr3_ ( frame->getSE3Pose() ),
    cam_ ( frame->cam_ ), cfg_ ( frame->cfg_ ), rgb_ ( frame->rgb_.clone() ), depth_ ( frame->depth_.clone() ),
    voc_ ( voc )
{
    fid_ = frame->id_;
    kfid_ = N_KFS;
    N_KFS ++;

    if ( bow_vec_.empty() || feat_vec_.empty() ) {
        vector<cv::Mat> vCurrentDesc = CvMat2DescriptorVector ( des_ );
        voc_->transform ( vCurrentDesc, bow_vec_, feat_vec_, 4 );
    }
} // KeyFrame


void KeyFrame::calculateRelativePosition()
{
    pts_r_ = std::vector<Eigen::Vector3d> ( n_features_, Eigen::Vector3d() );

    for ( size_t i = 0; i < n_features_; i ++ ) {
        MapPoint * mpt = mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }

        if ( mpt->first_ob_kf_->kfid_ != kfid_ ) {
            continue;
        }

        pts_r_.at ( i ) = Twr3_.inverse() * mpt->getPosition();
    }

} // calculateRelativePosition

void KeyFrame::reCalculateMpts()
{
    for ( size_t i = 0; i < n_features_; i ++ ) {
        MapPoint * mpt = mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }

        if ( mpt->first_ob_kf_->kfid_ != kfid_ ) {
            continue;
        }

        mpt->setPosition ( Twr3_ * pts_r_.at ( i ) );
    }
} // reCalculateMpts


void KeyFrame::reCalculateSingleMpts()
{
    for ( size_t i = 0; i < n_features_; i ++ ) {
        MapPoint * mpt = mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }

        if ( mpt->first_ob_kf_->kfid_ != kfid_ ) {
            continue;
        }

        if ( mpt->ob_kfs_.size() != 1 ) {
            continue;
        }

        mpt->setPosition ( Twr3_ * pts_r_.at ( i ) );
    }
} // reCalculateSingleMpts

void KeyFrame::setPose ( const Sophus::SE2 &Twr )
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    Twr2_ = Twr;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.block ( 0,0, 2, 2 ) = Twr.matrix();
    Eigen::Vector3d t3;
    Eigen::Vector2d t2 = Twr.translation();
    t3 << t2, 0.0;
    Twr3_ = Sophus::SE3 ( R, t3 );
} // setPose

Sophus::SE2 KeyFrame::getSE2Pose()
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    return  Twr2_;
} // getSE2Pose

Sophus::SE3 KeyFrame::getSE3Pose()
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    return Twr3_;
} // getSE3Pose

void KeyFrame::cvtDouble2Eigen()
{
    // std::unique_lock<mutex> lock ( mutex_pose_ );
    setPose ( Sophus::SE2 ( Twrd_[2], Eigen::Vector2d ( Twrd_[0], Twrd_[1] ) ) );
} // cvtDouble2Eigen

void KeyFrame::cvtEigen2Double()
{
    std::unique_lock<mutex> lock ( mutex_pose_ );
    Twrd_[0] = Twr2_.translation() ( 0 );
    Twrd_[1] = Twr2_.translation() ( 1 );
    Twrd_[2] = Twr2_.so2().log();
} // cvtEigen2Double


/****  Graph ****/
void KeyFrame::addLastKeyFrameEdge ( KeyFrame *kf )
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    last_kf_ = kf;
}

void KeyFrame::addVisualEdge ( KeyFrame *kf )
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    ob_kfs_.insert ( kf );
}

std::set<KeyFrame*> KeyFrame::getObKFs()
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    return ob_kfs_;
}

void KeyFrame::setObKFs ( const std::set<KeyFrame*>& ob_kfs )
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    ob_kfs_ = ob_kfs;
}

void KeyFrame::addLoopEdge ( KeyFrame *kf, const Sophus::SE2 &T )
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    loop_kfs_.push_back ( kf );
    loop_delta_pose_.push_back ( T );
}

KeyFrame* KeyFrame::getLastKeyFrameEdge()
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    return last_kf_;
}

std::set<KeyFrame*> KeyFrame::getVisualEdge()
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    return ob_kfs_;
}

int KeyFrame::getLoopEdge ( std::vector<KeyFrame *> &loop_kfs, std::vector<Sophus::SE2> &Ts )
{
    std::unique_lock<mutex> lock ( mutex_graph_ );
    loop_kfs = loop_kfs_;
    Ts = loop_delta_pose_;
    return loop_kfs.size();
}


void KeyFrame::freeMemory()
{
    rgb_ = cv::Mat();
    image_mask_ = cv::Mat();
    image_objects_ = cv::Mat();
    image_source_clusters_ = cv::Mat();
    image_static_clusters_ = cv::Mat();
	
	rgb_.release();
	image_mask_.release();
	image_objects_.release();
	image_source_clusters_.release();
	image_static_clusters_.release();
	
} // freeMemory


void KeyFrame::freeDepth()
{
    depth_ = cv::Mat();
	depth_.release();
} // freeDepth


// Assign the static flag to keypoint according to the depth image. If there is depth value, the keypoint is static. If there is no depth value, the keypoint is dynamic.
void KeyFrame::selectStaticFeatures()
{
    static_flags_.clear();

    for ( size_t nf = 0; nf < kps_.size(); nf ++ ) {

        const cv::Point2f& kpt = kps_.at ( nf ).pt;
        const float& dp = depth_.at<float> ( kpt.y, kpt.x );

        if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
            static_flags_.push_back ( true );
        } else {
            static_flags_.push_back ( false );
            mpts_.at(nf) = NULL; // delete the mpt.
        }
    } // for all keypoints.
    
} // selectStaticFeatures

} // namespace dre_slam
