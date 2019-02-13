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

#include <dre_slam/local_mapping.h>
#include <dre_slam/run_timer.h>

namespace dre_slam
{

	LocalMapping::LocalMapping (  LoopClosing* loop_closing, SubOctoMapConstruction* sub_OctoMap_construction, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Config* cfg ) :
	loop_closing_ ( loop_closing ), sub_OctoMap_construction_( sub_OctoMap_construction ), ros_puber_(ros_puber), optimizer_ ( optimizer ), map_ ( map ), cam_ ( cam ), cfg_ ( cfg )
{
    // start thread
    th_local_mapping_ = new std::thread ( &LocalMapping::processing, this );
	
} // local mapping

void LocalMapping::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
}

bool LocalMapping::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
}

KeyFrame* LocalMapping::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
}

void LocalMapping::processing()
{
    while ( true ) {

        if ( checkNewFrame() == true ) {
			
            KeyFrame* kf = getNewKeyFrame();

            kf->selectStaticFeatures();

            // add new mpts.
            int N = addNewMpts ( kf );

            // add visual edges.
            addVisualEdges ( kf );

            // Local optimization.
            if ( map_->getKFsSize() > 1.5 *  cfg_->sm_lm_window_size_ ) {
				
                std::vector<KeyFrame*> opt_kfs;
				map_->getLastNKeyFrames( cfg_->sm_lm_window_size_, true, opt_kfs );

				RunTimer t; t.start();
                optimizer_->localBA ( opt_kfs );
				t.stop();
				std::cout << "\nKeyFrame " << kf->kfid_ << " Local BA time: " << t.duration() << "\n\n";
            }

            // send keyframe to other thread.
            loop_closing_->insertKeyFrame ( kf );
			
			// send kf for octomap construction.
            sub_OctoMap_construction_->insertKeyFrame ( kf );
			
			// Publish KFs, and pose graph.
			ros_puber_->pubSparseMap( map_ );
        }
        usleep ( 5000 ); // sleep 5 ms
    } // while true.

} // processing


int LocalMapping::addNewMpts ( KeyFrame* kf )
{
    int N = 0;
    for ( size_t i = 0; i < kf->mpts_.size(); i ++ ) {

        // skip dynamic features.
        if ( kf->static_flags_.at ( i ) == false ) {
            continue;
        }

        MapPoint* mpt = kf->mpts_.at ( i );

        // If have a map point / add observation.
        if ( mpt != NULL ) {
            mpt->addObservation ( kf, i );
            continue;
        }

        // If no map point. created one.
        const cv::KeyPoint& kp = kf->kps_.at ( i );

        double& dp = kf->dps_.at ( i );
        if ( ( dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
            continue;
        }

        Eigen::Vector3d ptc = cam_ ->img2Cam ( Eigen::Vector2d ( kp.pt.x, kp.pt.y ), dp );
        Eigen::Vector3d ptw =kf->getSE3Pose() *  cfg_->Trc_ * ptc;
        MapPoint* nmpt = new MapPoint ( ptw, kf->des_.row ( i ) , cfg_ );
        kf->mpts_.at ( i ) = nmpt;
        nmpt->addObservation ( kf, i );
        map_->addMapPoint ( nmpt );

        N ++;
    }// for all mpts in kf

    return N;
} // addNewMpts


void LocalMapping::addVisualEdges ( KeyFrame* kf )
{
    std::vector<MapPoint*>& mpts = kf->mpts_;
    std::map<KeyFrame*, int> kfs_statistics;

    for ( MapPoint* mpt: mpts ) {
        if ( mpt == NULL ) {
            continue;
        }

        std::map<KeyFrame*, int>::iterator it;
        for ( it = mpt->ob_kfs_.begin(); it != mpt->ob_kfs_.end(); it ++ ) {
            KeyFrame* nkf = it->first;
            kfs_statistics[nkf] ++;
        } // for all ob kfs
    }//for all mpts

    std::map<KeyFrame*, int>::iterator it;
    for ( it = kfs_statistics.begin(); it != kfs_statistics.end(); it ++ ) {
        if ( it->first != kf && it->first != kf->getLastKeyFrameEdge() ) {
            if ( it->second > cfg_->sm_lm_cobv_th_ ) {
                kf->addVisualEdge ( it->first );
            }//
        } // if not last kf
    } // for all  kfs_statistics

} // addVisualEdges

} // namespace dre_slam
