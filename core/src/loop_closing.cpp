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

#include <dre_slam/loop_closing.h>
#include <dre_slam/common.h>
#include <dre_slam/run_timer.h>

namespace dre_slam
{

	LoopClosing::LoopClosing ( SubOctoMapConstruction* sub_octomap_construction, OctoMapFusion* octomap_fusion, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Vocabulary* voc, Config* cfg ) : sub_octomap_construction_(sub_octomap_construction),
    octomap_fusion_ ( octomap_fusion ), ros_puber_(ros_puber), optimizer_ ( optimizer ), map_ ( map ), cam_ ( cam ),  voc_ ( voc ), cfg_ ( cfg )
{
    last_loop_kfid_ = 0;
    th_loop_ = new std::thread ( &LoopClosing::processing, this );
} // LoopClosing

void LoopClosing::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
} // insertKeyFrame


bool LoopClosing::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
}

KeyFrame* LoopClosing::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
}


void LoopClosing::processing()
{
    while ( true ) {

        if ( checkNewFrame() == true ) {

            KeyFrame *kf = getNewKeyFrame();
            
			// check loop
            KeyFrame* lkf;
			bool isloop = checkLoop ( kf, lkf );

			if( isloop )
			{
				RunTimer t; t.start();
				
				optimizer_->poseGraphOptimization ( lkf );
				
				t.stop();
				std::cout << "\nKeyFrame " << kf->kfid_ << " Pose Graph Optimization time: " << t.duration() << "\n\n";
				
				last_loop_kfid_ = kf->kfid_;
				
				// insert all data to octomap.
				sub_octomap_construction_->insertAllData2OctoMapFusion();
				// send loop signal to octomap fusion.
				octomap_fusion_->setLoopFlag ();
				
				// publish the sparse map.
				ros_puber_->pubSparseMap( map_ );
			}
			
			// Free memory.
			kf->freeMemory();
        }
        usleep ( 5000 ); // sleep 5 ms.
    }
}


bool LoopClosing::checkLoop ( KeyFrame* kf, KeyFrame*& lkf )
{
    const size_t base_score_kf_num = 4;
    std::map<long int, KeyFrame*> map_kfs = map_->getAllKeyFrames();
	
    // if number of KFs is too small in the map.
    if ( map_kfs.size() < 10 ) {
        return false;
    }

    /* check keyframe pssed */
    if ( kf->kfid_ < ( last_loop_kfid_ + 10 ) ) {
        return false;
    }

    // Calculate base score
    double base_score = 1.0;
    std::map<long int, KeyFrame*> kfs = map_->getAllKeyFrames();
    std::map<long int, KeyFrame*>::reverse_iterator rit = ( kfs.rbegin() ++ );
    for ( int cnt = 0; ( rit != kfs.rend() && cnt != 4 ); rit ++, cnt ++ ) {
        double score = voc_->score ( rit->second->bow_vec_, kf->bow_vec_ );
        if ( score < base_score ) {
            base_score = score;
        }
    }

    // base score must be not too small.
    if ( base_score < 0.01 ) {
        return false;
    }

    // Get near kfs for compare.
    std::vector<KeyFrame*> near_kfs;
    int nkfs = map_->getNearKeyFrames ( kf, cfg_->sm_lc_search_dist_, cfg_->sm_lc_search_angle_, 10, near_kfs );

    if ( nkfs == 0 ) {
        return false;
    }
	
    double best_score = 0.0;
    long int best_kf_id = 1e10;
    std::vector<KeyFrame*>::iterator nkfs_it = near_kfs.begin();

    int N = 0;
    // for all near kfs
    for ( size_t i = 0; i < near_kfs.size(); i ++ ) {
        KeyFrame* nkf =  near_kfs.at ( i );
        double sim_score = voc_->score ( kf->bow_vec_, nkf->bow_vec_ );

		if ( sim_score > 1.1* base_score ) {

            Sophus::SE2 delta_pose;
			bool isok = computeTrans2KF ( kf, nkf, delta_pose );
            if ( isok == true ) {
				kf->addLoopEdge ( nkf, delta_pose );
				lkf = nkf;
                N ++;
            }
        }
    } // for all nkfs

    if ( N == 0 ) {
        return false;
    }

    return true;
} // checkLoop


bool LoopClosing::computeTrans2KF ( KeyFrame* kf, KeyFrame* lkf, Sophus::SE2& delta_pose )
{
	// Get Neighbour KFs.
    std::vector<KeyFrame*> near_kfs;
	int n_kfs = map_->getNeighbourKeyFrames ( lkf, 4, near_kfs );
    if ( n_kfs == 0 ) {
        return false;
    }

    // Match mpts.
    std::vector<MapPoint*> vmpts;
    int nmatched = matchNnearKfs ( kf, near_kfs, vmpts );

	double nmatched_th = 100;
	if(cfg_->ret_ft_n_features_ > 1000)
		nmatched_th = 100 + 0.05 * (cfg_->ret_ft_n_features_ - 1000);
	
	if ( nmatched < nmatched_th ) {
        return false;
    }

    /* pnp */
    std::vector<cv::KeyPoint> kps;
    std::vector<cv::Point2d> kpt2s;
    std::vector<cv::Point3d> mpt3s;
    std::vector<MapPoint*> mpts;

    for ( size_t i = 0; i < vmpts.size(); i ++ ) {
        MapPoint* mpt = vmpts.at ( i );

        if ( mpt == NULL ) {
            continue;
        }

        mpt->is_selected_ = false;

        cv::Point2d kpt2 ( kf->kps_.at ( i ).pt );
        kpt2s.push_back ( kpt2 );
        kps.push_back ( kf->kps_.at ( i ) );

        cv::Point3d mpt3;
        Eigen::Vector3d ptw = mpt->getPosition();
        mpt3.x = ptw ( 0 );
        mpt3.y = ptw ( 1 );
        mpt3.z = ptw ( 2 );

        mpt3s.push_back ( mpt3 );
        mpts.push_back ( mpt );
    }

    cv::Vec3d rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac ( mpt3s, kpt2s, cam_->cvK(), cv::Mat(), rvec, tvec, false, 100, 14.0, 0.95, inliers );
    double inlier_size = inliers.size[0];

    Eigen::Matrix4d T_c_w = AngleAxisTrans2EigenT ( rvec, tvec );
    Eigen::Matrix4d T_r_w =  cfg_->Trc_.matrix() * T_c_w;
    Eigen::Matrix4d T_w_r = T_r_w.inverse();
    double z = fabs ( T_w_r ( 2, 3 ) ) ;


    if ( (inlier_size > 55 && z < 0.012) || (inlier_size > 90 && z < 0.015) || (inlier_size > 200 && z < 0.025)) {
        std::vector<cv::KeyPoint> inlier_kps;
        std::vector<MapPoint*> inlier_mpts;

        /* select inliers */
        for ( size_t n = 0; n < inlier_size; n ++ ) {
            int idx = inliers.at<int32_t> ( n, 0 );
            inlier_kps.push_back ( kps.at ( idx ) );
            inlier_mpts.push_back ( mpts.at ( idx ) );
        }

        Sophus::SE2 pose = EigenT2Pose2d ( T_w_r ); 

        optimizer_->motionOnlyBA ( inlier_kps, inlier_mpts, pose );

        delta_pose =  lkf->getSE2Pose().inverse() * pose; 

        // check delta_pose;
        if ( fabs ( delta_pose.translation() ( 0 ) )  > 0.6 || fabs ( delta_pose.translation() ( 1 ) ) > 0.6 || fabs ( delta_pose.so2().log() ) > 0.7 ) { 
            return false;
        }

        return true;
    }
    return false;
}

int LoopClosing::matchNnearKfs ( KeyFrame* kf, std::vector<KeyFrame*>& lkfs, std::vector< MapPoint* >& vpMatches12 )
{

    for ( std::vector<KeyFrame*>::const_iterator iter = lkfs.begin(); iter != lkfs.end(); iter ++ ) {
        KeyFrame* lkf = *iter;

        /* reset match flag */
        for ( size_t i = 0; i < lkf->mpts_.size(); i ++ ) {
            MapPoint* mpt = lkf->mpts_.at ( i );
            if ( mpt != NULL ) {
                lkf->mpts_.at ( i )->is_selected_ = false;
            }
        }
    }// for near kfs

    /* for every keyframe */
    vpMatches12 = vector<MapPoint*> ( kf->mpts_.size(),static_cast<MapPoint*> ( NULL ) ); 
    for ( size_t i = 0; i < lkfs.size(); i ++ ) {
        KeyFrame* lkf = lkfs.at ( i );

        matchByBow ( kf, lkf,  vpMatches12 );
    }// for all selected kfs match by bow

    int N = 0;
    for ( size_t i = 0; i < vpMatches12.size(); i ++ ) {
        if ( vpMatches12.at ( i ) != NULL ) {
            N ++;
        }
    }
    return N;
}


int LoopClosing::matchByBow ( KeyFrame* pKF1, KeyFrame* pKF2, std::vector< MapPoint* >& vpMatches12 )
{
    //const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->feat_vec_;
    const vector<MapPoint*> vpMapPoints1 = pKF1->mpts_;
    const cv::Mat &Descriptors1 = pKF1->des_;

    //const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->feat_vec_;
    const vector<MapPoint*> vpMapPoints2 = pKF2->mpts_;
    const cv::Mat &Descriptors2 = pKF2->des_;

    //     vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL)); 
    vector<bool> vbMatched2 ( vpMapPoints2.size(),false );

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while ( f1it != f1end && f2it != f2end ) {
        if ( f1it->first == f2it->first ) {
            for ( size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++ ) {
                const size_t idx1 = f1it->second[i1];

                if ( vpMatches12[idx1] != NULL ) { 
                    continue;
                }

                const cv::Mat &d1 = Descriptors1.row ( idx1 );

                // Only use static features.
                if ( pKF1->static_flags_.at ( idx1 ) == false ) {
                    continue;
                }

                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;

                for ( size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++ ) {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if ( vbMatched2[idx2] || pMP2==NULL ) {
                        continue;
                    }

                    // The map points used for closed loop must be created near the loop keyframe.
                    KeyFrame* creat_kf = pMP2 ->first_ob_kf_;
                    if ( fabs ( creat_kf->kfid_ - pKF2->kfid_ ) > 3 ) {
                        continue;
                    }

                    if ( pMP2->is_selected_ == true ) {
                        continue;
                    }

                    const cv::Mat &d2 = Descriptors2.row ( idx2 );

                    int dist =  DescriptorDistance ( d1,d2 );

                    if ( dist<bestDist1 ) {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    } else if ( dist<bestDist2 ) {
                        bestDist2=dist;
                    }
                }

                if ( bestDist1 < 50 ) { 
                    if ( vpMatches12[idx1] != NULL ) {
                        continue;
                    }

                    vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                    vpMapPoints2[bestIdx2]->is_selected_ = true;

                    vbMatched2[bestIdx2]=true;
                    nmatches++;
                }
            }

            f1it++;
            f2it++;
        } else if ( f1it->first < f2it->first ) {
            f1it = vFeatVec1.lower_bound ( f2it->first );
        } else {
            f2it = vFeatVec2.lower_bound ( f1it->first );
        }
    }// while all kfs_

    return nmatches;
}


} // namespace dre_slam
