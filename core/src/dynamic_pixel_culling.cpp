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

#include <dre_slam/dynamic_pixel_culling.h>
#include <dre_slam/run_timer.h>

namespace dre_slam
{
DynamicPixelCulling::DynamicPixelCulling ( LocalMapping* local_mapping,ObjectDetector* object_detector,RosPuber* ros_puber, Map* map, Camera* cam, Config* cfg ) : local_mapping_ ( local_mapping ), ros_puber_ ( ros_puber ), map_ ( map ), cam_ ( cam ), cfg_ ( cfg )
{
    dynamic_pixel_detector_ = new DynamicPixelDetector ( object_detector, map, cam, cfg );
    th_dynamic_pixel_culling_ = new std::thread ( &DynamicPixelCulling::processing, this );
} // DynamicPixelCulling

void DynamicPixelCulling::processing()
{
    while ( true ) {
        if ( checkNewFrame() == true ) {
            KeyFrame* kf = getNewKeyFrame();

            // remove dynamic pixel.
			RunTimer t;
			t.start();
			dynamic_pixel_detector_->removeDynamicPixels ( kf );
			t.stop();
			std::cout << "\nKeyFrame " << kf->kfid_ << " Dynamic Pixel Culling time: " << t.duration() << "\n\n";
            

            // Send to local mapping
            local_mapping_->insertKeyFrame ( kf );

            // publish results of the dynamic pixel culling.
            ros_puber_->pubDynamicPixelCullingResults ( kf );

            // free the depth image of the last Nth kf.
            passed_kfs_queue_.push ( kf );

            if ( passed_kfs_queue_.size() > ( cfg_->dpc_n_near_kfs_+1 ) ) {
                KeyFrame* pass_kf = passed_kfs_queue_.front();
                passed_kfs_queue_.pop();
                pass_kf->freeDepth();
            }
        } // if new KF come in.

        usleep ( 5000 ); // sleep 5 ms.
    }
} // processing

bool DynamicPixelCulling::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
} // checkNewFrame

KeyFrame* DynamicPixelCulling::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
} // getNewKeyFrame

void DynamicPixelCulling::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
} // insertKeyFrame

} // namespace dre_slam
