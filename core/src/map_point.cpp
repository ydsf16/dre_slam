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

#include <dre_slam/map_point.h>

namespace dre_slam
{
MapPoint::MapPoint ( const Eigen::Vector3d& ptw, const cv::Mat& des , Config* cfg_) :
    ptw_ ( ptw ), des_ ( des.clone() )
{
    is_selected_ = false;
} // MapPoint

void MapPoint::setPosition ( const Eigen::Vector3d& ptw )
{
    std::unique_lock<std::mutex> lock ( mutex_position_ );
    ptw_ = ptw;
} // setPosition

Eigen::Vector3d MapPoint::getPosition()
{
    std::unique_lock<std::mutex> lock ( mutex_position_ );
    return ptw_;
} // getPosition


void MapPoint::addObservation ( KeyFrame* kf, int idx )
{
    if ( ob_kfs_.empty() )
        first_ob_kf_ = kf ;

    ob_kfs_[kf] = idx; 

} // addObservation


cv::Mat& MapPoint::getDescriptor()
{
    return des_;
} // getDescriptor


void MapPoint::cvtEigen2Double()
{
    std::unique_lock<std::mutex> lock ( mutex_position_ );
    ptwd_[0] = ptw_( 0 );
    ptwd_[1] = ptw_( 1 );
    ptwd_[2] = ptw_( 2 );
}

void MapPoint::cvtDouble2Eigen()
{
    std::unique_lock<std::mutex> lock ( mutex_position_ );
    ptw_[0] = ptwd_[0];
    ptw_[1] = ptwd_[1];
    ptw_[2] = ptwd_[2] ;
}


} // namespace dre_slam

