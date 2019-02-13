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

#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <sophus/se2.h>

namespace dre_slam
{

class Camera
{
public:
    Camera ( int width, int height,
             double fx, double fy, double cx, double cy,
             double k1, double k2 , double p1 , double p2, double k3);


    Eigen::Vector3d img2Cam ( const Eigen::Vector2d& px, const double& depth );
    Eigen::Vector2d cam2Img ( const Eigen::Vector3d& ptc );
    void undistortKeyPoints ( const std::vector< cv::KeyPoint >& dist_kps, std::vector< cv::KeyPoint >& undist_kps );
    bool projectWithCovariance(const Sophus::SE3& Trc, const Sophus::SE3& T_w_rr, const Sophus::SE2& T_rr_rc, const Eigen::Matrix3d& cov_o, const Eigen::Vector3d& pw, const double& simga_p, Eigen::Vector2d& u, Eigen::Matrix2d& cov_u);
    bool projectWorldPoint2Img(const Sophus::SE3& Trc, const Sophus::SE3& Twr, const Eigen::Vector3d& pw, Eigen::Vector2d& u);
    
    inline bool isInFrame ( const Eigen::Vector2d & obs, int boundary=0 ) const {
     if ( obs[0]>=boundary && obs[0]<width_-boundary
      && obs[1]>=boundary && obs[1]<height_-boundary ) {
      return true;
      }
      return false;
    }
    
    /* get parameters */
    inline const Eigen::Matrix3d& K() const {
        return K_;
    };
    inline const Eigen::Matrix3d& K_inv() const {
        return K_inv_;
    };
    inline double fx() const {
        return fx_;
    };
    inline double fy() const {
        return fy_;
    };
    inline double cx() const {
        return cx_;
    };
    inline double cy() const {
        return cy_;
    };
    inline double d0() const {
        return d_[0];
    };
    inline double d1() const {
        return d_[1];
    };
    inline double d2() const {
        return d_[2];
    };
    inline double d3() const {
        return d_[3];
    };
    inline double d4() const {
        return d_[4];
    };
    
    inline cv::Mat cvK() const {
     return cvK_;
    };
    
    inline cv::Mat dist() const {
      return cvD_;
    };
    
private:
    int width_, height_;
    const double fx_, fy_;
    const double cx_, cy_;
    double d_[5];                 
    cv::Mat cvK_, cvD_;    
    Eigen::Matrix3d K_, K_inv_;
}; //class camera

} //namespace dre_slam

#endif

