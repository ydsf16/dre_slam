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

#include <dre_slam/common.h>

namespace dre_slam
{

double normAngle ( double angle )
{
    static double Two_PI = 2.0 * M_PI;
    
    if ( angle >= M_PI ) {
        angle -= Two_PI;
    }
    if ( angle < -M_PI ) {
        angle += Two_PI;
    }
    return angle;
}


int DescriptorDistance ( const cv::Mat &a, const cv::Mat &b )
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for ( int i=0; i<8; i++, pa++, pb++ ) {
        unsigned  int v = *pa ^ *pb;
        v = v - ( ( v >> 1 ) & 0x55555555 );
        v = ( v & 0x33333333 ) + ( ( v >> 2 ) & 0x33333333 );
        dist += ( ( ( v + ( v >> 4 ) ) & 0xF0F0F0F ) * 0x1010101 ) >> 24;
    }
    return dist;
}

std::vector<cv::Mat> CvMat2DescriptorVector ( const cv::Mat &Descriptors )
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve ( Descriptors.rows );
    for ( int j=0; j<Descriptors.rows; j++ ) {
        vDesc.push_back ( Descriptors.row ( j ) );
    }

    return vDesc;
}

Eigen::Matrix4d AngleAxisTrans2EigenT ( cv::Vec3d rvec, cv::Vec3d tvec )
{
    /* convert rotation angle to R matrix */
    cv::Mat R;
    cv::Rodrigues ( rvec, R );

    /* convert to eigen style */
    Eigen::Matrix4d T;
    T<<
     R.at<double> ( 0, 0 ), R.at<double> ( 0, 1 ), R.at<double> ( 0, 2 ), tvec[0],
          R.at<double> ( 1, 0 ), R.at<double> ( 1, 1 ), R.at<double> ( 1, 2 ), tvec[1],
          R.at<double> ( 2, 0 ), R.at<double> ( 2, 1 ), R.at<double> ( 2, 2 ), tvec[2],
          0.,0.,0.,1.;
    return T;
}

Sophus::SE2 EigenT2Pose2d ( Eigen::Matrix4d& T )
{
    double theta = atan2 ( T ( 1,0 ), T ( 0,0 ) );
    double x = T ( 0, 3 );
    double y = T ( 1, 3 );
    return Sophus::SE2 ( theta, Eigen::Vector2d ( x, y ) );
}

} // namespace dre_slam
