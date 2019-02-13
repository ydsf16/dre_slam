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

#include <dre_slam/camera.h>

namespace dre_slam
{
Camera::Camera ( int width, int height,
         double fx, double fy, double cx, double cy,
         double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0 ) :
    width_ ( width ), height_ ( height ),  fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy )
{
    d_[0] = k1;
    d_[1] = k2;
    d_[2] = p1;
    d_[3] = p2;
    d_[4] = k3;
    cvK_ = ( cv::Mat_<float> ( 3, 3 ) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0 );
    cvD_ = ( cv::Mat_<float> ( 1, 5 ) << d_[0], d_[1], d_[2], d_[3], d_[4] );
    K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
    K_inv_ = K_.inverse();
}


Eigen::Vector3d Camera::img2Cam ( const Eigen::Vector2d& px, const double& depth ) 
{
    Eigen::Vector3d xyz;
    xyz[0] = ( px[0] - cx_ ) * depth/fx_;
    xyz[1] = ( px[1] - cy_ ) * depth/fy_;
    xyz[2] = depth;
    return xyz;
}

Eigen::Vector2d Camera::cam2Img ( const Eigen::Vector3d& ptc ) 
{
    Eigen::Vector2d px;
    px[0] = fx_ * ptc[0] / ptc[2] + cx_;
    px[1] = fy_ * ptc[1] / ptc[2] + cy_;
    return px;
}

void Camera::undistortKeyPoints ( const std::vector< cv::KeyPoint >& dist_kps, std::vector< cv::KeyPoint >& undist_kps ) 
{
    int N = dist_kps.size();

    // Fill matrix with points
    cv::Mat mat ( N,2,CV_32F );
    for ( int i=0; i<N; i++ ) {
        mat.at<float> ( i,0 ) =dist_kps[i].pt.x;
        mat.at<float> ( i,1 ) =dist_kps[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape ( 2 );
    cv::undistortPoints ( mat, mat, cvK_, cvD_,cv::Mat(), cvK_ );
    mat=mat.reshape ( 1 );

    // Fill undistorted keypoint vector
    undist_kps.clear();
    undist_kps.resize ( N );
    for ( int i=0; i<N; i++ ) {
        cv::KeyPoint kp = dist_kps[i];
        kp.pt.x=mat.at<float> ( i,0 );
        kp.pt.y=mat.at<float> ( i,1 );
        undist_kps[i]=kp;
    }
} // undistortKeyPoints

bool Camera::projectWithCovariance ( const Sophus::SE3& Trc, const Sophus::SE3& T_w_rr,
                                     const Sophus::SE2& T_rr_rc, const Eigen::Matrix3d& cov_o, 
                                     const Eigen::Vector3d& pw, const double& simga_p, 
                                     Eigen::Vector2d& u, Eigen::Matrix2d& cov_u )
{

    static Sophus::SE3 Tcr = Trc.inverse();
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t;
    R.block(0, 0, 2, 2) = T_rr_rc.rotation_matrix();
    t << T_rr_rc.translation(), 0.0;
    Sophus::SE3 T_rr_rc3(R, t);
    
    Eigen::Vector3d pc = Tcr * T_rr_rc3.inverse() * T_w_rr.inverse() * pw;
    if(pc(2) < 0.01) 
        return false;
    
    u = cam2Img(pc);
   if(  isInFrame(u) == false )
       return false;

   /**** Covariance ****/
    Eigen::Matrix<double, 3, 4> K; 
    K.setZero();
    K.block(0,0, 3, 3) = K_;
    
    Eigen::Matrix<double, 3, 4> A = K * Tcr.matrix();
    Eigen::Vector3d B = T_w_rr.inverse() * pw;
    
    Eigen::Matrix<double, 3, 4> C = A * T_rr_rc3.inverse().matrix() * T_w_rr.inverse().matrix();
    double& a11 = A(0, 0);
    double& a12 = A(0, 1);
    double& a13 = A(0, 2);
    double& a14 = A(0, 3);
    double& a21 = A(1, 0);
    double& a22 = A(1, 1);
    double& a23 = A(1, 2);
    double& a24 = A(1, 3);
    double& a31 = A(2, 0);
    double& a32 = A(2, 1);
    double& a33 = A(2, 2);
    double& a34 = A(2, 3);
    
    double& b1 = B(0);
    double& b2 = B(1);
    double& b3 = B(2);
    
    double& c11 = C(0, 0);
    double& c12 = C(0, 1);
    double& c13 = C(0, 2);
    double& c14 = C(0, 3);
    double& c21 = C(1, 0);
    double& c22 = C(1, 1);
    double& c23 = C(1, 2);
    double& c24 = C(1, 3);
    double& c31 = C(2, 0);
    double& c32 = C(2, 1);
    double& c33 = C(2, 2);
    double& c34 = C(2, 3);
    
    const double& tx = T_rr_rc.translation()(0);
    const double& ty = T_rr_rc.translation()(1);
    const double& theta = T_rr_rc.so2().log();
    
    const double& x = pw(0);
    const double& y = pw(1);
    const double& z = pw(2);
	
	const double sin_theta = sin(theta);
	const double cos_theta = cos(theta);
    
	/**** Jacobian ****/
	double den1 = (a34 + a33*b3 + a31*b2*sin_theta - a32*b1*sin_theta - a31*tx*cos_theta - a32*ty*cos_theta + a32*tx*sin_theta - a31*ty*sin_theta + a31*b1*cos_theta + a32*b2*cos_theta);
	double den1_2 = den1 * den1;
	
	double Go11= (a12*a34*sin_theta - a14*a32*sin_theta - a11*a32*b2 + a12*a31*b2 + a11*a32*ty - a12*a31*ty - a11*a34*cos_theta + a14*a31*cos_theta - a11*a33*b3*cos_theta + a13*a31*b3*cos_theta + a12*a33*b3*sin_theta - a13*a32*b3*sin_theta)/den1_2;
	double Go12 = -(a11*a34*sin_theta - a14*a31*sin_theta - a11*a32*b1 + a12*a31*b1 + a11*a32*tx - a12*a31*tx + a12*a34*cos_theta - a14*a32*cos_theta + a12*a33*b3*cos_theta - a13*a32*b3*cos_theta + a11*a33*b3*sin_theta - a13*a31*b3*sin_theta)/ den1_2;
	double Go13 = (a11*a32*b1* b1 - a12*a31*b1* b1 + a11*a32*b2* b2 - a12*a31*b2* b2 + a11*a32*tx*tx - a12*a31*tx*tx + a11*a32*ty*ty - a12*a31*ty*ty + a11*a34*b2*cos_theta - a12*a34*b1*cos_theta - a14*a31*b2*cos_theta + a14*a32*b1*cos_theta - a11*a34*b1*sin_theta + a14*a31*b1*sin_theta - a12*a34*b2*sin_theta + a14*a32*b2*sin_theta + a12*a34*tx*cos_theta - a14*a32*tx*cos_theta - a11*a34*ty*cos_theta + a14*a31*ty*cos_theta + a11*a34*tx*sin_theta - a14*a31*tx*sin_theta + a12*a34*ty*sin_theta - a14*a32*ty*sin_theta - 2*a11*a32*b1*tx + 2*a12*a31*b1*tx - 2*a11*a32*b2*ty + 2*a12*a31*b2*ty + a11*a33*b2*b3*cos_theta - a12*a33*b1*b3*cos_theta - a13*a31*b2*b3*cos_theta + a13*a32*b1*b3*cos_theta - a11*a33*b1*b3*sin_theta + a13*a31*b1*b3*sin_theta - a12*a33*b2*b3*sin_theta + a13*a32*b2*b3*sin_theta + a12*a33*b3*tx*cos_theta - a13*a32*b3*tx*cos_theta - a11*a33*b3*ty*cos_theta + a13*a31*b3*ty*cos_theta + a11*a33*b3*tx*sin_theta - a13*a31*b3*tx*sin_theta + a12*a33*b3*ty*sin_theta - a13*a32*b3*ty*sin_theta)/ den1_2;
	double Go21 = (a22*a34*sin_theta - a24*a32*sin_theta - a21*a32*b2 + a22*a31*b2 + a21*a32*ty - a22*a31*ty - a21*a34*cos_theta + a24*a31*cos_theta - a21*a33*b3*cos_theta + a23*a31*b3*cos_theta + a22*a33*b3*sin_theta - a23*a32*b3*sin_theta)/den1_2;
	double Go22 =  -(a21*a34*sin_theta - a24*a31*sin_theta - a21*a32*b1 + a22*a31*b1 + a21*a32*tx - a22*a31*tx + a22*a34*cos_theta - a24*a32*cos_theta + a22*a33*b3*cos_theta - a23*a32*b3*cos_theta + a21*a33*b3*sin_theta - a23*a31*b3*sin_theta)/ den1_2;
	double Go23 =  (a21*a32*b1*b1 - a22*a31*b1*b1 + a21*a32*b2*b2 - a22*a31*b2*b2 + a21*a32*tx*tx - a22*a31*tx*tx + a21*a32*ty*ty - a22*a31*ty*ty + a21*a34*b2*cos_theta - a22*a34*b1*cos_theta - a24*a31*b2*cos_theta + a24*a32*b1*cos_theta - a21*a34*b1*sin_theta + a24*a31*b1*sin_theta - a22*a34*b2*sin_theta + a24*a32*b2*sin_theta + a22*a34*tx*cos_theta - a24*a32*tx*cos_theta - a21*a34*ty*cos_theta + a24*a31*ty*cos_theta + a21*a34*tx*sin_theta - a24*a31*tx*sin_theta + a22*a34*ty*sin_theta - a24*a32*ty*sin_theta - 2*a21*a32*b1*tx + 2*a22*a31*b1*tx - 2*a21*a32*b2*ty + 2*a22*a31*b2*ty + a21*a33*b2*b3*cos_theta - a22*a33*b1*b3*cos_theta - a23*a31*b2*b3*cos_theta + a23*a32*b1*b3*cos_theta - a21*a33*b1*b3*sin_theta + a23*a31*b1*b3*sin_theta - a22*a33*b2*b3*sin_theta + a23*a32*b2*b3*sin_theta + a22*a33*b3*tx*cos_theta - a23*a32*b3*tx*cos_theta - a21*a33*b3*ty*cos_theta + a23*a31*b3*ty*cos_theta + a21*a33*b3*tx*sin_theta - a23*a31*b3*tx*sin_theta + a22*a33*b3*ty*sin_theta - a23*a32*b3*ty*sin_theta)/ den1_2;
	Eigen::Matrix<double, 2, 3> Go;
	Go << Go11, Go12, Go13, Go21, Go22, Go23;
    
    double den2 = (c34 + c31*x + c32*y + c33*z);
    double den2_2 = den2 * den2;
    double Gp11 = (c11*c34 - c14*c31 + c11*c32*y - c12*c31*y + c11*c33*z - c13*c31*z)/den2_2;
    double Gp12 = (c12*c34 - c14*c32 - c11*c32*x + c12*c31*x + c12*c33*z - c13*c32*z)/den2_2;
    double Gp13 = (c13*c34 - c14*c33 - c11*c33*x + c13*c31*x - c12*c33*y + c13*c32*y)/den2_2;
    double Gp21 = (c21*c34 - c24*c31 + c21*c32*y - c22*c31*y + c21*c33*z - c23*c31*z)/den2_2;
    double Gp22 = (c22*c34 - c24*c32 - c21*c32*x + c22*c31*x + c22*c33*z - c23*c32*z)/den2_2;
    double Gp23 = (c23*c34 - c24*c33 - c21*c33*x + c23*c31*x - c22*c33*y + c23*c32*y)/den2_2;
    Eigen::Matrix<double, 2, 3> Gp;
    Gp << Gp11, Gp12, Gp13, Gp21, Gp22, Gp23;
    

    Eigen::Matrix3d cov_p;
    static double sigma_p2 = simga_p * simga_p;
    cov_p << sigma_p2, 0.0, 0.0, 
    0.0, sigma_p2, 0.0,
    0.0, 0.0, sigma_p2;
    
    cov_u = Go * cov_o * Go.transpose() + Gp * cov_p * Gp.transpose();
    
    return true;
}// projectWithCovariance


bool Camera::projectWorldPoint2Img ( const Sophus::SE3& Trc, const Sophus::SE3& Twr, const Eigen::Vector3d& pw, Eigen::Vector2d& u )
{
    Eigen::Vector3d pc = (Twr * Trc).inverse() * pw;
    if(pc(2) < 0.01) 
        return false;
    
    u = cam2Img(pc);
    if(  isInFrame(u) == false )
        return false;
    
    return true;
} // projectWorldPoint2Img


} //namespace dre_slam
