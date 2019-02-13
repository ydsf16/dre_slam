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

#include <dre_slam/encoder_integration.h>
#include <dre_slam/common.h>

namespace dre_slam
{

EncoderIntegration::EncoderIntegration ( const double& kl, const double& kr, const double& b, const double& k ) :
    kl_ ( kl ), kr_ ( kr ), b_ ( b ), k_ ( k ), is_init_ ( false ), x_ ( 0.0 ), y_ ( 0.0 ), th_ ( 0.0 )
{
}

Sophus::SE2 EncoderIntegration::addEncoder ( const Encoder& enc )
{

    if ( is_init_ == false ) {
        last_enc_ = enc;
        Trr_ = Sophus::SE2();
        cov_.setZero();
        is_init_ = true;
    } else { 
        double delta_enl = enc.enl_ - last_enc_.enl_;
        double delta_enr = enc.enr_ - last_enc_.enr_;
        last_enc_ = enc;
		
        double delta_sl = kl_ * delta_enl;
        double delta_sr = kr_ * delta_enr;

        double delta_theta = ( delta_sr - delta_sl ) / b_;
        double delta_s = 0.5 * ( delta_sr + delta_sl );

        double tmp_th = th_ + 0.5 * delta_theta;
        double cos_tmp_th = cos ( tmp_th );
        double sin_tmp_th = sin ( tmp_th );

        x_ += delta_s * cos_tmp_th;
        y_ += delta_s * sin_tmp_th;
        th_ += delta_theta;
        th_ = normAngle ( th_ ); 

        Trr_ = Sophus::SE2 ( th_, Eigen::Vector2d ( x_, y_ ) );

        Eigen::Matrix3d Gt;
        Gt << 1.0, 0.0, -delta_s * sin_tmp_th,
           0.0, 1.0, delta_s * cos_tmp_th,
           0.0, 0.0, 1.0;

        Eigen::Matrix<double, 3, 2> Gu;
        Gu << 0.5  * ( cos_tmp_th - delta_s * sin_tmp_th / b_ ), 0.5  * ( cos_tmp_th + delta_s * sin_tmp_th / b_ ),
           0.5  * ( sin_tmp_th + delta_s * cos_tmp_th /b_ ), 0.5  * ( sin_tmp_th - delta_s * cos_tmp_th/b_ ),
           1.0/b_, -1.0/b_;

        Eigen::Matrix2d sigma_u;
        sigma_u << k_ * k_ * delta_sr * delta_sr, 0.0, 0.0, k_ * k_* delta_sl * delta_sl;

        cov_ = Gt * cov_ *Gt.transpose() +  Gu * sigma_u * Gu.transpose() ;
    } // odom

    return Trr_;
}

Sophus::SE2 EncoderIntegration::addEncoders ( const std::vector<Encoder>& encs )
{
    for ( size_t i = 0; i < encs.size(); i ++ ) {
        const Encoder& enc = encs.at ( i );
        addEncoder ( enc );
    }
}


Eigen::Matrix3d EncoderIntegration::getCov()
{
    return cov_;
} // getCov


Sophus::SE2 EncoderIntegration::getTrr()
{
    return Trr_;
} // getTrr


} // namespace dre_slam
