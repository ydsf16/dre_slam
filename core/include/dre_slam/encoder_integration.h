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

#ifndef ENCODER_INTEGRATION_H
#define ENCODER_INTEGRATION_H

#include <sophus/se2.h>
#include <dre_slam/encoder.h>

namespace dre_slam
{
class EncoderIntegration
{
public:
	EncoderIntegration(){}
	EncoderIntegration(const double& kl, const double& kr, const double& b, const double& k);
    Sophus::SE2 addEncoder(const Encoder& enc); // 
    Sophus::SE2 addEncoders(const std::vector< Encoder >& encs);

    Sophus::SE2 getTrr();
    Eigen::Matrix3d getCov();

private:
    double kl_, kr_, b_, k_;

    Sophus::SE2 Trr_;
    Eigen::Matrix3d cov_;

    bool is_init_;
    Encoder last_enc_;
    double x_, y_, th_;
    
}; // class EncoderIntegratation

} // namespace dre_slam

#endif
