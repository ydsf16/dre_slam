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

#ifndef ENCODER_H
#define ENCODER_H

namespace dre_slam
{

class Encoder
{
public:
    Encoder(): enl_(0.0), enr_(0.0), timestamp_(0.0){}
    Encoder(const double& enl, const double& enr, const double& timestamp):
    enl_(enl), enr_(enr), timestamp_(timestamp) {}
    
    double enl_, enr_, timestamp_;
}; // class Encoder

} // namespace dre_slam

#endif
