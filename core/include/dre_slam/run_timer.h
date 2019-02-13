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

#ifndef RUN_TIMER_H
#define RUN_TIMER_H

#include <chrono>
#include <iostream>

namespace dre_slam
{

class RunTimer
{
public:
    inline void start() {
        t_s_  = std::chrono::steady_clock::now();
    }

    inline void stop() {
        t_e_ = std::chrono::steady_clock::now();
    }

    inline double duration() {
        return std::chrono::duration_cast<std::chrono::duration<double>> ( t_e_ - t_s_ ).count() * 1000.0;
    }

private:
    std::chrono::steady_clock::time_point t_s_; //start time ponit
    std::chrono::steady_clock::time_point t_e_; //stop time point
};


} //namespace dre_slam

#endif
