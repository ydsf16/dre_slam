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

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <opencv2/opencv.hpp>

class Object{
public:
	Object(){};
	std::string name_;
	int id_;
	cv::Rect rect_box_;
	float score_;
}; //class Objects


class ObjectDetector{
public:
	ObjectDetector( const std::string& classes_file, const std::string& model_config, const std::string& model_weights );
	int detect( const cv::Mat& imag_in, cv::Mat& image_out, std::vector< Object >& objects);

private:
	std::vector<std::string> classes_;
}; //class ObjectDetector


#endif
