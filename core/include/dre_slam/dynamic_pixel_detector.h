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

#ifndef DYNAMIC_PIXEL_DETECTOR_H
#define DYNAMIC_PIXEL_DETECTOR_H

#include <dre_slam/map.h>
#include <dre_slam/camera.h>
#include <dre_slam/config.h>
#include <object_detector/object_detector.h>

namespace dre_slam {
	
enum PointType{
	UNKNOWN,
	STATIC,
	DYNAMIC
};

class DynamicPixelDetector{
public:
	DynamicPixelDetector( ObjectDetector* object_detector, Map* map, Camera* cam, Config* cfg );
	
	void removeDynamicPixels( KeyFrame* ckf );
	
private:
	// detect Objects by YOLOv3
	void detectObjects( const cv::Mat& rgb, std::vector<Object>& object ); 
	void detectDynamicObjects(const cv::Mat& rgb, const std::vector<std::string>& dynamic_object_names, const float expand_ratio, std::vector<Object>& dynamic_objects);
	bool isObjectDynamic(const std::string& object_name, const std::vector<std::string>& dynamic_object_names);
	void expandBoundingBox(Object& object, const float expand_ratio);
	void drawObjectsOnRgbImage(const std::vector<Object>& objects, cv::Mat& rgb);
	void dynamicDepthImgCullingByDynamicObjects(cv::Mat& depth, const std::vector<Object>& dynamic_objects);
	
	// Kmeans
	void computePointCloud( const cv::Mat& depth, std::vector<Eigen::Vector2d>& pt2d, std::vector<Eigen::Vector3d>& pt3d );
	void segmentPointCloudByKmeans( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d );
	void drawClustersOnImage(cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors);
	
	// Detect dynamic clusters.
	void detectDynamicClusters( Map* map, KeyFrame* ckf, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< vector< Eigen::Vector3d > >& clusters_3d, std::vector<bool>& dynamic_flags );
	bool isDynamicCluster( const vector< Eigen::Vector2d >& cluster_2d, const vector< Eigen::Vector3d >& cluster_3d, KeyFrame* ckf, std::vector< KeyFrame* > near_kfs );
	void dynamicDepthImgCullingByMultiViewConstraint( cv::Mat& depth, const std::vector<std::vector<Eigen::Vector2d>>& clusters_2d, const std::vector<bool>& dynamic_flags );
	void drawDynamicClusters(cv::Mat& io_img, const std::vector<std::vector<Eigen::Vector2d>>& clusters_2d, const std::vector<bool>& dynamic_flags);
	
	// Convert depth to mask.
	void convertDepth2Mask(const cv::Mat& depth, cv::Mat& mask);
	
	// Paper writing
	void drawResult( const cv::Mat& depth, const std::vector<Object>& dynamic_objects, const std::vector<std::vector<Eigen::Vector2d>>& clusters_2d, const std::vector<bool>& dynamic_flags, cv::Mat& color_mask);
	
private:
	Map* map_;
	Camera* cam_;
	Config* cfg_;
	ObjectDetector* object_detector_;
	int width_;
	int height_;
	std::vector<Object> objects_;
	cv::Mat img_objects_;
	std::vector<uint8_t> colors_ = {213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80, 213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80};
	
}; // class DynamicPixelDetector	
	
} // namespace dre_slam


#endif