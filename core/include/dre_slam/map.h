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

#ifndef MAP_H
#define MAP_H

#include <pcl/kdtree/kdtree_flann.h>
#include <dre_slam/config.h>
#include <dre_slam/map_point.h>
#include <dre_slam/keyframe.h>

namespace dre_slam
{

class Map
{
public:
    Map(Config* cfg);
    
    void addMapPoint(MapPoint* mpt); 
    void addKeyFrame(KeyFrame* kf); 
    void updateKFKDTree(); 
    
    //Get last N keyframes for dynamic pixel culling and local sliding window BA.
	int getLastNKeyFrames( int N, bool keep_last, std::vector<KeyFrame*>& last_n_kfs);
    
	// Get Local MapPoints for RGB-D Encoder Tracking.
	// Keyframes are sorted by distance from near to far
	int getLocalKeyFrames( Frame* frame, double dist_th, double angle_th, std::vector<KeyFrame*>& local_kfs );
	int getLocalMappoints( Frame* frame, double dist_th, double angle_th, int max_n_mpts, std::set< MapPoint* >& local_mpts);
	
	// Get near KFs in dist and angle radius for loop closure. the near kfs must be befor N kfs.
	int getNearKeyFrames( KeyFrame* kf, double dist_th, double angle_th, int npassed_th, std::vector<KeyFrame*>& near_kfs );
	
	// Get n befor and n after kfs around a KF.
	int getNeighbourKeyFrames(KeyFrame* lkf, int n, std::vector< KeyFrame* >& kfs );
    
    // Get map data.
    int getKFsSize();
    void addFrame(Frame* frame);
    std::vector<Frame*> getAllFrames();
    std::set<MapPoint*> getAllMapPoints();
    std::map<long int, KeyFrame*> getAllKeyFrames();

    // For map update.
    std::mutex update_mutex_;
	
private:
    Config* cfg_;
    std::set<MapPoint*> mpts_; // all map point.
    std::map<long int, KeyFrame*> kfs_; // all keyframes, with their kf id.
    pcl::KdTreeFLANN<pcl::PointXY> KDTree_kfs_; // KD-Tree of the 2D positions of the KFs.
    std::vector<Frame*> frames_; // all frames.
    std::mutex mutex_map_; // For map change.
}; //class Map

} //namespace dre_slam

#endif

