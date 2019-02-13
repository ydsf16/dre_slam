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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <dre_slam/frame.h>
#include <dre_slam/map_point.h>
#include <dre_slam/encoder_integration.h>
#include <dre_slam/vocabulary.h>
#include <mutex>

namespace dre_slam
{
class MapPoint;
class Frame;

class KeyFrame
{
public:
    KeyFrame( Frame* frame , Vocabulary* voc);

    // pose
    void cvtEigen2Double();
    void cvtDouble2Eigen();
    void setPose(const Sophus::SE2& Twr);
    Sophus::SE2 getSE2Pose();
    Sophus::SE3 getSE3Pose();

    // Pose graph
    void addLastKeyFrameEdge(KeyFrame* kf);
    void addVisualEdge(KeyFrame* kf);
    
    std::set<KeyFrame*> getObKFs();
    void setObKFs(const std::set<KeyFrame*>& ob_kfs);

    void addLoopEdge(KeyFrame* kf, const Sophus::SE2& T);
    KeyFrame* getLastKeyFrameEdge();
    std::set<KeyFrame*> getVisualEdge();
    int getLoopEdge(std::vector<KeyFrame*>& loop_kfs, std::vector<Sophus::SE2>& Ts);

    // Optimization
    void calculateRelativePosition();
    void reCalculateMpts();
    void reCalculateSingleMpts();
    void freeMemory();
    void freeDepth();
    
    // select static features.
    void selectStaticFeatures();
    
    
public:
    // Number of all keyframes.
    static long int N_KFS;

    // Read only data.
    long int kfid_;
    long int fid_;
    double timestamp_;

    // The static KeyPoints and descriptors.
    std::vector<cv::KeyPoint> kps_;
    std::vector<bool> static_flags_;
    cv::Mat des_;
    
    Vocabulary* voc_;
    DBoW2::BowVector bow_vec_;
    DBoW2::FeatureVector feat_vec_;
    
    int n_features_;
    std::vector<double> dps_;
    std::vector<MapPoint*> mpts_; 
    
    std::vector<Eigen::Vector3d> pts_r_; 

    Camera* cam_;
    Config* cfg_;
    cv::Mat rgb_;
    cv::Mat depth_;
    
    // segmentation used.
    cv::Mat image_mask_;
    cv::Mat image_objects_;
    cv::Mat image_source_clusters_;
    cv::Mat image_static_clusters_;

    Sophus::SE2 Trr_; 
    Eigen::Matrix3d covrr_; 

    double Twrd_[3];
    
private:
    Sophus::SE2 Twr2_; // robot pose
    Sophus::SE3 Twr3_; // robot pose

    // Pose graph connections.
    KeyFrame* last_kf_; // The last frame.
    std::set<KeyFrame*> ob_kfs_; // Cov keyframes.
    std::vector<KeyFrame*> loop_kfs_; // Loop Frames.
    std::vector<Sophus::SE2> loop_delta_pose_;

    std::mutex mutex_pose_;
    std::mutex mutex_graph_;
    std::mutex mute_mpts_;

}; // class KeyFrame

} // namespace dre_slam


#endif
