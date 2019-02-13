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

#include <dre_slam/dynamic_pixel_detector.h>
#include <dre_slam/run_timer.h>

namespace dre_slam
{
DynamicPixelDetector::DynamicPixelDetector ( ObjectDetector* object_detector, Map* map, Camera* cam, Config* cfg ) : object_detector_ ( object_detector ), map_ ( map ), cam_ ( cam ), cfg_ ( cfg ), width_ ( cfg->cam_width_ ), height_ ( cfg->cam_height_ )
{
}

void DynamicPixelDetector::removeDynamicPixels ( KeyFrame* ckf )
{
    // Step 1. Detect objects using YOLOv3.
    std::vector<Object> dynamic_objects;
    detectDynamicObjects ( ckf->rgb_, cfg_->dpc_predef_dyn_obj_names_, cfg_->dpc_ob_scale_up_factor_, dynamic_objects );

    dynamicDepthImgCullingByDynamicObjects ( ckf->depth_, dynamic_objects );

    // draw objects
    ckf->image_objects_ = ckf->rgb_.clone();
    drawObjectsOnRgbImage ( dynamic_objects, ckf->image_objects_ );

    // If no keyframes in the map.
    if ( map_->getKFsSize() == 1 ) {
        return;
    }

    // Step 2. Segment the depth image using K-means.
    std::vector<Eigen::Vector2d> pts2d;
    std::vector<Eigen::Vector3d> pts3d;
    computePointCloud ( ckf->depth_, pts2d, pts3d );

    // If no enough pts remain.
    if ( pts2d.size() <= 10 ) {
        return;
    }

    // Calc Number of clusters.
    int K_clusters = std::ceil ( ( double ) pts2d.size() / ( double ) cfg_->dpc_npts_per_cluster_ );

    std::vector<std::vector<Eigen::Vector2d>> clusters_2d;
    std::vector<std::vector<Eigen::Vector3d>> clusters_3d;
    segmentPointCloudByKmeans ( pts2d, pts3d, K_clusters, clusters_2d, clusters_3d );

    // draw
    drawClustersOnImage ( ckf->image_source_clusters_, width_, height_, clusters_2d, colors_ );

    // Step 3. Detect dynamic clusters based on multi-view geometry consistant.
    std::vector<bool> dynamic_flags;
    detectDynamicClusters ( map_, ckf, clusters_2d, clusters_3d, dynamic_flags );

    dynamicDepthImgCullingByMultiViewConstraint ( ckf->depth_, clusters_2d, dynamic_flags );

    // draw.
//     ckf->image_static_clusters_ = ckf->image_source_clusters_.clone();
//     drawDynamicClusters ( ckf->image_static_clusters_, clusters_2d, dynamic_flags );

    // draw mask for visualization.
    drawResult ( ckf->depth_, dynamic_objects, clusters_2d, dynamic_flags, ckf->image_mask_ );
} // dynamicPixelsCulling


/*****************************************************************
 * Step 1. Detect Objects using YOLOv3
 *****************************************************************/
void DynamicPixelDetector::detectObjects ( const cv::Mat& rgb, std::vector<Object>& object )
{
    object.clear();
    img_objects_ = cv::Mat();
    object_detector_->detect ( rgb, img_objects_, object );
} // detectObjects


void DynamicPixelDetector::detectDynamicObjects ( const cv::Mat& rgb, const vector< string >& dynamic_object_names, const float expand_ratio, std::vector<Object>& dynamic_objects )
{
    // detect all objects in the rgb imaeg.
    std::vector<Object> objects;
    detectObjects ( rgb, objects );

    // filter out the dynamic objects and expand the bounding box.
    dynamic_objects.clear();
    for ( size_t i = 0; i < objects.size(); i ++ ) {
        Object& obj = objects.at ( i );
        if ( isObjectDynamic ( obj.name_, dynamic_object_names ) == true ) {
            // expandBoundingBox
            expandBoundingBox ( obj, expand_ratio );

            // push back the object to the vector of the dynamic objects .
            dynamic_objects.push_back ( obj );
        }
    }
} // detectDynamicObjects


bool DynamicPixelDetector::isObjectDynamic ( const string& object_name, const vector< string >& dynamic_object_names )
{
    for ( size_t i = 0; i < dynamic_object_names.size(); i ++ ) {
        const std::string& dy_name = dynamic_object_names.at ( i );

        if ( object_name == dy_name ) {
            return true;
        }
    }

    return false;
} // isObjectDynamic


void DynamicPixelDetector::expandBoundingBox ( Object& object, const float expand_ratio )
{
    cv::Rect& rect = object.rect_box_;

    cv::Point2i mid_point = 0.5 * ( rect.tl() + rect.br() );

    int expand_height = expand_ratio * rect.height;
    int expand_width = expand_ratio * rect.width;

    cv::Point2i new_tl ( mid_point + cv::Point2i ( -0.5*expand_width, -0.5*expand_height ) );
    cv::Point2i new_br ( mid_point + cv::Point2i ( 0.5*expand_width, 0.5*expand_height ) );

    // check bound
    if ( new_tl.x < 0 ) {
        new_tl.x = 0;
    }
    if ( new_tl.y < 0 ) {
        new_tl.y = 0;
    }
    if ( new_br.x >= width_ ) {
        new_br.x = width_ - 1;
    }
    if ( new_br.y >= height_ ) {
        new_br.y = height_ - 1;
    }

    object.rect_box_ = cv::Rect ( new_tl, new_br );
} //


void DynamicPixelDetector::drawObjectsOnRgbImage ( const vector< Object >& objects, cv::Mat& rgb )
{
    for ( size_t i = 0; i < objects.size(); i ++ ) {
        const Object& obj = objects.at ( i );
        cv::rectangle ( rgb, obj.rect_box_.tl(), obj.rect_box_.br(), cv::Scalar ( 0,0,255 ), 2 );
    }
} // drawObjectsOnRgbImage


void DynamicPixelDetector::dynamicDepthImgCullingByDynamicObjects ( cv::Mat& depth, const vector< Object >& dynamic_objects )
{
    for ( size_t i = 0; i < dynamic_objects.size(); i ++ ) {
        const Object& obj = dynamic_objects.at ( i );
        const cv::Rect& rect = obj.rect_box_;

        cv::Mat roi = depth ( rect );
        cv::Mat img ( roi.rows, roi.cols, CV_32F, cv::Scalar ( 0.0f ) );

        img.copyTo ( roi );
    }

} // dynamicDepthCullingByDynamicObjects


/*****************************************************************
 * Step2. Segment by Kmeans
 *****************************************************************/
void DynamicPixelDetector::computePointCloud ( const cv::Mat& depth, vector< Eigen::Vector2d >& pt2d, vector< Eigen::Vector3d >& pt3d )
{
    pt3d.clear();
    pt2d.clear();
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );
            if ( ( dp > cfg_->cam_dmin_ ) && ( dp < cfg_->cam_dmax_ ) ) {
                Eigen::Vector3d ptc = cam_->img2Cam ( Eigen::Vector2d ( u, v ), dp );
                pt3d.push_back ( ptc );
                pt2d.push_back ( Eigen::Vector2d ( u, v ) );
            }
        } // for all pixels.
    } //  for all pixels.
} // computePointCloud

void DynamicPixelDetector::segmentPointCloudByKmeans ( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d )
{
    // Convert
    cv::Mat points ( pts3d.size(), 3, CV_32F, cv::Scalar ( 0,0,0 ) );
    cv::Mat centers ( n_clusters, 1, points.type() );

    // Convert to opencv type
    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        const Eigen::Vector3d& ept = pts3d.at ( i );
        points.at<float> ( i, 0 ) = ept[0];
        points.at<float> ( i, 1 ) = ept[1];
        points.at<float> ( i, 2 ) = ept[2];
    } // for all points


    // Do Kmeans
    cv::Mat labels;
    cv::TermCriteria criteria = cv::TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0 );
    cv::kmeans ( points, n_clusters, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers );

    // Collect clusters.
    clusters_2d.clear();
    clusters_3d.clear();

    clusters_2d.resize ( n_clusters );
    clusters_3d.resize ( n_clusters );

    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        int label_idx = labels.at<int> ( i, 0 );
        clusters_2d.at ( label_idx ).push_back ( pts2d.at ( i ) );
        clusters_3d.at ( label_idx ).push_back ( pts3d.at ( i ) );
    }

} // segmentPointCloudByKmeans


void DynamicPixelDetector::drawClustersOnImage ( cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors )
{
    io_img = cv::Mat ( height, width, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );

    for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
        uint8_t r = colors.at ( i * 3 );
        uint8_t g = colors.at ( i * 3 + 1 );
        uint8_t b = colors.at ( i * 3 + 2 );

        // draw
        const std::vector<Eigen::Vector2d>& pts = clusters_2d.at ( i );
        for ( size_t j = 0; j < pts.size(); j ++ ) {
            const Eigen::Vector2d& pt = pts.at ( j );
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = r;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = g;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = b;
        }
    }
} // drawClustersOnImages



/*****************************************************************
 * Step 3. Detect dynamic clusters use multi-view geometry.
 *****************************************************************/
void DynamicPixelDetector::detectDynamicClusters ( Map* map, KeyFrame* ckf, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< vector< Eigen::Vector3d > >& clusters_3d, vector< bool >& dynamic_flags )
{
    // get last N keyframes.
    std::vector<KeyFrame*> near_kfs;
	map->getLastNKeyFrames ( cfg_->dpc_n_near_kfs_, false, near_kfs );

    for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
        const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );
        const std::vector<Eigen::Vector3d>& cluster_3d = clusters_3d.at ( i );
        bool is_dynamic = isDynamicCluster ( cluster_2d, cluster_3d, ckf, near_kfs );
        dynamic_flags.push_back ( is_dynamic );
    }
} // detectDynamicClusters

bool DynamicPixelDetector::isDynamicCluster ( const vector< Eigen::Vector2d >& cluster_2d, const vector< Eigen::Vector3d >& cluster_3d, KeyFrame* ckf, std::vector< KeyFrame* > near_kfs )
{
    const int n_test_points = cfg_->dpc_n_sel_pts_per_cluster_;
    const int half_search_square_size = cfg_->dpc_search_square_size_ / 2;
    //const double dist_th = cfg_->dpc_dist_threshold_;
	double dist_th = 0.0;

    std::vector<Eigen::Vector2d> test_pts_2d;
    std::vector<Eigen::Vector3d> test_pts_3d;

    // select n test points.
    if ( cluster_2d.size() <= n_test_points ) {
        test_pts_2d = cluster_2d;
        test_pts_3d = cluster_3d;
    } else {
        int step = cluster_2d.size()  / n_test_points;
        for ( size_t i = 0; i < cluster_2d.size(); i +=step ) {
            test_pts_2d.push_back ( cluster_2d.at ( i ) );
            test_pts_3d.push_back ( cluster_3d.at ( i ) );
        }
    }

    // project each selected points to every near keyframe for comparing.
    const Sophus::SE3& cTwr = ckf->getSE3Pose(); // current frame pose.
    const Sophus::SE2& cTwr2 = ckf->getSE2Pose();

    int n_unknown = 0;
    int n_static = 0;
    int n_dynamic = 0;

    for ( size_t np = 0; np < test_pts_2d.size(); np ++ ) {

        // points in current kf.
        const Eigen::Vector2d& pt2d = test_pts_2d.at ( np );
        const Eigen::Vector3d& pt3d = test_pts_3d.at ( np );
        const Eigen::Vector3d pt3dw = cTwr * cfg_->Trc_* pt3d; // point in 3d.

        PointType pt_type = UNKNOWN;

        int pt_n_unknown = 0;
        int pt_n_static = 0;
        int pt_n_dynamic = 0;

        for ( size_t nk = 0; nk < near_kfs.size(); nk ++ ) {
            // near kf.
            KeyFrame* nkf = near_kfs.at ( nk );
            const cv::Mat& ndepth = nkf->depth_;

            // continue when no depth.
            if ( ndepth.empty() ) {
                continue;
            }

            const Sophus::SE3& nTwr = nkf->getSE3Pose();
            const Sophus::SE2& nTwr2 = nkf->getSE2Pose();

            // project point 2 near kf.
            Eigen::Vector3d ptc = ( nTwr * cfg_->Trc_ ).inverse() * pt3dw;

			// Point must be in front of the KF.
            if ( ptc[2] < 0.1 ) {
                continue;
            }

            const Eigen::Vector2d nu = cam_->cam2Img ( ptc );

            // check if is in frame.
            if ( !cam_->isInFrame ( nu ) ) {
                continue;
            }

            // search in nxn square for best match.
            const double double_max = 1.0e10;
            double min_dist = double_max; // min_dist.
            double z_primer = 0.0;
            for ( int u = nu[0]-half_search_square_size; u<=nu[0]+half_search_square_size; u++ ) {
                for ( int v=nu[1]-half_search_square_size; v<=nu[1]+half_search_square_size; v++ ) {

                    // check if is in frame.
                    if ( !cam_->isInFrame ( Eigen::Vector2d ( u, v ) ) ) {
                        continue;
                    }

                    float dp = ndepth.at<float> ( v,u );
                    if ( ( dp > cfg_->cam_dmin_ ) && ( dp < cfg_->cam_dmax_ ) ) {
                        const Eigen::Vector3d nptc = cam_->img2Cam ( Eigen::Vector2d ( u, v ), dp );
                        const Eigen::Vector3d nptw = nTwr * cfg_->Trc_ * nptc;

                        // compute distance.
                        const double dist = ( nptw - pt3dw ).norm();
						
                        // update min_dist
                        if ( dist < min_dist ) {
                            min_dist = dist;
							z_primer = dp;
                        }
                    } // check if depth is ok.

                } // for a square
            } // for a square.

            // check if xx is static or dynamic.
            if ( min_dist != double_max ) {

				dist_th = cfg_->ret_tk_db_ + cfg_->ret_tk_kd_ * z_primer;
				
                if ( min_dist > dist_th ) {
                    pt_n_dynamic ++;
                    break;
                } else {
                    pt_n_static ++;
                }
            } else {
                pt_n_unknown ++;
            }
        } // for all near kfs.

        // define the pt_type
        if ( pt_n_static == 0 && pt_n_dynamic == 0 ) {
            pt_type = UNKNOWN;
        } else if ( pt_n_dynamic > pt_n_static ) {
            pt_type = DYNAMIC;
        } else {
            pt_type = STATIC;
        }

        // Count the number of different types of points
        if ( pt_type == UNKNOWN ) {
            n_unknown++;
        } else if ( pt_type == STATIC ) {
            n_static ++;
        } else if ( pt_type == DYNAMIC ) {
            n_dynamic ++;
        }

    } // for all test pts.

    // Decision the cluster type.
    if ( n_static == 0 && n_dynamic == 0 ) {
        return false;
    } else if ( n_dynamic >= n_static ) {
        return true;
    } else {
        return false;
    }

} // isDynamicCluster


void DynamicPixelDetector::drawDynamicClusters ( cv::Mat& io_img, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< bool >& dynamic_flags )
{
    for ( size_t i = 0; i < dynamic_flags.size(); i ++ ) {

        if ( dynamic_flags.at ( i ) ) {

            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );

            for ( int np = 0; np < cluster_2d.size(); np ++ ) {
                const Eigen::Vector2d& pt = cluster_2d.at ( np );

                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = 0;
                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = 0;
                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = 0;
            } // for all pixels in one cluster.
        } // is is dynamic
    } // for all clusters.

} // drawDynamicClusters


void DynamicPixelDetector::dynamicDepthImgCullingByMultiViewConstraint ( cv::Mat& depth, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< bool >& dynamic_flags )
{
    for ( size_t i = 0; i < dynamic_flags.size(); i ++ ) {

        if ( dynamic_flags.at ( i ) ) {

            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );

            for ( int np = 0; np < cluster_2d.size(); np ++ ) {
                const Eigen::Vector2d& pt = cluster_2d.at ( np );
                depth.at<float> ( pt[1], pt[0] ) = 0.0f;

            } // for all pixels in one cluster.
        } // is is dynamic
    } // for all clusters.
} // dynamicDepthImgCullingByMultiViewConstraint



void DynamicPixelDetector::convertDepth2Mask ( const cv::Mat& depth, cv::Mat& mask )
{
    mask = cv::Mat ( depth.rows, depth.cols, CV_8UC1, cv::Scalar ( 0 ) );

    for ( int u = 0; u < depth.cols; u ++ ) {
        for ( int v = 0; v < depth.rows; v++ ) {
            float dp = depth.at<float> ( v,u );
            if ( ( dp > cfg_->cam_dmin_ ) && ( dp < cfg_->cam_dmax_ ) ) {
                mask.at<uint8_t> ( v, u ) = 255;
            }
        }
    }
} // convertDepth2Mask


void DynamicPixelDetector::drawResult ( const cv::Mat& depth, const vector< Object >& dynamic_objects, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< bool >& dynamic_flags, cv::Mat& color_mask )
{
    const cv::Scalar color_static ( 255, 0, 0 );
    const cv::Scalar color_dynamic ( 0, 0, 255 );
    const cv::Scalar color_nodepth ( 0, 0, 0 );

    color_mask = cv::Mat ( height_, width_, CV_8UC3, color_static );


    // drawNoDepth pixels
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );
            if ( ( dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
                color_mask.at<cv::Vec3b> ( v,u ) [0] = color_nodepth[0];
                color_mask.at<cv::Vec3b> ( v,u ) [1] = color_nodepth[1];
                color_mask.at<cv::Vec3b> ( v,u ) [2] = color_nodepth[2];
            }
        } // for all pixels.
    } //  for all pixels.


    // draw dynamic objects.
    for ( size_t i = 0; i < dynamic_objects.size(); i ++ ) {
        const Object& obj = dynamic_objects.at ( i );
        const cv::Rect& rect = obj.rect_box_;

        cv::Mat roi = color_mask ( rect );
        cv::Mat img ( roi.rows, roi.cols, CV_8UC3, color_dynamic );

        img.copyTo ( roi );
    }

    // draw dynamic clusters.
    for ( size_t i = 0; i < dynamic_flags.size(); i ++ ) {

        if ( dynamic_flags.at ( i ) ) {

            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );

            for ( int np = 0; np < cluster_2d.size(); np ++ ) {
                const Eigen::Vector2d& pt = cluster_2d.at ( np );

                color_mask.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = color_dynamic[0];
                color_mask.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = color_dynamic[1];
                color_mask.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = color_dynamic[2];
            } // for all pixels in one cluster.
        } // is is dynamic
    } // for all clusters.
}


} // namespace dreslam
