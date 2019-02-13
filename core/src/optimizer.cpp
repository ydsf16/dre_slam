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

#include <dre_slam/optimizer.h>
#include <dre_slam/keyframe.h>

namespace dre_slam
{

template <typename T>
inline T NormalizeAngle ( const T& angle_radians )
{
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi ( 2.0 * M_PI );
    return angle_radians - two_pi * ceres::floor ( ( angle_radians + T ( M_PI ) ) / two_pi );
}

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class AngleLocalParameterization
{
public:

    template <typename T>
    bool operator() ( const T* theta_radians, const T* delta_theta_radians,
                      T* theta_radians_plus_delta ) const {
        *theta_radians_plus_delta =
            NormalizeAngle ( *theta_radians + *delta_theta_radians );

        return true;
    }

    static ceres::LocalParameterization* Create() {
        return ( new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                 1, 1> );
    }
};

// sqrtMatrix
template <typename T>
T Optimizer::sqrtMatrix ( const T& TT )
{
    Eigen::JacobiSVD<T> svd ( TT, Eigen::ComputeFullU | Eigen::ComputeFullV );

    T V = svd.matrixV(), U = svd.matrixU();
    T  S = U.inverse() * TT * V.transpose().inverse();

    T sqrt_S;
    sqrt_S << sqrt ( S ( 0, 0 ) ), 0., 0.,
           0., sqrt ( S ( 1, 1 ) ), 0.,
           0., 0., sqrt ( S ( 2, 2 ) );

    T sT = sqrt_S * V.transpose();
    if ( std::isnan ( ( double ) sT ( 0,0 ) ) ||
            std::isnan ( ( double ) sT ( 0,1 ) ) ||
            std::isnan ( ( double ) sT ( 0,2 ) ) ||
            std::isnan ( ( double ) sT ( 1,0 ) ) ||
            std::isnan ( ( double ) sT ( 1,1 ) ) ||
            std::isnan ( ( double ) sT ( 1,2 ) ) ||
            std::isnan ( ( double ) sT ( 2,0 ) ) ||
            std::isnan ( ( double ) sT ( 2,1 ) ) ||
            std::isnan ( ( double ) sT ( 2,2 ) )
       ) {
        sT = T::Identity() * 1e8;
    }

    return sT;
}// sqrtMatrix

double getVisualSqrtInfo ( int octave, double scale_factor )
{
    double scale = 1.0;
    for ( int i = 0; i < octave; i ++ ) {
        scale *= scale_factor;
    }
    return 1.0 / scale;
}

class ProjectionError
{
public:
    ProjectionError ( const Eigen::Matrix4d& T_c_r,
                      const double& fx, const double& fy, const double& cx, const double& cy,
                      const cv::KeyPoint& kp,  const Eigen::Matrix2d& sqrt_info ) :
        T_c_r_ ( T_c_r ), fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ),
        kp_ ( kp ), sqrt_info_ ( sqrt_info ) {}

    template <typename T>
    bool operator() ( const T* const rx,const T* const ry,const T* const rth,
                      const T* const pwx, const T* const pwy, const T* const pwz,
                      T* residuals ) const {
        T c00 = T ( T_c_r_ ( 0,0 ) );
        T c01 = T ( T_c_r_ ( 0,1 ) );
        T c02 = T ( T_c_r_ ( 0,2 ) );
        T c03 = T ( T_c_r_ ( 0,3 ) );
        T c10 = T ( T_c_r_ ( 1,0 ) );
        T c11 = T ( T_c_r_ ( 1,1 ) );
        T c12 = T ( T_c_r_ ( 1,2 ) );
        T c13 = T ( T_c_r_ ( 1,3 ) );
        T c20 = T ( T_c_r_ ( 2,0 ) );
        T c21 = T ( T_c_r_ ( 2,1 ) );
        T c22 = T ( T_c_r_ ( 2,2 ) );
        T c23 = T ( T_c_r_ ( 2,3 ) );

        T fx = T ( fx_ );
        T fy = T ( fy_ );
        T cx = T ( cx_ );
        T cy = T ( cy_ );

        T px = pwx[0];
        T py = pwy[0];
        T pz = pwz[0];

        T ob_u = T ( kp_.pt.x );
        T ob_v = T ( kp_.pt.y );

        T x = rx[0];
        T y = ry[0];
        T th = rth[0];

        T cth = cos ( th );
        T sth = sin ( th );

        T lamda_u =  px* ( cx* ( c20*cth - c21*sth ) + fx* ( c00*cth - c01*sth ) )
                     + py* ( cx* ( c21*cth + c20*sth ) + fx* ( c01*cth + c00*sth ) ) -
                     cx* ( c20* ( x*cth + y*sth ) - c23 + c21* ( y*cth - x*sth ) ) -
                     fx* ( c00* ( x*cth + y*sth ) - c03 + c01* ( y*cth - x*sth ) ) + pz* ( c22*cx + c02*fx );

        T lamda_v = px* ( cy* ( c20*cth - c21*sth ) + fy* ( c10*cth -
                          c11*sth ) ) + py* ( cy* ( c21*cth + c20*sth ) + fy* ( c11*cth +
                                              c10*sth ) ) - cy* ( c20* ( x*cth + y*sth ) - c23 + c21* ( y*cth -
                                                      x*sth ) ) - fy* ( c10* ( x*cth + y*sth ) - c13 + c11* ( y*cth -
                                                              x*sth ) ) + pz* ( c22*cy + c12*fy );

        T lamda = c23 + c22*pz + px* ( c20*cth - c21*sth ) + py* ( c21*cth + c20*sth )
                  - c20* ( x*cth + y*sth ) - c21* ( y*cth - x*sth );

        T u = lamda_u / lamda;
        T v = lamda_v / lamda;

        T e1 = ob_u - u;
        T e2 = ob_v - v;
        residuals[0] = T ( sqrt_info_ ( 0,0 ) ) * e1 + T ( sqrt_info_ ( 0,1 ) ) * e2;
        residuals[1] = T ( sqrt_info_ ( 1,0 ) ) * e1 + T ( sqrt_info_ ( 1,1 ) ) * e2;

        return true;
    }

    static ceres::CostFunction* Create ( const Eigen::Matrix4d& T_c_r,
                                         const double& fx, const double& fy, const double& cx, const double& cy,
                                         const cv::KeyPoint& kp,  const Eigen::Matrix2d& sqrt_info ) {
        return ( new ceres::AutoDiffCostFunction<ProjectionError, 2, 1, 1, 1, 1, 1, 1>
                 ( new ProjectionError ( T_c_r, fx, fy, cx, cy, kp,  sqrt_info ) ) );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Matrix4d T_c_r_;
    double fx_, fy_, cx_, cy_;
    cv::KeyPoint kp_;
    Eigen::Matrix2d sqrt_info_;
};

class EncoderFrame2FrameError
{
public:
    EncoderFrame2FrameError ( const double& delta_x, const double& delta_y, const double& delta_theta, const Eigen::Matrix3d sqrt_info ) :
        delta_x_ ( delta_x ), delta_y_ ( delta_y ), delta_theta_ ( delta_theta ), sqrt_info_ ( sqrt_info ) {}

    template <typename T>
    bool operator() ( const T* const ref_x, const T* const ref_y, const T* const ref_th,
                      const T* const cur_x, const T* const cur_y, const T* const cur_th,
                      T* residuals ) const {

        // ref pose
        T xr = ref_x[0];
        T yr = ref_y[0];
        T thr = ref_th[0];

        // cur pose
        T xc = cur_x[0];
        T yc = cur_y[0];
        T thc = cur_th[0];

        T ob_dx = T ( delta_x_ );
        T ob_dy = T ( delta_y_ );
        T ob_dth = T ( delta_theta_ );

        T tmp_dx = xc - xr;
        T tmp_dy = yc - yr;

        T dx = cos ( thr ) *tmp_dx + sin ( thr ) *tmp_dy;
        T dy = -sin ( thr ) *tmp_dx + cos ( thr ) *tmp_dy;

        T ex = dx - ob_dx;
        T ey = dy - ob_dy;
        T eth = NormalizeAngle ( thc - thr - ob_dth );

        residuals[0] = T ( sqrt_info_ ( 0,0 ) ) * ex + T ( sqrt_info_ ( 0,1 ) ) * ey + T ( sqrt_info_ ( 0,2 ) ) * eth ;
        residuals[1] = T ( sqrt_info_ ( 1,0 ) ) * ex + T ( sqrt_info_ ( 1,1 ) ) * ey + T ( sqrt_info_ ( 1,2 ) ) * eth ;
        residuals[2] = T ( sqrt_info_ ( 2,0 ) ) * ex + T ( sqrt_info_ ( 2,1 ) ) * ey + T ( sqrt_info_ ( 2,2 ) ) * eth ;

        return true;
    }
    static ceres::CostFunction* Create ( const double& delta_x, const double& delta_y, const double& delta_theta, const Eigen::Matrix3d sqrt_info ) {
        return ( new ceres::AutoDiffCostFunction<EncoderFrame2FrameError, 3, 1, 1, 1, 1, 1, 1>
                 ( new EncoderFrame2FrameError ( delta_x, delta_y, delta_theta, sqrt_info ) ) );
    }

private:
    const double delta_x_, delta_y_, delta_theta_;
    const Eigen::Matrix3d sqrt_info_;
}; //class


/***************************************************************************************
 * class Optimizer
 * **************************************************************************************/
void Optimizer::motionOnlyBA ( Frame* frame, KeyFrame* ref_kf, EncoderIntegration& encoder_kf2f_ )
{
    /* Parameters */
    double& fx = cfg_->cam_fx_;
    double& fy = cfg_->cam_fy_;
    double& cx = cfg_->cam_cx_;
    double& cy = cfg_->cam_cy_;
    const Eigen::Matrix4d& Tcr = cfg_->Trc_.inverse().matrix();

    /* Copy data */
    Sophus::SE2 f_Twr2 = frame->getSE2Pose();
    double frame_pose[3] = {f_Twr2.translation() ( 0 ), f_Twr2.translation() ( 1 ), f_Twr2.so2().log() };
    Sophus::SE2 ref_kf_Twr = ref_kf->getSE2Pose();
    double  ref_kf_pose[3] = {ref_kf_Twr.translation() ( 0 ), ref_kf_Twr.translation() ( 1 ), ref_kf_Twr.so2().log() };

    double mpts[  frame->n_matched_  *3 ];
    int idx = 0; // map point id.
    for ( size_t i = 0; i < frame->mpts_.size(); i ++ ) {
        MapPoint* mpt = frame->mpts_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }
        Eigen::Vector3d ps = mpt->getPosition();

        mpts[ idx*3+0 ] = ps ( 0 );
        mpts[ idx*3+1 ] = ps ( 1 );
        mpts[ idx*3+2 ] = ps ( 2 );

        idx ++;
    }// for all mpts

    // loss function
    ceres::LossFunction* visual_loss = new ceres::HuberLoss ( 2.5 );
    ceres::LossFunction* encoder_loss = new ceres::HuberLoss ( 0.5 );
    ceres::LocalParameterization* angle_local_parameterization =
        AngleLocalParameterization::Create();


    ceres::Problem problem;

    // add projection errors.
    idx = 0;
    for ( size_t i = 0; i < frame->mpts_.size(); i ++ ) {
        MapPoint* mpt = frame->mpts_.at ( i );
        cv::KeyPoint& kp = frame->kps_.at ( i );
        if ( mpt == NULL ) {
            continue;
        }

        Eigen::Matrix2d v_sqrt_info = getVisualSqrtInfo ( kp.octave, cfg_->ret_ft_scale_factor_ ) * Eigen::Matrix2d::Identity();

        // cost function
        ceres::CostFunction* cost_function =
            ProjectionError::Create ( Tcr, fx, fy, cx, cy, kp, v_sqrt_info );

        problem.AddResidualBlock (
            cost_function,
            visual_loss ,
            frame_pose, ( frame_pose+1 ), ( frame_pose+2 ),
            mpts+ 3 * idx , mpts + 3 * idx + 1 , mpts + 3 * idx + 2
        );

        // set mpts fixed
        problem.SetParameterBlockConstant ( mpts + ( 3 * idx ) );
        problem.SetParameterBlockConstant ( mpts + ( 3 * idx + 1 ) );
        problem.SetParameterBlockConstant ( mpts + ( 3 * idx + 2 ) );

        idx ++;
    }// add reprojection error terms

    // local parameterization angle
    problem.SetParameterization ( frame_pose + 2 , angle_local_parameterization );

    // add encoder errors.
	Eigen::Matrix3d o_sqrt_info = sqrtMatrix<Eigen::Matrix3d> ( encoder_kf2f_.getCov().inverse() );
    ceres::CostFunction* cost_function =
    EncoderFrame2FrameError::Create ( encoder_kf2f_.getTrr().translation() ( 0 ), encoder_kf2f_.getTrr().translation() ( 1 ), encoder_kf2f_.getTrr().so2().log(), o_sqrt_info );
    problem.AddResidualBlock ( cost_function,
                               encoder_loss,
                               ref_kf_pose, ref_kf_pose + 1, ref_kf_pose +2,
                               frame_pose, frame_pose + 1,  frame_pose +2
                             );

    // Local parameterization
    problem.SetParameterization ( ref_kf_pose + 2 , angle_local_parameterization );

    // Set reference kf fixed
    problem.SetParameterBlockConstant ( ref_kf_pose );
    problem.SetParameterBlockConstant ( ref_kf_pose + 1 );
    problem.SetParameterBlockConstant ( ref_kf_pose + 2 );


    /* Do optimization */
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.gradient_tolerance = 1e-10;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    options.num_threads = 1;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );

    // Set back the result.
    frame->setPose ( Sophus::SE2 ( frame_pose[2], Eigen::Vector2d ( frame_pose[0], frame_pose[1] ) ) );
} // motionOnlyBA



void Optimizer::localBA ( std::vector< KeyFrame* >& opt_kfs )
{
    /* Parameters */
    double& fx = cfg_->cam_fx_;
    double& fy = cfg_->cam_fy_;
    double& cx = cfg_->cam_cx_;
    double& cy = cfg_->cam_cy_;
    const Eigen::Matrix4d& Tcr = cfg_->Trc_.inverse().matrix();

    // min optimize keyframe id.
    long int min_opt_kf_id = opt_kfs.at ( opt_kfs.size()-1 )->kfid_;

    // loss function
    ceres::LossFunction* visual_loss = new ceres::HuberLoss ( 2.5 );
    ceres::LossFunction* encoder_loss = new ceres::HuberLoss ( 0.5 );
    ceres::LocalParameterization* angle_local_parameterization =
        AngleLocalParameterization::Create();

    ceres::Problem problem;
	
    for ( size_t i = 0; i < opt_kfs.size(); i ++ ) {

        // add visual error terms.
        KeyFrame* kf = opt_kfs.at ( i );
        kf->cvtEigen2Double();

        // for all mappoints.
        for ( size_t j = 0; j < kf->mpts_.size(); j ++ ) {
            // discard null mpts.
            MapPoint* mpt = kf->mpts_.at ( j );
            if ( mpt == NULL ) {
                continue;
            }
            // discard single view mpts.
            if ( mpt->ob_kfs_.size() == 1 ) {
                continue;
            }

            mpt->cvtEigen2Double();

            // for all view keyframes.
            std::map<KeyFrame*, int>& ob_kfs = mpt->ob_kfs_;
            std::map<KeyFrame*, int>::iterator iter = ob_kfs.begin();
            for ( ; iter != ob_kfs.end(); iter ++ ) {

                KeyFrame* ob_kf = iter->first;
                ob_kf->cvtEigen2Double();

                int idx = iter->second;

                // add visual error.
                cv::KeyPoint& kp = ob_kf->kps_.at ( idx );

                // get information matrix.
                Eigen::Matrix2d v_sqrt_info = getVisualSqrtInfo ( kp.octave, cfg_->ret_ft_scale_factor_ ) * Eigen::Matrix2d::Identity();

                // cost function
                ceres::CostFunction* cost_function =
                    ProjectionError::Create ( Tcr, fx, fy, cx, cy,  kp, v_sqrt_info );

                problem.AddResidualBlock (
                    cost_function,
                    visual_loss ,
                    ob_kf->Twrd_, ob_kf->Twrd_+1, ob_kf->Twrd_+2,
                    mpt->ptwd_, mpt->ptwd_ + 1, mpt->ptwd_ + 2
                );

                if ( ob_kf->kfid_ < min_opt_kf_id ) {
                    problem.SetParameterBlockConstant ( ob_kf->Twrd_ );
                    problem.SetParameterBlockConstant ( ob_kf->Twrd_ + 1 );
                    problem.SetParameterBlockConstant ( ob_kf->Twrd_ + 2 );
                }

            } // for all viewed keyframes.

            // if the mpt is created by fixed kf, fixed it.
            if ( mpt->first_ob_kf_->kfid_ < min_opt_kf_id ) {
                problem.SetParameterBlockConstant ( mpt->ptwd_ );
                problem.SetParameterBlockConstant ( mpt->ptwd_ + 1 );
                problem.SetParameterBlockConstant ( mpt->ptwd_ + 2 );
            }

            const double range  = 5.0;
            problem.SetParameterLowerBound ( ( mpt->ptwd_ ), 0, * ( mpt->ptwd_ ) - range );
            problem.SetParameterLowerBound ( ( mpt->ptwd_ + 1 ), 0, * ( mpt->ptwd_ + 1 ) - range );
            problem.SetParameterLowerBound ( ( mpt->ptwd_ + 2 ), 0, * ( mpt->ptwd_ + 2 ) - range );

            problem.SetParameterUpperBound ( ( mpt->ptwd_ ), 0, * ( mpt->ptwd_ ) + range );
            problem.SetParameterUpperBound ( ( mpt->ptwd_ + 1 ), 0, * ( mpt->ptwd_ + 1 ) + range );
            problem.SetParameterUpperBound ( ( mpt->ptwd_ + 2 ), 0, * ( mpt->ptwd_ + 2 ) + range );

        } // for all map points.


        // add encoder error terms.
        KeyFrame* rkf = kf->getLastKeyFrameEdge();
        rkf->cvtEigen2Double();

        Eigen::Matrix3d o_sqrt_info = sqrtMatrix<Eigen::Matrix3d> ( kf->covrr_.inverse() );

        ceres::CostFunction* cost_function =
            EncoderFrame2FrameError::Create ( kf->Trr_.translation() ( 0 ), kf->Trr_.translation() ( 1 ), kf->Trr_.so2().log(), o_sqrt_info );

        problem.AddResidualBlock ( cost_function,
                                   encoder_loss,
                                   rkf->Twrd_, rkf->Twrd_ + 1, rkf->Twrd_ +2,
                                   kf->Twrd_, kf->Twrd_ + 1, kf->Twrd_ +2
                                 );

        problem.SetParameterization ( kf->Twrd_ + 2 , angle_local_parameterization );
        problem.SetParameterization ( rkf->Twrd_ + 2 , angle_local_parameterization );

        if ( rkf->kfid_ < min_opt_kf_id ) {
            problem.SetParameterBlockConstant ( rkf->Twrd_ );
            problem.SetParameterBlockConstant ( rkf->Twrd_ + 1 );
            problem.SetParameterBlockConstant ( rkf->Twrd_ + 2 );
        }

    } // for all optimize keyframes.
    
    // Solve.
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.max_num_iterations = cfg_->sm_lm_lba_niter;
    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );
	
    // reset keyframe pose and single view keypoints.
    for ( size_t i = 0; i < opt_kfs.size(); i ++ ) {
        KeyFrame* kf = opt_kfs.at ( i );
		
        // Reset the kf.
        kf->calculateRelativePosition();
        kf->cvtDouble2Eigen();
        kf->reCalculateSingleMpts();
		
        // Reset optimized mpts.
        for ( MapPoint* mpt: kf->mpts_ ) {
            if ( mpt !=NULL ) {
                if ( mpt->ob_kfs_.size() == 1 ) {
                    continue;
                }

                if ( mpt->first_ob_kf_->kfid_ < min_opt_kf_id ) {
                    continue;
                }
                mpt->cvtDouble2Eigen();
            }
        }
    } // for all opt_kfs
} // local BA


void Optimizer::motionOnlyBA ( vector< cv::KeyPoint >& kps, std::vector< MapPoint* >& mpts, Sophus::SE2& pose )
{
    /* Params */
    double& fx = cfg_->cam_fx_;
    double& fy = cfg_->cam_fy_;
    double& cx = cfg_->cam_cx_;
    double& cy = cfg_->cam_cy_;
    const Eigen::Matrix4d& Tcr = cfg_->Trc_.inverse().matrix();

    // loss function
    ceres::LossFunction* visual_loss = new ceres::HuberLoss ( 5 );

    // local params
    ceres::LocalParameterization* angle_local_parameterization =
        AngleLocalParameterization::Create();

    double pd3[3] = {pose.translation() ( 0 ), pose.translation() ( 1 ), pose.so2().log() };

    ceres::Problem problem;

    for ( size_t i = 0; i < mpts.size(); i ++ ) {
        MapPoint* mpt = mpts.at ( i );
        if ( mpt == NULL ) {
            continue;
        }
        mpt->cvtEigen2Double();
        cv::KeyPoint& kp = kps.at ( i );


        Eigen::Matrix2d v_sqrt_info = getVisualSqrtInfo ( kp.octave, cfg_->ret_ft_scale_factor_ ) * Eigen::Matrix2d::Identity();

        // cost function
        ceres::CostFunction* cost_function =
            ProjectionError::Create ( Tcr, fx, fy, cx, cy,  kp, v_sqrt_info );

        problem.AddResidualBlock (
            cost_function,
            visual_loss ,
            pd3, ( pd3+1 ), ( pd3+2 ),
            mpt->ptwd_, mpt->ptwd_ +1 , mpt->ptwd_+2
        );

        // set mpts fixed
        problem.SetParameterBlockConstant ( mpt->ptwd_ );
        problem.SetParameterBlockConstant ( mpt->ptwd_ + 1 );
        problem.SetParameterBlockConstant ( mpt->ptwd_ + 2 );
    }// for all mpts

    problem.SetParameterization ( pd3 + 2 , angle_local_parameterization );

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.max_num_iterations = 40;
    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );

    pose = Sophus::SE2 ( pd3[2], Eigen::Vector2d ( pd3[0],pd3[1] ) ) ;
} // motionOnlyBA


// Only optimize KFs in the loop.
void Optimizer::poseGraphOptimization ( KeyFrame* loop_kf )
{
    std::map<long int, KeyFrame*> kfs = map_->getAllKeyFrames();
    std::map<long int, KeyFrame*>::iterator it = ( ++kfs.begin() );

    ceres::LossFunction* loss = new ceres::HuberLoss ( 0.5 );
    ceres::LocalParameterization* angle_local_parameterization =
        AngleLocalParameterization::Create();
	
    ceres::Problem problem;
    for ( ; it != kfs.end(); it ++ ) {

        KeyFrame* kf = it->second;
        if ( kf->kfid_ <= loop_kf->kfid_ ) {
            continue;
        }
   
        // encoder edge
        KeyFrame* lkf = kf->getLastKeyFrameEdge();
		
        kf->cvtEigen2Double();
        lkf->cvtEigen2Double();
		
        ceres::CostFunction* cost_function =
            EncoderFrame2FrameError::Create ( kf->Trr_.translation() ( 0 ), kf->Trr_.translation() ( 1 ), kf->Trr_.so2().log(), cfg_->sm_lc_encoder_edge_weight_ *Eigen::Matrix3d::Identity() );
        problem.AddResidualBlock ( cost_function,
                                   loss,
                                   lkf->Twrd_, lkf->Twrd_ + 1, lkf->Twrd_ +2,
                                   kf->Twrd_, kf->Twrd_ + 1, kf->Twrd_ +2
                                 );

        problem.SetParameterization ( kf->Twrd_ + 2 , angle_local_parameterization );
        problem.SetParameterization ( lkf->Twrd_ + 2 , angle_local_parameterization );

        // loop edge
        std::vector< KeyFrame* > loop_kfs;
        std::vector< Sophus::SE2 > loop_delta_pose;
        kf->getLoopEdge ( loop_kfs, loop_delta_pose );
		
        for ( size_t nkf = 0; nkf < loop_kfs.size(); nkf ++ ) {
            KeyFrame* loop_kf = loop_kfs.at ( nkf );
            loop_kf->cvtEigen2Double();

            // Check if there is a conflict with the consensus side
            std::set<KeyFrame*> ob_kfs = kf->getObKFs();
            std::set<KeyFrame*>::iterator dkf = ob_kfs.find ( loop_kf );
            if ( dkf != ob_kfs.end() ) {
                ob_kfs.erase ( dkf );
            }
            kf->setObKFs ( ob_kfs );

            Sophus::SE2 delta_pose = loop_delta_pose.at ( nkf );
            ceres::CostFunction* cost_function =
                EncoderFrame2FrameError::Create ( delta_pose.translation() ( 0 ), delta_pose.translation() ( 1 ), delta_pose.so2().log(), cfg_->sm_lc_loop_edge_weight_ *  Eigen::Matrix3d::Identity() );
            problem.AddResidualBlock ( cost_function,
                                       loss,
                                       loop_kf->Twrd_, loop_kf->Twrd_ + 1, loop_kf->Twrd_ +2,
                                       kf->Twrd_, kf->Twrd_ + 1, kf->Twrd_ +2
                                     );

            problem.SetParameterization ( loop_kf->Twrd_ + 2 , angle_local_parameterization );
            problem.SetParameterization ( kf->Twrd_ +2 , angle_local_parameterization );
        } //  for all loop edge

        // observation edge
        std::set<KeyFrame*> ob_kfs = kf->getVisualEdge();
        for ( KeyFrame* okf: ob_kfs ) {
            okf->cvtEigen2Double();

            Sophus::SE2 delta_pose = okf->getSE2Pose().inverse() * kf->getSE2Pose();

            ceres::CostFunction* cost_function =
                EncoderFrame2FrameError::Create ( delta_pose.translation() ( 0 ), delta_pose.translation() ( 1 ), delta_pose.so2().log(), cfg_->sm_lc_cov_edge_weight_ *  Eigen::Matrix3d::Identity() );
            problem.AddResidualBlock ( cost_function,
                                       loss,
                                       okf->Twrd_, okf->Twrd_ + 1, okf->Twrd_ +2,
                                       kf->Twrd_, kf->Twrd_ + 1, kf->Twrd_ +2
                                     );

            problem.SetParameterization ( okf->Twrd_ + 2 , angle_local_parameterization );
            problem.SetParameterization ( kf->Twrd_ +2 , angle_local_parameterization );
        } //  for all observed kfs
    }// for all kfs

    // fixed the first KF
    loop_kf->cvtEigen2Double();
    problem.SetParameterBlockConstant ( loop_kf->Twrd_ );
    problem.SetParameterBlockConstant ( loop_kf->Twrd_ + 1 );
    problem.SetParameterBlockConstant ( loop_kf->Twrd_ + 2 );

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.max_num_iterations = cfg_->sm_lc_pgop_niter_;

    options.gradient_tolerance = 1e-20;
    options.function_tolerance = 1e-20;
    options.parameter_tolerance = 1e-20;

    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );

	
    // Mutex
    std::unique_lock<mutex> lock ( map_->update_mutex_ );

    /* move all mpts and Kfs. */
    for ( it = kfs.begin(); it != kfs.end(); it ++ ) {
        KeyFrame*kf = it->second;
        if ( kf->kfid_ <= loop_kf->kfid_ ) {
            continue;
        }
        kf->calculateRelativePosition();
        kf->cvtDouble2Eigen();
        kf->reCalculateMpts();
    }
} // poseGraphOptimization

} // namespace dre_slam
