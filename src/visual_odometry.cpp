#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "gms/old_gms.h"
#include "vie_slam/config.h"
#include "vie_slam/visual_odometry.h"

namespace vie_slam {

VisualOdometry::VisualOdometry()
    : state_ (INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map),
      num_lost_(0),num_inliers_(0)
    {
      // 注意：这里写错是没有自动检查的
      // 如果没有找到对应的key 不会进行赋值 也不会报错
      // 应当做检查 或者在构造时设默认值
      num_features_ = Config::get<int>("number_of_features");
      scale_factor_ = Config::get<double>("scale_factor");
      level_pyramid_ = Config::get<int>("level_pyramid");
      match_ratio_ = Config::get<float>("match_ratio");
      max_num_lost_ = Config::get<float>("max_num_lost");
      min_inliers_ = Config::get<int>("min_inliers");
      keyframe_min_rotation = Config::get<double>("keyframe_min_rotation");
      keyframe_min_trans = Config::get<double>("keyframe_min_translation");
      orb_ = cv::ORB::create( num_features_);//, scale_factor_, level_pyramid_ );
      orb_->setFastThreshold(0);
    }

VisualOdometry::~VisualOdometry(){

}

bool VisualOdometry::addFrame(Frame::Ptr frame){
  switch (state_) {
    case INITIALIZING:
    {
      state_ = OK;
      curr_ = ref_ = frame;
      map_->insertKeyFrame ( frame );
      extractKeyPoints();
      computeDescriptors();
      // 计算参考帧中特征点的3d位置
      setRef3DPoints();
      break;
    }
    case OK:
    {
      curr_ = frame;
      extractKeyPoints();
      computeDescriptors();
      featureMatching();
      poseEstimationPnP();
      if (checkEstimatedPose() == true) {
        curr_->T_c_w_ = T_curr_ref_estimated_ * ref_->T_c_w_;
        ref_ = curr_;
        setRef3DPoints();
        num_lost_ = 0;
        if (checkKeyFrame() == true) {
          addKeyFrame();
        }
      }else{
        num_lost_++;
        if (num_lost_ > max_num_lost_) {
          state_ = LOST;
        }
        return false;
      }
      break;
    }
    case LOST:
    {
      std::cout << "Visual Odometry has lost." << '\n';
      break;
    }
    default:
    break;
  }
  return true;
}

void VisualOdometry::extractKeyPoints(){
  orb_->detect ( curr_->color_, keypoints_curr_ );
}

void VisualOdometry::computeDescriptors(){
  orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
}

void VisualOdometry::featureMatching(){
  // match desp_ref and desp_curr, use OpenCV's brute force match
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher ( cv::NORM_HAMMING );
  matcher.match ( descriptors_ref_, descriptors_curr_, matches );

  // GMS部分
  int num_inliers = 0;
	std::vector<bool> vbInliers;
  // 这里的keypoint和matches的descriptor对应不上的话会无法得到正确的筛选
	gms_matcher gms(keypoints_ref_,ref_->color_.size(),
                  keypoints_curr_,curr_->color_.size(), matches);
	num_inliers = gms.GetInlierMask(vbInliers, false, false);

  feature_matches_.clear();
  for (size_t i = 0; i < vbInliers.size(); ++i){
		if (vbInliers[i] == true){
			feature_matches_.push_back(matches[i]);
		}
	}
  std::cout<<"good matches: "<<feature_matches_.size()<<std::endl;
}

void VisualOdometry::setRef3DPoints(){
  // select the features with depth measurements
  points_3d_ref_.clear();
  descriptors_ref_ = Mat();
  keypoints_ref_.clear();

  for ( size_t i=0; i<keypoints_curr_.size(); i++ ) {
    double d = ref_->findDepth(keypoints_curr_[i]);
    if (d > 0){
      Vector3d p_cam = ref_->camera_->Pixel2Camera(
        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
      );
      points_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
      descriptors_ref_.push_back(descriptors_curr_.row(i));
      keypoints_ref_.push_back(keypoints_curr_[i]);
      // 这里只保留了成功计算深度的点到ref中
      // 为了同步keypoints 在这里也把keypoints重新进行筛选
      // 注意在push_back的时候要清除掉原来的
    }
  }
}

void VisualOdometry::poseEstimationPnP(){
  // construct the 3d 2d observations
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;

  for ( cv::DMatch m:feature_matches_ ) {
    pts3d.push_back( points_3d_ref_[m.queryIdx] );
    pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
  }
  Mat K = ( cv::Mat_<double>(3,3)<<
            ref_->camera_->fx_, 0, ref_->camera_->cx_,
            0, ref_->camera_->fy_, ref_->camera_->cy_,
            0,0,1
          );

  Mat rvec, tvec, inliers;
  cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

  num_inliers_ = inliers.rows;
  cout<<"pnp inliers: "<<num_inliers_<<endl;

  T_curr_ref_estimated_ = SE3(
    SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
    Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
  );
  // using bundle adjustment to optimize the pose
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;

  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block( linearSolver );

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
  g2o::SparseOptimizer optimizer;

  optimizer.setAlgorithm ( solver );

  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();

  pose->setId ( 0 );
  pose->setEstimate ( g2o::SE3Quat (
    T_c_r_estimated_.rotation_matrix(),
    T_c_r_estimated_.translation()
  ) );

  optimizer.addVertex ( pose );

  // edges
  for ( int i=0; i<inliers.rows; i++ ) {
    int index = inliers.at<int>(i,0);
    // 3D -> 2D projection
    EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
    edge->setId(i);
    edge->setVertex(0, pose);
    edge->camera_ = curr_->camera_.get();
    edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
    edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
    edge->setInformation( Eigen::Matrix2d::Identity() );

    optimizer.addEdge( edge );
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  T_curr_ref_estimated_ = SE3 (
    pose->estimate().rotation(),
    pose->estimate().translation()
  );
}

bool VisualOdometry::checkEstimatedPose(){
  // check if the estimated pose is good
  if ( num_inliers_ < min_inliers_ ) {
    cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
    return false;
  }
  // if the motion is too large, it is probably wrong
  Sophus::Vector6d d = T_curr_ref_estimated_.log();
  if ( d.norm() > 5.0 ) {
    cout<<"reject because motion is too large: "<<d.norm()<<endl;
    return false;
  }
  return true;
}

bool VisualOdometry::checkKeyFrame(){
  Sophus::Vector6d d = T_curr_ref_estimated_.log();
  Vector3d trans = d.head<3>();
  Vector3d rot = d.tail<3>();
  if ( rot.norm() >keyframe_min_rotation || trans.norm() >keyframe_min_trans )
    return true;
  return false;
}

void VisualOdometry::addKeyFrame(){
  cout<<"adding a key-frame"<<endl;
  map_->insertKeyFrame ( curr_ );
}

} /* vie_slam */
