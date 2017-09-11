namespace vie_slam {

VisualOdometry::VisualOdometry()
    : state_ (INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map),
      num_lost_(0),num_inliers_(0)
    {
      num_features_ = Config::get<int>("num_features");
      scale_factor_ = Config::get<double>("scale_factor");
      level_pyramid_ = Config::get<int>("level_pyramid");
      match_ratio_ = Config::get<float>("match_ratio");
      max_num_lost_ = Config::get<int>("max_num_lost");
      min_inliers_ = Config::get<int>("min_inliers");
      keyframe_min_rotation = Config::get<double>("keyframe_min_rotation");
      keyframe_min_trans = Config::get<double>("keyframe_min_trans");
      orb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
    }

VisualOdometry::~VisualOdometry(){

}

bool VisualOdometry::addFrame(Frame::Ptr frame){
  switch (state_) {
    case INITIALIZING:
    {
      state_ = OK;
      curr_ = ref_ = frame;
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
      std::cout << "VisualOdometry has lost." << '\n';
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
  // select the best matches
  float min_dis = std::min_element(
    matches.begin(), matches.end(),
    [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
      return m1.distance < m2.distance;
    }
  )->distance;

  feature_matches_.clear();

  for ( cv::DMatch& m : matches ) {
    if ( m.distance < max<float>( min_dis * match_ratio_, 30.0 ) ) {
      feature_matches_.push_back(m);
    }
  }
  std::cout<<"good matches: "<<feature_matches_.size()<<std::endl;
}

void VisualOdometry::setRef3DPoints(){
  // select the features with depth measurements
  points_3d_ref_.clear();
  descriptors_ref_ = Mat();

  for ( size_t i=0; i<keypoints_curr_.size(); i++ ) {
    double d = ref_->findDepth(keypoints_curr_[i]);
    if (d > 0){
      Vector3d p_cam = ref_->camera_->pixel2camera(
        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
      );
      points_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
      descriptors_ref_.push_back(descriptors_curr_.row(i));
      // 这里只保留了成功匹配的点到ref中
      // 可以匹配的特征点会越来越少 过一段时间应该重新提取特征点来补充
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
  if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
    return true;
  return false;
}

void VisualOdometry::addKeyFrame(){
  cout<<"adding a key-frame"<<endl;
  map_->insertKeyFrame ( curr_ );
}

} /* vie_slam */
