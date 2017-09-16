#ifndef VISUALODOMETRY_H_
#define VISUALODOMETRY_H_

#include "vie_slam/common.h"
#include "vie_slam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace vie_slam {

class VisualOdometry {
private:

  Map::Ptr map_;
  Frame::Ptr ref_;
  Frame::Ptr curr_;

  cv::Ptr<cv::ORB> orb_;
  std::vector<cv::Point3f> points_3d_ref_;
  std::vector<cv::KeyPoint> keypoints_curr_;
  std::vector<cv::KeyPoint> keypoints_ref_;
  Mat descriptors_curr_;
  Mat descriptors_ref_;
  std::vector<cv::DMatch> feature_matches_;

  SE3 T_curr_ref_estimated_;
  int num_inliers_;
  int num_lost_; // number of lost times

  // 参数
  int num_features_; // 提取的特征数
  double scale_factor_;
  int level_pyramid_; // 金字塔数量
  float match_ratio_; // 选择正确匹配的比例
  int max_num_lost_; // 最大允许无效pose数
  int min_inliers_; // 最小内点数

  double keyframe_min_rotation; // 两关键帧间最小旋转量
  double keyframe_min_trans; // 两关键帧间最小运动量

public:
  typedef shared_ptr<VisualOdometry> Ptr;
  enum VisualOdometryState {
    INITIALIZING = -1,
    OK = 0,
    LOST
  };
  VisualOdometryState state_;

  VisualOdometry();
  virtual ~VisualOdometry ();

  bool addFrame(Frame::Ptr frame);

protected:
  void extractKeyPoints();
  void computeDescriptors();
  void featureMatching();
  void poseEstimationPnP();
  void setRef3DPoints();

  void addKeyFrame();
  bool checkEstimatedPose();
  bool checkKeyFrame();
};

} /* vie_slam */

#endif
