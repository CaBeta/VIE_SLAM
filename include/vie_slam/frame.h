#ifndef FRAME_H_
#define FRAME_H_

#include "vie_slam/common.h"
#include "vie_slam/camera.h"

namespace vie_slam {
class MapPoint;
class Frame {
private:


public:
  typedef std::shared_ptr<Frame> Ptr;

  Frame ();
  Frame (long id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera=nullptr,
         Mat color=Mat(),Mat depth=Mat());
  ~Frame ();
  // 构建一个帧
  static Frame::Ptr createFrame();
  // 获得特征点对应的深度
  double findDepth(const cv::KeyPoint& kp);
  // 获得相机中心
  Vector3d getCamCenter() const;
  // 判断世界坐标系上的点是否在该帧内
  bool isInFrame(const Vector3d& pt_world);

  unsigned long id_;
  SE3 T_c_w_;
  Mat color_,depth_;
  Camera::Ptr camera_;
  double time_stamp_;
};

} /* vie_slam */

#endif
