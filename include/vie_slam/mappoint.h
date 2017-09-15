#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include "vie_slam/common.h"

namespace vie_slam {

class MapPoint {
private:
  Vector3d pos_; //世界坐标系的坐标
  Vector3d norm_; //Normal of viewing direction
  Mat descriptor_; //描述子
  int observed_times_; //匹配阶段 被观测的次数
  int correct_times_; //位姿估计阶段 被使用的次数


public:
  typedef shared_ptr<MapPoint> Ptr;

  MapPoint ();
  MapPoint (long id,Vector3d position,Vector3d norm);
  virtual ~MapPoint ();

  // 构建一个地图点
  static MapPoint::Ptr createMapPoint();

  unsigned long id_;
};

} /* vie_slam */

#endif
