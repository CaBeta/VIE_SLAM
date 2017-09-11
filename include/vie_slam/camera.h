#ifndef VIESLAM_CAMERA_H_
#define VIESLAM_CAMERA_H_

#include "common/common_include.h"

namespace vie_slam {

class Camera {
private:
  //相机内参
  float fx_,fy_,cx_,cy_,depth_scale_;

public:
  typedef std::shared_ptr<Camera> Ptr;

  Camera();
  Camera (float fx,float fy,float cx,float cy,float depth_scale=0)
    : fx_(fx),fy_(fy),cx_(cx),cy_(cy),depth_scale_(depth_scale)
    {}
  virtual ~Camera ();

  //坐标系变换 世界坐标系 相机坐标系 图像坐标系 之间的转换
  Vector3d World2Camera(const Vector3d& p_w,const SE3& T_c_w);
  Vector3d Camera2World(const Vector3d& p_c,const SE3& T_c_w);
  Vector3d Camera2Pixel(const Vector3d& p_c);
  Vector3d Pixel2Camera(const Vector2d& p_p,double depth=1);
  Vector3d Pixel2World(const Vector2d& p_p,const SE3& T_c_w,double depth=1);
  Vector3d World2Pixel(const Vector3d& p_w,const SE3& T_c_w);
};

} /* vie_slam */

#endif
