#include "vie_slam/camera.h"
#include "vie_slam/config.h"

namespace vie_slam {

Camera::Camera(){
  fx_ = Config::get<float>("camera.fx");
  fy_ = Config::get<float>("camera.fy");
  cx_ = Config::get<float>("camera.cx");
  cy_ = Config::get<float>("camera.cy");
  depth_scale_ = Config::get<float>("camera.depth_scale");
}
Camera::~Camera(){

}

Vector3d Camera::World2Camera(const Vector3d& p_w,const SE3& T_c_w){
  return T_c_w * p_w;
}
Vector3d Camera::Camera2World(const Vector3d& p_c,const SE3& T_c_w){
  return T_c_w.inverse() * p_c;
}
Vector2d Camera::Camera2Pixel(const Vector3d& p_c){
  return Vector2d(
    fx_ * p_c(0,0) / p_c(2,0) + cx_,
    fy_ * p_c(1,0) / p_c(2,0) + cy_
  );
}
Vector3d Camera::Pixel2Camera(const Vector2d& p_p,double depth){
  return Vector3d(
    (p_p(0,0) - cx_) * depth / fx_,
    (p_p(1,0) - cy_) * depth / fy_,
    depth
  );
}
Vector3d Camera::Pixel2World(const Vector2d& p_p,const SE3& T_c_w,double depth){
  return Camera2World(Pixel2Camera(p_p,depth),T_c_w);
}
Vector2d Camera::World2Pixel(const Vector3d& p_w,const SE3& T_c_w){
  return Camera2Pixel(World2Camera(p_w,T_c_w));
}


} /* vie_slam */
