namespace vie_slam {

class Frame {
private:
  unsigned long id_;
  double time_stamp_;
  SE3 T_c_w_;
  Camera::ptr camera_;
  Mat color_,depth_;

public:
  typedef std::shared_ptr<Frame> Ptr;

  Frame ()
  Frame (long id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera=nullptr,
         Mat color=Mat(),Mat depth=Mat());
  virtual ~Frame ();
  // 构建一个帧
  static Frame::Ptr createFrame();
  // 获得特征点对应的深度
  double findDepth(const cv::KeyPoint& kp);
  // 获得相机中心
  Vector3d getCamCenter() const;
  // 判断世界坐标系上的点是否在该帧内
  bool isInFrame(const Vector3d& pt_world);
};

} /* vie_slam */
