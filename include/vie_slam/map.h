namespace vie_slam {

class Map {
private:
  // C++11新特性 无序列
  unordered_map<unsigned long,MapPoint::Ptr> map_points_; //所有地图上的点
  unordered_map<unsigned long,Frame::Ptr> keyframes_; //所有地图上的点

public:
  typedef shared_ptr<Map> Ptr;

  Map (){}
  virtual ~Map ();

  void insertKeyFrame(Frame::Ptr Frame);
  void insertMapPoint(MapPoint::Ptr map_point);
};

} /* vie_slam */
