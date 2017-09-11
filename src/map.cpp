namespace vie_slam {

void Map::insertKeyFrame ( Frame::Ptr frame ){
  cout<<"Key frame size = "<<keyframes_.size()<<endl;
  // 如果找不到id 则没有插入过
  // 插入新的keyframe
  if ( keyframes_.find(frame->id_) == keyframes_.end() ){
    keyframes_.insert( make_pair(frame->id_, frame) );
  }else{
    keyframes_[ frame->id_ ] = frame;
  }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point ){
  if ( map_points_.find(map_point->id_) == map_points_.end() ){
    map_points_.insert( make_pair(map_point->id_, map_point) );
  }else{
    map_points_[map_point->id_] = map_point;
  }
}

} /* vie_slam */