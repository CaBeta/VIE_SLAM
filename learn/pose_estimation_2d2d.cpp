void pose_estimation_2d2d(
  std::vector<KeyPoint> keypoints_1,
  std::vector<KeyPoint> keypoints_2,
  std::vector<DMatch> matches;
  Mat& R,Mat& t
) {
  //相机内参 camera parameter
  Mat K = (Mat_<double>(3,3) << fx,0,cx,0,fy,cy,0,0,1);

  //把匹配的特征点存入 points1 points2 转化成opencv用的数据类型
  std::vector<Point2f> points1;
  std::vector<Point2f> points2;
  for (size_t i = 0; i < (int)matches.size(); i++) {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  //计算F矩阵 （基础）
  Mat fundamentalMatrix = findFundamentalMat(points1,points2,CV_FM_8POINT);
  std::cout << "F is" << '\n';
  std::cout << fundamentalMatrix << '\n';

  //计算E矩阵 （本质）
  Point2d principal_point(cx,cy); //光心
  int focal_length = fx; //焦距
  Mat essentialMatrix = findEssentialMat(points1,points2,focal_length,principal_point,RANSAC);
  std::cout << "E is" << '\n';
  std::cout << essentialMatrix << '\n';

  //从E中计算R和t
  recoverPose(essentialMatrix,points1,points2,R,t,focal_length,principal_point);
  std::cout << "R is" << '\n' << R << '\n';
  std::cout << "t is" << '\n' << t << '\n';

}
