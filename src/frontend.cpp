#include <opencv2/opencv.hpp>

#include "algorithm.hpp"
#include "config.hpp"
#include "feature.hpp"
#include "frontend.hpp"
#include "g2o_types.hpp"

Frontend::Frontend() {
  gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
  num_features_init_ = Config::Get<int>("num_features_init");
  num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(Frame::Ptr frame) {
  current_frame_ = frame;

  switch (status_) {
  case FrontendStatus::INITING:
    StereoInit();
    break;
  case FrontendStatus::TRACKING_GOOD:
  case FrontendStatus::TRACKING_BAD:
    Track();
    break;
  case FrontendStatus::TRACKING_LOST:
    Reset();
    break;
  }
  last_frame_ = current_frame_;
  return true;
}

bool Frontend::Track() {
  if (last_frame_) {
    current_frame_->SetFramePose(relative_motion_ *
                                 last_frame_->GetFramePose());
  }

  TrackLastFrame();
  tracking_inliers_ = EstimateCurrentPose();

  if (tracking_inliers_ > num_features_tracking_) {
    // tracking good
    status_ = FrontendStatus::TRACKING_GOOD;
  } else if (tracking_inliers_ > num_features_tracking_bad_) {
    // tracking bad
    status_ = FrontendStatus::TRACKING_BAD;
  } else {
    // tracking lost
    status_ = FrontendStatus::TRACKING_LOST;
  }

  InsertKeyFrame();
  relative_motion_ =
      current_frame_->GetFramePose() * last_frame_->GetFramePose().inverse();

  if (viewer_)
    viewer_->AddCurrentFrame(current_frame_);
  return true;
}

bool Frontend::InsertKeyFrame() {
  if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
    // still have enough features, don't insert keyframe
    return false;
  }
  // current frame is a new keyframe
  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);

  LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
            << current_frame_->keyframe_id_;

  SetObservationsForKeyFrame();
  // feature detection
  DetectFeatures();
  // track in right image by detected features
  FindFeaturesInRight();
  // triangulate map points
  TriangulateNewPoints();
  // update backend because we have a new keyframe
  backend_->UpdateMap();

  if (viewer_)
    viewer_->UpdateMap();

  return true;
}

void Frontend::SetObservationsForKeyFrame() {
  for (auto &feature : current_frame_->features_left_) {
    auto map_point = feature->map_point_.lock();
    if (map_point)
      map_point->AddObservation(feature);
  }
}

int Frontend::TriangulateNewPoints() {
  std::vector<SE3> poses{camera_left_->GetCameraPose(),
                         camera_right_->GetCameraPose()};
  SE3 current_pose_T_w_c = current_frame_->GetFramePose().inverse();
  int cnt_triangulated_points = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.expired() &&
        current_frame_->features_right_[i] != nullptr) {
      // try triangulation
      Vec2 left_pt =
          Point2Vec2(current_frame_->features_left_[i]->keypoint_.pt);
      Vec2 right_pt =
          Point2Vec2(current_frame_->features_right_[i]->keypoint_.pt);

      std::vector<Vec3> points{camera_left_->Pixel2Camera(left_pt),
                               camera_right_->Pixel2Camera(right_pt)};
      Vec3 pt_world = Vec3::Zero();
      if (Triangulation(poses, points, pt_world) && pt_world[2] > 0) {
        auto new_map_point = MapPoint::CreateNewMapPoint();
        pt_world = current_pose_T_w_c * pt_world;
        new_map_point->SetMapPointPosition(pt_world);
        new_map_point->AddObservation(current_frame_->features_left_[i]);
        new_map_point->AddObservation(current_frame_->features_right_[i]);
        current_frame_->features_left_[i]->map_point_ = new_map_point;
        current_frame_->features_right_[i]->map_point_ = new_map_point;
        map_->InsertMapPoint(new_map_point);
        cnt_triangulated_points++;
      }
    }
  }
  LOG(INFO) << "new landmarks : " << cnt_triangulated_points;
  return cnt_triangulated_points;
}

int Frontend::EstimateCurrentPose() {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // camera pose
  VertexPose *vertex_pose = new VertexPose();
  vertex_pose->setId(0);
  vertex_pose->setEstimate(current_frame_->GetFramePose());
  optimizer.addVertex(vertex_pose);

  // intrinsic parameter
  auto K = camera_left_->K();

  int index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Feature::Ptr> features;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    auto mp = current_frame_->features_left_[i]->map_point_.lock();
    if (mp) {
      features.push_back(current_frame_->features_left_[i]);
      EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pose_, K);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(
          Point2Vec2(current_frame_->features_left_[i]->keypoint_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.push_back(edge);
      optimizer.addEdge(edge);
      index++;
    }
  }

  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iter = 0; iter < 4; ++iter) {
    vertex_pose->setEstimate(current_frame_->GetFramePose());
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cnt_outlier = 0;

    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->is_outlier_) {
        e->computeError();
      }
      if (e->chi2() > chi2_th) {
        features[i]->is_outlier_ = true;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->is_outlier_ = false;
        e->setLevel(0);
      }

      // @TODO why set robust kernel to nullptr when iter is 2??
      if (iter == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }
  LOG(INFO) << "Outlier/Inlier in pose estimating : " << cnt_outlier << "/"
            << features.size() - cnt_outlier;
  current_frame_->SetFramePose(vertex_pose->estimate());

  LOG(INFO) << "Current Pose = \n" << current_frame_->GetFramePose().matrix();

  for (auto &feature : features) {
    if (feature->is_outlier_) {
      feature->map_point_.reset();
      feature->is_outlier_ = false; // maybe we can still use it in future
    }
  }
  return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {
  // use Lukas-Kanade flow to estimate points in the right image
  std::vector<cv::Point2f> kps_last, kps_current;
  for (auto &kp : last_frame_->features_left_) {
    if (kp->map_point_.lock()) {
      // use project point
      auto mp = kp->map_point_.lock();
      auto px =
          camera_left_->World2Pixel(mp->pose_, current_frame_->GetFramePose());
      kps_last.push_back(kp->keypoint_.pt);
      kps_current.push_back(cv::Point2f(px[0], px[1]));
    } else {
      kps_last.push_back(kp->keypoint_.pt);
      kps_current.push_back(kp->keypoint_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_current[i], 7);
      Feature::Ptr feature(new Feature(current_frame_, kp));
      feature->map_point_ = last_frame_->features_left_[i]->map_point_;
      current_frame_->features_left_.push_back(feature);
      num_good_pts++;
    }
  }

  LOG(INFO) << "Find " << num_good_pts << " in the last image";
  return num_good_pts;
}

bool Frontend::StereoInit() {
  DetectFeatures();
  int num_coor_features = FindFeaturesInRight();
  if (num_coor_features < num_features_init_) {
    // the number of right_feature is smaller than the number of feature_init
    return false;
  }

  bool build_map_success = BuildInitMap();
  if (build_map_success) {
    status_ = FrontendStatus::TRACKING_GOOD;
    if (viewer_) {
      viewer_->AddCurrentFrame(current_frame_);
      viewer_->UpdateMap();
    }
    return true;
  }
  return false;
}

int Frontend::DetectFeatures() {
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto &feat : current_frame_->features_left_) {
    cv::rectangle(mask, feat->keypoint_.pt - cv::Point2f(10, 10),
                  feat->keypoint_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(current_frame_->left_img_, keypoints, mask);
  int cnt_detected = 0;
  for (auto &kp : keypoints) {
    current_frame_->features_left_.push_back(
        Feature::Ptr(new Feature(current_frame_, kp)));
    cnt_detected++;
  }

  LOG(INFO) << "Detect " << cnt_detected << " new features";
  return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
  // use Lukas-Kanade flow to estimate points in the right image
  std::vector<cv::Point2f> kps_left, kps_right;
  for (auto &kp : current_frame_->features_left_) {
    kps_left.push_back(kp->keypoint_.pt);
    auto mp = kp->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      auto px =
          camera_right_->World2Pixel(mp->pose_, current_frame_->GetFramePose());
      kps_right.push_back(cv::Point2f(px[0], px[1]));
    } else {
      // use same pixel in left image
      kps_right.push_back(kp->keypoint_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_img_, current_frame_->right_img_, kps_left,
      kps_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_right[i], 7);
      Feature::Ptr feature(new Feature(current_frame_, kp));
      feature->is_on_left_image_ = false;
      current_frame_->features_right_.push_back(feature);
      num_good_pts++;
    } else {
      current_frame_->features_right_.push_back(nullptr);
    }
  }

  LOG(INFO) << "Find " << num_good_pts << " in the right image";
  return num_good_pts;
}

bool Frontend::BuildInitMap() {
  std::vector<SE3> poses{camera_left_->GetCameraPose(),
                         camera_right_->GetCameraPose()};
  size_t cnt_init_landmarks = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_right_[i] == nullptr)
      continue;

    Vec2 left_pt = Point2Vec2(current_frame_->features_left_[i]->keypoint_.pt);
    Vec2 right_pt =
        Point2Vec2(current_frame_->features_right_[i]->keypoint_.pt);
    std::vector<Vec3> points{camera_left_->Pixel2Camera(left_pt),
                             camera_right_->Pixel2Camera(right_pt)};
    Vec3 pt_world = Vec3::Zero();

    if (Triangulation(poses, points, pt_world) && pt_world[2] > 0) {
      auto new_map_point = MapPoint::CreateNewMapPoint();
      new_map_point->SetMapPointPosition(pt_world);
      new_map_point->AddObservation(current_frame_->features_left_[i]);
      new_map_point->AddObservation(current_frame_->features_right_[i]);
      current_frame_->features_left_[i]->map_point_ = new_map_point;
      current_frame_->features_right_[i]->map_point_ = new_map_point;
      cnt_init_landmarks++;
      map_->InsertMapPoint(new_map_point);
    }
  }
  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);
  backend_->UpdateMap();

  LOG(INFO) << "Initial map created with " << cnt_init_landmarks
            << " map points";

  return true;
}

bool Frontend::Reset() {
  // @TODO reset implementation when tracking lost
  LOG(INFO) << "Reset is not implemented. ";
  return true;
}