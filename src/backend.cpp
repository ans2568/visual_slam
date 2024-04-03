#include "backend.hpp"
#include "algorithm.hpp"
#include "feature.hpp"
#include "g2o_types.hpp"
#include "map.hpp"
#include "mappoint.hpp"

Backend::Backend() {
  backend_running_.store(true);
  backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap() {
  std::unique_lock<std::mutex> lock(mutex_);
  map_update_.notify_one();
}

void Backend::Stop() {
  backend_running_.store(false);
  map_update_.notify_one();
  backend_thread_.join();
}

void Backend::BackendLoop() {
  while (backend_running_.load()) {
    std::unique_lock<std::mutex> lock(mutex_);
    map_update_.wait(lock);

    auto active_keyframes = map_->GetActiveKeyFrames();
    auto active_landmarks = map_->GetActiveMapPoints();
    Optimize(active_keyframes, active_landmarks);
  }
}

void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  std::map<unsigned long, VertexPose *> vertices;
  unsigned long max_keyframe_id = 0;
  for (auto &keyframe : keyframes) {
    auto kf = keyframe.second;
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(kf->keyframe_id_);
    vertex_pose->setEstimate(kf->GetFramePose());
    optimizer.addVertex(vertex_pose);
    if (kf->keyframe_id_ > max_keyframe_id)
      max_keyframe_id = kf->keyframe_id_;

    vertices.insert({kf->keyframe_id_, vertex_pose});
  }

  std::map<unsigned long, VertexXYZ *> vertices_landmarks;

  auto K = camera_left_->K();
  SE3 left_extrinsic = camera_left_->GetCameraPose();
  SE3 right_extrinsic = camera_right_->GetCameraPose();

  int index = 1;
  double chi2_th = 5.991;
  std::map<EdgeProjection *, Feature::Ptr> edges_and_features;
  for (auto &landmark : landmarks) {
    if (landmark.second->is_outlier_)
      continue;
    unsigned long landmark_id = landmark.first;
    auto observations = landmark.second->GetObservation();
    for (auto &obs : observations) {
      if (obs.lock() == nullptr)
        continue;
      auto feat = obs.lock();
      if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
        continue;
      auto frame = feat->frame_.lock();
      EdgeProjection *edge = nullptr;
      if (feat->is_on_left_image_) {
        edge = new EdgeProjection(K, left_extrinsic);
      } else {
        edge = new EdgeProjection(K, right_extrinsic);
      }

      if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
        VertexXYZ *v = new VertexXYZ;
        v->setEstimate(landmark.second->GetMapPointPosition());
        v->setId(landmark_id + max_keyframe_id + 1);
        v->setMarginalized(true);
        vertices_landmarks.insert({landmark_id, v});
        optimizer.addVertex(v);
      }

      if (vertices.find(frame->keyframe_id_) != vertices.end() &&
          vertices_landmarks.find(landmark_id) != vertices_landmarks.end()) {
        edge->setId(index);
        edge->setVertex(0, vertices.at(frame->keyframe_id_));
        edge->setVertex(1, vertices_landmarks.at(landmark_id));
        edge->setMeasurement(Point2Vec2(feat->keypoint_.pt));
        edge->setInformation(Mat22::Identity());
        auto robust_kernel = new g2o::RobustKernelHuber();
        robust_kernel->setDelta(chi2_th);
        edge->setRobustKernel(robust_kernel);
        edges_and_features.insert({edge, feat});
        optimizer.addEdge(edge);
        index++;
      }
      else delete edge;
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  int cnt_outlier = 0, cnt_inlier = 0;
  int iteration = 0;
  while (iteration < 5) {
    cnt_outlier = 0;
    cnt_inlier = 0;

    for (auto &ef : edges_and_features) {
      if (ef.first->chi2() > chi2_th) {
        cnt_outlier++;
      } else {
        cnt_inlier++;
      }
    }
    double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
    if (inlier_ratio > 0.5) {
      break;
    } else {
      chi2_th *= 2;
      iteration++;
    }
  }

  for (auto &ef : edges_and_features) {
    if (ef.first->chi2() > chi2_th) {
      ef.second->is_outlier_ = true;
      // remove the observation
      ef.second->map_point_.lock()->RemoveObservation(ef.second);
    } else {
      ef.second->is_outlier_ = false;
    }
  }

  LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
            << cnt_inlier;

  for (auto &v : vertices)
    keyframes.at(v.first)->SetFramePose(v.second->estimate());
  for (auto &v_l : vertices_landmarks)
    landmarks.at(v_l.first)->SetMapPointPosition(v_l.second->estimate());
}