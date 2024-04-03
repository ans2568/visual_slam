#include "map.hpp"
#include "feature.hpp"

void Map::InsertKeyFrame(Frame::Ptr frame) {
  current_frame_ = frame;

  if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
    keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
  } else {
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;
  }

  if (active_keyframes_.size() > num_active_keyframes_)
    RemoveOldKeyframe();
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
  if (landmarks_.find(map_point->id_) == landmarks_.end()) {
    landmarks_.insert(std::make_pair(map_point->id_, map_point));
    active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
  } else {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
  }
}

void Map::RemoveOldKeyframe() {
  if (current_frame_ == nullptr)
    return;
  double max_dist = 0, min_dist = 9999;
  double max_keyframe_id = 0, min_keyframe_id = 0;
  auto T_c_w = current_frame_->GetFramePose();
  auto T_w_c = T_c_w.inverse();
  for (auto &keyframe : active_keyframes_) {
    if (keyframe.second == current_frame_)
      continue;
    auto dist = (keyframe.second->GetFramePose() * T_w_c).log().norm();
    if (dist > max_dist) {
      max_dist = dist;
      max_keyframe_id = keyframe.first;
    }
    if (dist < min_dist) {
      min_dist = dist;
      min_keyframe_id = keyframe.first;
    }
  }

  const double min_dist_th = 0.2;
  Frame::Ptr frame_to_remove = nullptr;
  if (min_dist < min_dist_th)
    frame_to_remove = keyframes_.at(min_keyframe_id);
  else
    frame_to_remove = keyframes_.at(max_keyframe_id);

  LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;

  active_keyframes_.erase(frame_to_remove->keyframe_id_);
  for (auto feature : frame_to_remove->features_left_) {
    auto map_point = feature->map_point_.lock();
    if (map_point)
      map_point->RemoveObservation(feature);
  }
  for (auto feature : frame_to_remove->features_right_) {
    if (feature == nullptr)
      continue;
    auto map_point = feature->map_point_.lock();
    if (map_point)
      map_point->RemoveObservation(feature);
  }
  CleanMap();
}

void Map::CleanMap() {
  int removed_landmark_cnt = 0;
  for (auto iter = active_landmarks_.begin();
       iter != active_landmarks_.end();) {
    if (iter->second->observed_times_ == 0) {
      iter = active_landmarks_.erase(iter);
      removed_landmark_cnt++;
    } else
      ++iter;
  }
  LOG(INFO) << "Removed " << removed_landmark_cnt << " active landmarks";
}