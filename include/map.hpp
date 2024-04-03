#pragma once

#ifndef MAP_HPP
#define MAP_HPP

#include "common_include.hpp"
#include "frame.hpp"
#include "mappoint.hpp"

class Map {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Map> Ptr;
  typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
  typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

  Map() {}

  /**
   * @brief insert keyframe to unordered_map of keyframes in map
   * @param frame frame to insert
   */
  void InsertKeyFrame(Frame::Ptr frame);

  /**
   * @brief insert map point to unordered_map of map points in map
   * @param p_w map point to insert
   */
  void InsertMapPoint(MapPoint::Ptr map_point);

  /**
   * @brief get all map points in map
   * @return landmarks_
   */
  LandmarksType GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    return landmarks_;
  }

  /**
   * @brief get active map points in map
   * @return active_landmarks_;
   */
  LandmarksType GetActiveMapPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    return active_landmarks_;
  }

  /**
   * @brief get all keyframes in map
   * @return keyframes_;
   */
  KeyframesType GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(mutex_);
    return keyframes_;
  }

  /**
   * @brief get active keyframes in map
   * @return active_keyframes_;
   */
  KeyframesType GetActiveKeyFrames() {
    std::unique_lock<std::mutex> lock(mutex_);
    return active_keyframes_;
  }

  /**
   * @brief remove map point of active_landmarks_ which observed_times_ is 0
   */
  void CleanMap();

private:
  std::mutex mutex_;

  LandmarksType landmarks_;
  LandmarksType active_landmarks_;
  KeyframesType keyframes_;
  KeyframesType active_keyframes_;

  Frame::Ptr current_frame_ = nullptr;

  int num_active_keyframes_ = 7;

  /**
   * @brief remove old keyframe in active_keyframes_ calculated by distance_threshold(default is 0.2)
   */
  void RemoveOldKeyframe();
};

#endif
