#pragma once
#ifndef BACKEND_HPP
#define BACKEND_HPP

#include "camera.hpp"
#include "map.hpp"

class Backend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Backend> Ptr;
  Backend();

  /**
   * @brief set camera
   * @param left left camera
   * @param right right camera
   */
  void SetCameras(Camera::Ptr left, Camera::Ptr right) {
    camera_left_ = left;
    camera_right_ = right;
  }

  /**
   * @brief set map
   * @param map map
   */
  void SetMap(Map::Ptr map) { map_ = map; }

  /**
   * @brief notify condition_variable map_update_
   */
  void UpdateMap();

  /**
   * @brief stop backend loop
   */
  void Stop();

private:
  /**
   * @brief start backend loop
   */
  void BackendLoop();

  /**
   * @brief start Bundle Adjustment
   * @param keyframes keyframes in map
   * @param landmarks landmarks in map
   */
  void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

  Map::Ptr map_;
  std::thread backend_thread_;
  std::mutex mutex_;

  std::condition_variable map_update_;
  std::atomic<bool> backend_running_;

  Camera::Ptr camera_left_ = nullptr, camera_right_ = nullptr;
};

#endif