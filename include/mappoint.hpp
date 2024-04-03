#pragma once

#ifndef MAPPOINT_HPP
#define MAPPOINT_HPP

#include "common_include.hpp"

// forward declaration
class Frame;
class Feature;

class MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<MapPoint> Ptr;

  unsigned long id_ = 0;                            // id of this map point
  bool is_outlier_ = false;
  Vec3 pose_ = Vec3::Zero();                        // map point pose
  std::mutex mutex_;                                // for thread safe
  int observed_times_ = 0; // being observed by feature matching algorithm
  std::list<std::weak_ptr<Feature>> observations_;  // feature list observed by this map point

  MapPoint() {}
  MapPoint(unsigned long id, Vec3 pose) : id_(id), pose_(pose) {}

  /**
   * @brief get pose of this map point
   * @return pose_
   */
  Vec3 GetMapPointPosition() {
    std::unique_lock<std::mutex> lock(mutex_);
    return pose_;
  }

  /**
   * @brief set pose of this map point
   */
  void SetMapPointPosition(const Vec3 &pose) {
    std::unique_lock<std::mutex> lock(mutex_);
    pose_ = pose;
  }

  /**
   * @brief add feature to observations_ and increase observed_times
   */
  void AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lock(mutex_);
    observations_.push_back(feature);
    observed_times_++;
  }

  /**
   * @brief remove feature from observations and decrease observed_times
   */
  void RemoveObservation(std::shared_ptr<Feature> feature);

  /**
   * @brief return list of feature in map point
   * @return observations_
   */
  std::list<std::weak_ptr<Feature>> GetObservation() {
    std::unique_lock<std::mutex> lock(mutex_);
    return observations_;
  }

  /**
   * @brief create new map point and increase id_
   * @return new map point
   */
  static MapPoint::Ptr CreateNewMapPoint();
};

#endif