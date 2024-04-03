#pragma once

#ifndef FEATURE_HPP
#define FEATURE_HPP

#include "common_include.hpp"

// forward declaration
class Frame;
class MapPoint;

class Feature {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Feature> Ptr;

  std::weak_ptr<Frame> frame_;              // frame that holds this feature
  cv::KeyPoint keypoint_;                   // keypoint of this feature
  std::weak_ptr<MapPoint> map_point_;       // map point that holds this feature

  bool is_outlier_ = false;
  bool is_on_left_image_ = true;            // direction which is on left image or right image

  Feature() {}
  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &keypoint)
      : frame_(frame), keypoint_(keypoint) {}
};

#endif