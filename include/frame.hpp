#pragma once

#ifndef FRAME_HPP
#define FRAME_HPP

#include "common_include.hpp"

// forward declaration
class MapPoint;
class Feature;

class Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frame> Ptr;
  unsigned long id_ = 0;                                    // id of this frame
  unsigned long keyframe_id_ = 0;                           // id of keyframe
  bool is_keyframe_ = false;                                // if this frame is keyframe, value is true
  double time_stamp_;                                       // timestamp
  SE3 pose_;
  std::mutex pose_mutex_;                                   // for thread safe
  cv::Mat left_img_, right_img_;                            // stereo camera
  std::vector<std::shared_ptr<Feature>> features_left_;     // some features of left image
  std::vector<std::shared_ptr<Feature>> features_right_;    // some features of right image

  Frame() {}
  Frame(unsigned long id, double timestamp, const SE3 &pose,
        const cv::Mat &left_img, cv::Mat &right_img)
      :id_(id), time_stamp_(timestamp), pose_(pose), left_img_(left_img), right_img_(right_img) {}

  /**
   * @brief get pose of this frame
   * @return pose_
   */
  SE3 GetFramePose() {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_;
  }

  /**
   * @brief set pose of this frame
   * @param pose pose
   */
  void SetFramePose(const SE3 &pose) {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    pose_ = pose;
  }

  /**
   * @brief set this frame to keyframe and assign keyframe_id_
   */
  void SetKeyFrame();

  /**
   * @brief create frame and increase id_
   * @return new frame
   */
  static Ptr CreateFrame(); 
  };

#endif