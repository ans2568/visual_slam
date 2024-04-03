#pragma once
#ifndef FRONTEND_HPP
#define FRONTEND_HPP

#include <opencv2/features2d.hpp>

#include "camera.hpp"
#include "common_include.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "viewer.hpp"
#include "backend.hpp"

enum class FrontendStatus {
  INITING,
  TRACKING_GOOD,
  TRACKING_BAD,
  TRACKING_LOST
};

class Frontend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frontend> Ptr;

  Frontend();

  /**
   * @brief add frame
   * @param frame frame
   * @return true if success
   */
  bool AddFrame(Frame::Ptr frame);

  /**
   * @brief set map
   * @param map map
   */
  void SetMap(Map::Ptr map) { map_ = map; }

  /**
   * @brief set backend
   * @param backend backend thread
   */
  void SetBackend(Backend::Ptr backend) { backend_ = backend; }

  /**
   * @brief set viewer
   * @param viewer viewer
   */
  void SetViewer(Viewer::Ptr viewer) { viewer_ = viewer; }

  /**
   * @brief get frontend status
   * @return enum element FrontendStatus{GOOD, BAD, LOST, INITING}
   */
  FrontendStatus GetStatus() const { return status_; }

  /**
   * @brief set camera
   * @param left left camera
   * @param right right camera
   */
  void SetCameras(Camera::Ptr left, Camera::Ptr right) {
    camera_left_ = left;
    camera_right_ = right;
  }

private:
  /**
   * @brief track in normal mode
   * @return true if success
   */
  bool Track();

  /**
   * @brief reset when tracking lost
   * @return true if success
   */
  bool Reset();

  /**
   * @brief track with last frame
   * @return num of tracked points
   */
  int TrackLastFrame();

  /**
   * @brief estimate current frame's pose
   * @return num of inliers
   */
  int EstimateCurrentPose();

  /**
   * @brief set current frame as a keyframe and insert it into backend
   * @return true if success
   */
  bool InsertKeyFrame();

  /**
   * @brief try init the frontend with stereo images saved in current_frame_
   * @return true if success
   */
  bool StereoInit();

  /**
   * @brief detect features in left image in current_frame_
   * @brief keypoints will be saved in current_frame_
   * @return
   */
  int DetectFeatures();

  /**
   * @brief find the corresponding features in right image of current_frame_
   * @return num of features found
   */
  int FindFeaturesInRight();

  /**
   * @brief build the initial map with single image
   * @return true if succeed
   */
  bool BuildInitMap();

  /**
   * @brief triangulate the 2D points in current frame
   * @return num of triangulated points
   */
  int TriangulateNewPoints();

  /**
   * @brief set the features in keyframe as new observation of the map points
   */
  void SetObservationsForKeyFrame();

  FrontendStatus status_ = FrontendStatus::INITING;

  Frame::Ptr current_frame_ = nullptr;
  Frame::Ptr last_frame_ = nullptr;
  Camera::Ptr camera_left_ = nullptr;
  Camera::Ptr camera_right_ = nullptr;

  Map::Ptr map_ = nullptr;
  Backend::Ptr backend_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;
  SE3 relative_motion_;

  int tracking_inliers_ = 0;

  // params
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  // utilities
  cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv
};

#endif