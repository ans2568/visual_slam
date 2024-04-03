#pragma once
#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <pangolin/pangolin.h>

#include "common_include.hpp"
#include "frame.hpp"
#include "map.hpp"

class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Viewer> Ptr;

  Viewer();

  /**
   * @brief set map
   * @param map map
   */
  void SetMap(Map::Ptr map) { map_ = map; }

  /**
   * @brief close viewer
   */
  void Close();

  /**
   * @brief add current frame to viewer
   * @param current_frame current frame
   */
  void AddCurrentFrame(Frame::Ptr current_frame);

  /**
   * @brief update map
   */
  void UpdateMap();

private:
  /**
   * @brief start viewer thread
   */
  void ThreadLoop();

  /**
   * @brief draw current frame to viewer
   * @param frame frame to visualize
   * @param color if current frame is green, last frame is red
   */
  void DrawFrame(Frame::Ptr frame, const float *color);

  /**
   * @brief draw map point to viewer
   */
  void DrawMapPoints();

  /**
   * @brief viewer follows current frame
   * @param vis_camera follow object
   */
  void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

  /**
   * @brief plot the features in current frame into an image
   */
  cv::Mat PlotFrameImage();

  Frame::Ptr current_frame_ = nullptr;
  Map::Ptr map_ = nullptr;

  std::thread viewer_thread_;
  bool viewer_running_ = true;

  Map::KeyframesType active_keyframes_;
  Map::LandmarksType active_landmarks_;
  bool map_updated_ = false;

  std::mutex mutex_;

  float width_ = 1024;
  float height_ = 768;
  float fx_ = 400;
  float fy_ = 400;
  float cx_ = 512;
  float cy_ = 384;
};

#endif