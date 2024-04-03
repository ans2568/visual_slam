#pragma once
#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

#include "backend.hpp"
#include "common_include.hpp"
#include "dataset.hpp"
#include "frontend.hpp"
#include "viewer.hpp"

class VisualOdometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;

  /// constructor with config file
  VisualOdometry(std::string &config_path) :config_file_path_(config_path) {}

  /**
   * @brief do initialization things before run
   * @return true if success
   */
  bool Init();

  /**
   * @brief start vo
   */
  void Run();

  /**
   * @brief make a step forward in dataset
   */
  bool Step();

  /**
   * @brief get frontend status
   * @return enum element FrontendStatus{GOOD, BAD, LOST, INITING}
   */
  FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
  bool inited_ = false;
  std::string config_file_path_ = "../config/config.yaml";

  Frontend::Ptr frontend_ = nullptr;
  Backend::Ptr backend_ = nullptr;
  Map::Ptr map_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;

  // dataset
  Dataset::Ptr dataset_ = nullptr;
};

#endif