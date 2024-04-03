#pragma once
#ifndef DATASET_HPP
#define DATASET_HPP

#include "camera.hpp"
#include "common_include.hpp"
#include "frame.hpp"

class Dataset {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;
  Dataset(const std::string &dataset_path,
          const std::string &calibration_path = "/calib.txt",
          const std::string &image_format = "%s/image_%d/%06d.png")
      : dataset_path_(dataset_path), calibration_path_(calibration_path),
        image_format_(image_format) {}

  /**
   * @brief init camera intrinsic and extrinsic parameter from config file
   * @return true if success initialization
   */
  bool Init();

  /**
   * @brief read image file and create next frame
   * @return new frame
   */
  Frame::Ptr NextFrame();

  /**
   * @brief get camera from cameras
   * @param camera_id_ id of cameras
   * @return camera by camera_id
   */
  Camera::Ptr GetCamera(int camera_id) const { return cameras_.at(camera_id); }

private:
  std::string dataset_path_;         // dataset path parameter from YAML file
  std::string calibration_path_;     // calibration file path parameter from YAML file
  std::string image_format_;         // image format parameter from YAML file
  int current_image_index_ = 0;
  std::vector<Camera::Ptr> cameras_;
};

#endif