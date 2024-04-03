#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "common_include.hpp"

class Config {
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config() {}

public:
  ~Config();  // release the file when this class deconstructing

  /**
   * @brief set a config file
   * @param file_name name of file
   * @return true if success
   */
  static bool SetParameterFile(const std::string &file_name);

  /**
   * @brief get parameter values from key
   * @param key name of parameter
   * @return T(Config::config_->file_[key])
   */
  template <typename T>
  static T Get(const std::string &key) {
    return T(Config::config_->file_[key]);
  }
};

#endif