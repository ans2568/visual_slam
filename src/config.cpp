#include "config.hpp"

bool Config::SetParameterFile(const std::string &file_name) {
  if (config_ == nullptr)
    config_ = std::shared_ptr<Config>(new Config);
  config_->file_ = cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);
  if (config_->file_.isOpened() == false) {
    LOG(ERROR) << "parameter file " << file_name << " does not exist.";
    config_->file_.release();
    return false;
  }
  return true;
}

Config::~Config() {
  if (file_.isOpened())
    file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;