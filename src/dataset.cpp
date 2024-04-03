#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "dataset.hpp"
#include "frame.hpp"

bool Dataset::Init() {
  // read camera intrinsic and extrinsic parameters
  std::ifstream fin(dataset_path_ + calibration_path_);
  if (!fin) {
    LOG(ERROR) << "cannot find " << dataset_path_ << calibration_path_ << "!";
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    // @TODO KITTI Dataset을 제외하고 다른 데이터 셋에 대한 호환성 추가 필요
    char camera_name[3];
    for (int j = 0; j < 3; ++j) {
      fin >> camera_name[j];
    }
    double projection_data[12];
    for (int j = 0; j < 12; ++j) {
      fin >> projection_data[j];
    }
    Mat33 K;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    Vec3 t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t; // coordinate transformation : image -> world
    K = K * 0.5; // why scale down values(focal_length and principal point)??
    Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                      t.norm(), SE3(SO3(), t)));
    cameras_.push_back(new_camera);
    LOG(INFO) << "Camera " << i << " extrinsic parameter : " << t.transpose();
  }
  fin.close();
  current_image_index_ = 0;
  return true;
}

Frame::Ptr Dataset::NextFrame() {
  boost::format fmt(image_format_);
  cv::Mat image_left, image_right;
  // read images
  image_left =
      cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);
  image_right =
      cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);

  if (image_left.data == nullptr || image_right.data == nullptr) {
    LOG(WARNING) << "cannot find images at index " << current_image_index_;
    return nullptr;
  }

  cv::Mat image_left_resized, image_right_resized;
  cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);
  cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);

  auto new_frame = Frame::CreateFrame();
  new_frame->left_img_ = image_left_resized;
  new_frame->right_img_ = image_right_resized;
  current_image_index_++;
  return new_frame;
}