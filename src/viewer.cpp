#include <opencv2/opencv.hpp>

#include "viewer.hpp"
#include "feature.hpp"
#include "config.hpp"

Viewer::Viewer() {
  width_ = Config::Get<float>("display_width");
  height_ = Config::Get<float>("display_height");
  fx_ = Config::Get<float>("display_fx");
  fy_ = Config::Get<float>("display_fy");
  cx_ = Config::Get<float>("display_cx");
  cy_ = Config::Get<float>("display_cy");
  viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
  viewer_running_ = false;
  viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
  std::unique_lock<std::mutex> lock(mutex_);
  current_frame_ =current_frame;
}

void Viewer::UpdateMap() {
  std::unique_lock<std::mutex> lock(mutex_);
  assert(map_ != nullptr);

  active_keyframes_ = map_->GetActiveKeyFrames();
  active_landmarks_ = map_->GetActiveMapPoints();
  map_updated_ = true;
}

void Viewer::ThreadLoop() {
  pangolin::CreateWindowAndBind("VISUAL_SLAM", width_, height_);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::cout << "start create window" << std::endl;

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(int(width_), int(height_), double(fx_), double(fy_), double(cx_), double(cy_), 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));
  std::cout << "success create window" << std::endl;

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -double(width_) / double(height_))
          .SetHandler(new pangolin::Handler3D(vis_camera));

  const float green[3] = {0, 1, 0};

  while (!pangolin::ShouldQuit() && viewer_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    vis_display.Activate(vis_camera);

    std::unique_lock<std::mutex> lock(mutex_);
    if (current_frame_) {
      DrawFrame(current_frame_, green);
      FollowCurrentFrame(vis_camera);

      cv::Mat img = PlotFrameImage();
      cv::imshow("image", img);
      cv::waitKey(1);
    }

    if (map_) {
      DrawMapPoints();
    }

    pangolin::FinishFrame();
    usleep(5000);
  }

  LOG(INFO) << "Stop viewer";
}

cv::Mat Viewer::PlotFrameImage() {
  cv::Mat img_out;
  cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.lock()) {
      auto feature = current_frame_->features_left_[i];
      cv::circle(img_out, feature->keypoint_.pt, 2, cv::Scalar(0, 250, 0), 2);
    }
  }
  return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera) {
  SE3 T_w_c = current_frame_->GetFramePose().inverse();
  pangolin::OpenGlMatrix m(T_w_c.matrix());
  vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(Frame::Ptr frame, const float *color) {
  SE3 T_w_c = frame->GetFramePose().inverse();
  const float sz = 1.0;
  const int line_width = 2.0;

  glPushMatrix();

  Sophus::Matrix4f m = T_w_c.matrix().template cast<float>();
  glMultMatrixf((GLfloat *)m.data());

  if (color == nullptr)
    glColor3f(1, 0, 0);
  else
    glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);

  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);
  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);

  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);
  glVertex3f(sz * (0 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);

  glVertex3f(sz * (0 - cx_) / fx_, sz * (height_ - 1 - cy_) / fy_, sz);
  glVertex3f(sz * (0 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);

  glVertex3f(sz * (0 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);
  glVertex3f(sz * (width_ - 1 - cx_) / fx_, sz * (0 - cy_) / fy_, sz);

  glEnd();
  glPopMatrix();
}

void Viewer::DrawMapPoints() {
  const float red[3] = {1.0, 0, 0};
  for (auto &keyframe : active_keyframes_) {
    DrawFrame(keyframe.second, red);
  }

  glPointSize(2);
  glBegin(GL_POINTS);
  for (auto &landmark : active_landmarks_) {
    auto pose = landmark.second->GetMapPointPosition();
    glColor3f(red[0], red[1], red[2]);
    glVertex3d(pose[0], pose[1], pose[2]);
  }
  glEnd();
}