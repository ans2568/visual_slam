#pragma once

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "common_include.hpp"

class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Camera> Ptr;

  // Intrinsic Parameter
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;

  // Extrinsic Parameter
  SE3 pose_;

  // inverse of extrinsic parameter
  SE3 pose_inv_;

  Camera() {}
  Camera(double fx, double fy, double cx, double cy, double baseline,
         const SE3 &pose)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
    pose_inv_ = pose.inverse();
  }

  /**
   * @brief return camera pose
   * @return pose_
   */
  SE3 GetCameraPose() const { return pose_; }

  /**
   * @brief return intrinsic parameter from camera
   * @return K_
   */
  Mat33 K() const {
    Mat33 k;
    k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return k;
  }

  /**
   * @brief coordinate transform world -> camera
   * @param p_w point in world coordinate
   * @param T_c_w relative transformation matrix between camera and world
   * @return pose_ * T_c_w * p_w
   */
  Vec3 World2Camera(const Vec3 &p_w, const SE3 &T_c_w);

  /**
   * @brief coordinate transform camera -> world
   * @param p_c point in camera coordinate
   * @param T_c_w relative transformation matrix between camera and world
   * @return T_c_w.inverse() * pose_inv_ * p_c
   */
  Vec3 Camera2World(const Vec3 &p_c, const SE3 &T_c_w);

  /**
   * @brief project point from camera to pixel
   * @param p_c point in camera coordinate
   * @brief p_p_x = fx * p_c_x / p_c_z + cx
   * @brief p_p_y = fy * p_c_y / p_c_z + cy
   * @return K() * p_c
   */
  Vec2 Camera2Pixel(const Vec3 &p_c);

  /**
   * @brief reproject point from pixel to camera
   * @param p_p point in pixel coordinate
   * @param depth depth information of the point
   * @brief p_c_x = (p_p_x - cx) * depth / fx
   * @brief p_c_y = (p_p_y - cy) * depth / fy
   * @brief p_c_z = depth
   */
  Vec3 Pixel2Camera(const Vec2 &p_p, double depth = 1);

  /**
   * @brief reproject point from pixel to world
   * @param p_p point in pixel coordinate
   * @param T_c_w relative transformation matrix between camera and world
   * @param depth depth information of the point
   * @return camera2world(pixel2camera(p_p, depth), T_c_w)
   */
  Vec3 Pixel2World(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

  /**
   * @brief project point from world to pixel
   * @param p_w point in world coordinate
   * @param T_c_w relative transformation matrix between camera and world
   * @return camera2pixel(world2camera(p_w, T_c_w))
   */
  Vec2 World2Pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

#endif