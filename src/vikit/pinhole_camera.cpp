/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <vikit/math_utils.h>
#include <vikit/pinhole_camera.h>

namespace vk {

PinholeCamera::PinholeCamera(double width, double height, double scale,
                             double fx, double fy, double cx, double cy,
                             double d0, double d1, double d2, double d3,
                             double d4, double d5, double d6, double d7,
                             double d8, double d9, double d10, double d11,
                             double d12, double d13)
    : AbstractCamera(width * scale, height * scale, scale), fx_(fx * scale),
      fy_(fy * scale), cx_(cx * scale), cy_(cy * scale),
      distortion_(fabs(d0) > 0.0000001),
      undist_map1_(height_, width_, CV_16SC2),
      undist_map2_(height_, width_, CV_16SC2), use_optimization_(false) {
  cout << "scale: " << scale << endl;
  d_[0] = d0;
  d_[1] = d1;
  d_[2] = d2;
  d_[3] = d3;
  d_[4] = d4;
  d_[5] = d5;
  d_[6] = d6;
  d_[7] = d7;
  d_[8] = d8;
  d_[9] = d9;
  d_[10] = d10;
  d_[11] = d11;
  d_[12] = d12;
  d_[13] = d13;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 14) << d_[0], d_[1], d_[2], d_[3], d_[4], d_[5],
          d_[6], d_[7], d_[8], d_[9], d_[10], d_[11], d_[12], d_[13]);
  cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3, 3), cvK_,
                              cv::Size(width_, height_), CV_16SC2, undist_map1_,
                              undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::~PinholeCamera() {}

Vector3d PinholeCamera::cam2world(const double &u, const double &v) const {
  Vector3d xyz;
  if (!distortion_) {
    xyz[0] = (u - cx_) / fx_;
    xyz[1] = (v - cy_) / fy_;
    xyz[2] = 1.0;
  } else {
    cv::Point2f uv(u, v), px;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
    cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
    xyz[0] = px.x;
    xyz[1] = px.y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Vector3d PinholeCamera::cam2world(const Vector2d &uv) const {
  return cam2world(uv[0], uv[1]);
}

Vector2d PinholeCamera::world2cam(const Vector3d &xyz) const {
  return world2cam(project2d(xyz));
}

Vector2d PinholeCamera::world2cam(const Vector2d &uv) const {
  Vector2d px;
  if (!distortion_) {
    px[0] = fx_ * uv[0] + cx_;
    px[1] = fy_ * uv[1] + cy_;
  } else {
    // double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
    // x = uv[0];
    // y = uv[1];
    // r2 = x * x + y * y;
    // r4 = r2 * r2;
    // r6 = r4 * r2;
    // a1 = 2 * x * y;
    // a2 = r2 + 2 * x * x;
    // a3 = r2 + 2 * y * y;
    // cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;
    // xd = x * cdist + d_[2] * a1 + d_[3] * a2;
    // yd = y * cdist + d_[2] * a3 + d_[3] * a1;
    // px[0] = xd * fx_ + cx_;
    // px[1] = yd * fy_ + cy_;

    std::vector<cv::Point3f> objectPoints = {cv::Point3f(uv[0], uv[1], 1.0)};
    std::vector<cv::Point2f> imagePoints;
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::projectPoints(objectPoints, rVec, tVec, cvK_, cvD_, imagePoints);
    px[0] = imagePoints[0].x;
    px[1] = imagePoints[0].y;
  }
  return px;
}

void PinholeCamera::undistortImage(const cv::Mat &raw, cv::Mat &rectified) {
  if (distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
  else
    rectified = raw.clone();
}

} // end namespace vk
