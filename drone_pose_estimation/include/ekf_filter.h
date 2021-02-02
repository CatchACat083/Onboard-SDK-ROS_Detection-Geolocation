//////////////////////////////////////////////////////////////////////////////
/// COPYRIGHT NOTICE
/// Copyright (c) 2017, Research Group of UAV SWARMs at NUDT.
///
/// \file     EKF_filter.h
/// \brief    An Extended Kalman Filter for ground vehicle positions estimation.
///
/// \author   Dengqing Tang
/// \version  1.0.0
/// \date     2017.08
///
//////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"


/// \brief Estimate object position with Extended Kalman Filter.
///
/// This class build an Extended Kalman Filter to estimate object
/// spatial position in local NED coordinate frame. In this filter,
/// the measurement module is nonlinear but the process module.
class plane_EKF
{

 public:

  /// \brief Constructed function.
  plane_EKF();

  /// \brief Destructor.
  ~plane_EKF();

 public:

  /// \brief Initialize the filter.
  void init(Eigen::Matrix<double, 4, 1> x0);

  /// \brief Update the state x.
  void step(double dt, std::vector<double> z, Eigen::Matrix<double, 3, 3> r, Eigen::Matrix<double, 3, 1> t, std::vector<double>& x_);

 private:

  /// \brief Define matrix A.
  void makeA(double dt);

  /// \brief Define matrix H.
  void makeH(Eigen::Matrix<double, 3, 3> r, Eigen::Matrix<double, 3, 1> t);

  /// \brief Define process module.
  void make_process(double dt);

  /// \brief Define measurement module.
  void make_measure(Eigen::Matrix<double, 3, 3> r, Eigen::Matrix<double, 3, 1> t);

 public:

  double f, u0, v0;                   ///< camera intrinsic parameters
  Eigen::Matrix<double, 3, 3> intr;   ///< camera intrinsic matrix

  double dt;           ///< interval between last estimation and current estimation

  int m;   ///< dimension of measurement
  int n;   ///< dimension of state

  std::vector<double> x;   ///< state of filter
  std::vector<double> z_;  ///< measurement of filter

  Eigen::Matrix<double, 4, 4> P;     ///< P matrix of EKF
  Eigen::Matrix<double, 4, 2> W;     ///< W matrix of EKF
  Eigen::Matrix<double, 2, 2> Q;     ///< Q matrix of EKF
  Eigen::Matrix<double, 2, 2> V;     ///< V matrix of EKF
  Eigen::Matrix<double, 2, 2> R;     ///< R matrix of EKF
  Eigen::Matrix<double, 2, 4> H;     ///< H matrix of EKF
  Eigen::Matrix<double, 4, 4> A;     ///< A matrix of EKF

};


