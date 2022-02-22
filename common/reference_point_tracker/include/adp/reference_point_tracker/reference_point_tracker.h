//
// Copyright (C) 2021 Jiwung Choi <jiwung@gmail.com>
//
// This file is part of ADP (Autonomous Driving Planner).
//

#pragma once

#include "adp/trajectory/path.h"

#include <Eigen/Geometry>
#include <boost/math/tools/roots.hpp>

using boost::math::tools::bisect;

namespace adp {

//@{
/** Reference point tracking error evaluation classes */

class TargetErrorEvaluator {
public:
  virtual ~TargetErrorEvaluator() {}

  ///
  /// \brief evaluate target point tracking error in local range around \p s
  /// \param reference_path  the reference path
  /// \param agent_pose      the agent pose
  /// \param s               queried distance along \p reference_path
  ///
  virtual double evalLocalError(const Path &reference_path, const Eigen::Isometry3d &agent_pose,
                                const double s) const = 0;

  ///
  /// \brief evaluate target point tracking error in global range along a reference path
  /// \param reference_point  a point on the reference path
  /// \param agent_pose
  /// \return
  ///
  virtual double evalGlobalError(const PathPoint &reference_point, const Eigen::Isometry3d &agent_pose) const = 0;
};

///
/// \brief evaluate target point tracking error based on distance between agent position and the target point on a
/// reference path/trajectory
///
class TargetDistanceErrorEvaluator : public TargetErrorEvaluator {
public:
  TargetDistanceErrorEvaluator(const double lookahead_distance) : lookahead_distance_(lookahead_distance) {}

  /// \copydoc TargetErrorEvaluator::evalLocalError()
  double evalLocalError(const Path &reference_path, const Eigen::Isometry3d &agent_pose, const double s) const override;

  /// \copydoc TargetErrorEvaluator::evalGlobalError()
  double evalGlobalError(const PathPoint &reference_point, const Eigen::Isometry3d &agent_pose) const override;

private:
  const double lookahead_distance_;
};

///
/// \brief evaluate target point tracking error based on normality between the line connecting agent position and target
/// point, and the tangential line at the point along reference path/trajectory.
///
class TargetProjectionErrorEvaluator : public TargetErrorEvaluator {
public:
  TargetProjectionErrorEvaluator() = default;

  /// \copydoc TargetErrorEvaluator::evalLocalError()
  double evalLocalError(const Path &reference_path, const Eigen::Isometry3d &agent_pose, const double s) const override;

  /// \copydoc TargetErrorEvaluator::evalGlobalError()
  double evalGlobalError(const PathPoint &reference_point, const Eigen::Isometry3d &agent_pose) const override;
};

//@}  // end of Reference point tracking error evaluation classes

///
/// \brief find the solution s such that \f$ error_eval_f(s) = 0 \f$
/// \tparam EVAL_F evaluate function or functor
/// \param s_initial_guess  the initial guess of s pursuing \f$ error_eval_f(s) = 0 \f$
/// \param search_range_ds  the relative s interval with respect to \p s_initial_guess for search
/// \return the s value such that \f$ error_eval_f(s) = 0 \f$
///
template <class EVAL_F>
double searchSForwardAndBackward(const double s_initial_guess, const std::pair<double, double> &search_range_ds,
                                 EVAL_F error_eval_f) {
  auto terminal_condition = [](const double f_min, const double f_max) { return fabs(f_min - f_max) <= 0.000001; };

  const auto current_error = error_eval_f(s_initial_guess);
  // check if current_error is nearly zero
  if (fabs(current_error) < 1e-10) {
    return s_initial_guess;
  }

  const auto s_lower_bound = s_initial_guess + search_range_ds.first;
  const auto s_upper_bound = s_initial_guess + search_range_ds.second;

  // 1. bisect() over the range of [s_initial_guess, s_max] such that current_error * error_eval_f(s_max) < 0
  for (double s_max = s_initial_guess + 1.; s_max < s_upper_bound; s_max += 1.) {
    if (current_error * error_eval_f(s_max) < 0) {
      std::pair<double, double> result = bisect(error_eval_f, s_initial_guess, s_max, terminal_condition);
      return (result.first + result.second) / 2.;
    }
  }

  // 2. bisect() over the range of [s_min, s_initial_guess] such that current_error * error_eval_f(s_min) < 0
  for (double s_min = s_initial_guess - 1.; s_min > s_lower_bound; s_min -= 1.) {
    if (current_error * error_eval_f(s_min) < 0) {
      std::pair<double, double> result = bisect(error_eval_f, s_min, s_initial_guess, terminal_condition);
      return (result.first + result.second) / 2.;
    }
  }
  return s_initial_guess;
}

///
/// \brief tracks a target point on a reference path/trajectory
///
class ReferencePointTracker {
public:
  ReferencePointTracker(std::unique_ptr<TargetErrorEvaluator> track_error_eval)
      : track_error_eval_(std::move(track_error_eval)) {}

  ///
  /// \brief findTargetS   finds longitudinal distance of \p reference such that
  /// \f$ track_error_eval_(reference, agent_pose, s) = 0 \f$
  /// \param reference  the reference path
  /// \param agent_pose  the agent pose
  ///
  double findTargetS(const Path &reference_path, const Eigen::Isometry3d &agent_pose);

  PathPoint findTargetPathPoint(const Path &reference_path, const Eigen::Isometry3d &agent_pose);
  PathPoint findTargetPathPoint();

  ///
  /// \brief find the point index of \p reference_path that minimizes error evaluated by
  /// \ref track_error_eval_->evalGlobalError()
  ///
  size_t findMinErrorRefPointIndex(const Path &reference_path, const Eigen::Isometry3d &agent_pose) const;

  double getLatestTargetS() const { return latest_target_s_; }

  void updateReferencePath(const std::shared_ptr<const Path> &reference_path);
  void updateAgentPose(const std::shared_ptr<const Eigen::Isometry3d> &agent_pose);

private:
  std::unique_ptr<TargetErrorEvaluator> track_error_eval_{nullptr};
  std::shared_ptr<const Path> reference_path_{nullptr};
  std::shared_ptr<const Eigen::Isometry3d> agent_pose_{nullptr};

  //!< latest tracked s value of target point on reference_points
  double latest_target_s_{0.};

  //!< relative interval of s to determine search range of \fn searchSForwardAndBackward()
  const std::pair<double, double> search_range_ds_{-5., 10.5};
};

} // namespace adp
