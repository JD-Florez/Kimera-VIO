/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionImuFrontend-definitions.h
 * @brief  Definitions for MonoVisionImuFrontend
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/StereoPoint2.h>

#include "../Thirdparty/Kimera-VIO/include/kimera-vio/frontend/Camera.h"
#include "../Thirdparty/Kimera-VIO/include/kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "../Thirdparty/Kimera-VIO/include/kimera-vio/frontend/MonoImuSyncPacket.h"
#include "../Thirdparty/Kimera-VIO/include/kimera-vio/frontend/VisionImuFrontendParams.h"

namespace VIO {

using MonoFrontendInputPayload = MonoImuSyncPacket;
using MonoFrontendParams = FrontendParams;

// TODO(marcus): need to match this with mono Backend!
using MonoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;
using MonoMeasurements = std::vector<MonoMeasurement>;
using MonoMeasurementsUniquePtr = std::unique_ptr<MonoMeasurements>;
using StatusMonoMeasurements =
    std::pair<TrackerStatusSummary, MonoMeasurements>;
using StatusMonoMeasurementsPtr = std::shared_ptr<StatusMonoMeasurements>;

// ============================== FROM SATSLAM ==============================
using kp_idx_t = std::size_t;
using lm_idx_t = std::size_t;
using frame_idx_t = std::size_t;
typedef std::map<frame_idx_t, lm_idx_t> Frame_to_Lm_Map_t;

// 3D point in the map
class SatLandmark {
 public:
  gtsam::Point3 pt;
  int seen;  // in how many frames does this landmark appear
  bool isInitialized;

  Frame_to_Lm_Map_t lm_frame_kp;

  // Constructor
  SatLandmark(gtsam::Point3& pnt) {
    pt = pnt;
    seen = 0;
    isInitialized = false;
  }

  bool map_to_frame_and_kp(frame_idx_t frame_idx, kp_idx_t kp_idx) {
    if (lm_frame_kp.count(frame_idx) == 0) {
      lm_frame_kp.insert(std::pair<frame_idx_t, kp_idx_t>(frame_idx, kp_idx));
      seen++;
      return true;
    } else {
      std::cout << "redundant match" << std::endl;
      std::cout << "Add: " << frame_idx << " " << kp_idx << std::endl;
      std::cout << "Existing:" << std::endl;
      for (Frame_to_Lm_Map_t::iterator it = lm_frame_kp.begin(),
                                       end = lm_frame_kp.end();
           it != end; it++) {
        std::cout << it->first << " " << it->second << std::endl;
      }
      return false;
    }
  }

  void unscale(double& LU) { pt = LU * pt; }
};

// ==============================================================================

struct MonoFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MonoFrontendOutput(
      const bool& is_keyframe,
      const StatusMonoMeasurementsPtr& status_mono_measurements,
      const TrackingStatus& tracker_status,
      const gtsam::Pose3& relative_pose_body,
      const gtsam::Pose3& b_Pose_cam_rect,
      const Frame& frame_lkf,
      const std::vector<gtsam::Vector3> mesher_landmarks,
      const ImuFrontend::PimPtr& pim = ImuFrontend::PimPtr{},
      const ImuAccGyrS& imu_acc_gyrs = ImuAccGyrS{},
      const cv::Mat& feature_tracks = cv::Mat{},
      const DebugTrackerInfo& debug_tracker_info = DebugTrackerInfo{})
      : FrontendOutputPacketBase(frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kMonoImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info),
        status_mono_measurements_(status_mono_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_(relative_pose_body),
        b_Pose_cam_rect_(b_Pose_cam_rect),
        frame_lkf_(frame_lkf),
        mesher_landmarks_(mesher_landmarks),
        feature_tracks_(feature_tracks) {}

  virtual ~MonoFrontendOutput() = default;

 public:
  const StatusMonoMeasurementsPtr status_mono_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_;
  const gtsam::Pose3 b_Pose_cam_rect_;
  const Frame frame_lkf_;
  const std::vector<gtsam::Vector3> mesher_landmarks_;
  // std::vector<gtsam::Vector3> mesher_landmarks_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
