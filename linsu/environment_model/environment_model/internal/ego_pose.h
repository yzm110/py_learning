#ifndef POINT_BASED_EM_EGO_POSE_H
#define POINT_BASED_EM_EGO_POSE_H
#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/ego_motion_data.h"
//#include "common/data/em_data.h"
//#include "common/time/time.h"
//#include "math/linear_interpolation.h"
//#include "math/motion_model/cst_acc_derivative_model.h"
//#include "preprocess_data.h"
namespace zone {
namespace environment_model {

// using namespace std;
// using ::zone::helper::math::LinearInterpolation;
// using ::zone::helper::math::CstAccDerivativeModel;
// using ::zone::helper::math::Lin_Interp_Method;
// using ::zone::common::Time;
using ::zone::common::EgoMotionData;

static const bc::uint8_t kMaxEgoPoseNum = 5;
static const bc::float64_t kEgoPoseTimeGap = 1.f;

struct TransForm
{
  TransForm() : rotation_f_(0.f), translation_dx_f_(0.f), translation_dy_f_(0.f), delta_time_f_(0.f) {}
  bc::float32_t rotation_f_;
  bc::float32_t translation_dx_f_;
  bc::float32_t translation_dy_f_;
  bc::float32_t delta_time_f_;
};

class EgoPose
{
 public:
  EgoPose()
      : valid_b_(bc::false_v),
        timestamp_f64_(0.f),
        dx_f_(0.f),
        dy_f_(0.f),
        heading_rad_f_(0.f),
        yawrate_f_(0.f),
        curvature_f_(0.f),
        long_velocity_f_(0.f),
        lat_velocity_f_(0.f),
        long_acceleration_f_(0.f),
        lat_acceleration_f_(0.f),
        transform_st_()
  {
  }
  virtual ~EgoPose() = default;

  bc::bool_t GetValidation() const { return valid_b_; }

  void SetValidation(bc::bool_t valid_b) { valid_b_ = valid_b; }

  bc::float64_t GetTimestamp() const { return timestamp_f64_; }

  void SetTimestamp(bc::float64_t timestamp_f64) { timestamp_f64_ = timestamp_f64; }

  bc::float32_t GetLongX() const { return dx_f_; }

  void SetLongX(bc::float32_t dx_f) { dx_f_ = dx_f; }

  bc::float32_t GetLatY() const { return dy_f_; }

  void SetLatY(bc::float32_t dy_f) { dy_f_ = dy_f; }

  bc::float32_t GetHeading() const { return heading_rad_f_; }

  void SetHeading(bc::float32_t heading_rad_f) { heading_rad_f_ = heading_rad_f; }

  bc::float32_t GetYawrate() const { return yawrate_f_; }

  void SetYawrate(bc::float32_t yawrate_f) { yawrate_f_ = yawrate_f; }

  bc::float32_t GetCurvature() const { return curvature_f_; }

  void SetCurvature(bc::float32_t curvature_f) { curvature_f_ = curvature_f; }

  bc::float32_t GetLongVelocity() const { return long_velocity_f_; }

  void SetLongVelocity(bc::float32_t long_velocity_f) { long_velocity_f_ = long_velocity_f; }

  bc::float32_t GetLatVelocity() const { return lat_velocity_f_; }

  void SetLatVelocity(bc::float32_t lat_velocity_f) { lat_velocity_f_ = lat_velocity_f; }

  bc::float32_t GetLongAcceleration() const { return long_acceleration_f_; }

  void SetLongAcceleration(bc::float32_t long_acceleration_f) { long_acceleration_f_ = long_acceleration_f; }

  bc::float32_t GetLatAcceleration() const { return lat_acceleration_f_; }

  void SetLatAcceleration(bc::float32_t lat_acceleration_f) { lat_acceleration_f_ = lat_acceleration_f; }

  TransForm GetTransformFromLastCycle() const { return transform_st_; }

  void SetTransform(bc::float32_t rot_f, bc::float32_t trans_dx_f, bc::float32_t trans_dy_f, bc::float32_t dt_f)
  {
    transform_st_.rotation_f_ = rot_f;
    transform_st_.translation_dx_f_ = trans_dx_f;
    transform_st_.translation_dy_f_ = trans_dy_f;
    transform_st_.delta_time_f_ = dt_f;
  }

  /** @brief Transform Ego Motion to local EgoPose structure
   *  @param ego_motion_data_st Raw Ego Motion data
   */
  void SetEgoMotionData(const EgoMotionData& ego_motion_data_st)
  {
    valid_b_ = bc::true_v;
    timestamp_f64_ = ego_motion_data_st.timestamp;
    dx_f_ = ego_motion_data_st.pose.position.x;
    dy_f_ = ego_motion_data_st.pose.position.y;
    heading_rad_f_ = ego_motion_data_st.heading;
    yawrate_f_ = ego_motion_data_st.twist.angular.z;
    curvature_f_ = ego_motion_data_st.kappa;
    long_velocity_f_ = ego_motion_data_st.twist.linear.x;
    lat_velocity_f_ = ego_motion_data_st.twist.linear.y;
    // Set acceleration to 0 if velocity is really close to 0
    if (fabsf(long_velocity_f_) <= FLOAT32_EPSILON)
    {
      long_acceleration_f_ = 0.f;
    }
    else
    {
      long_acceleration_f_ = ego_motion_data_st.linear_acceleration.x;
    }
    if (fabsf(lat_velocity_f_) <= FLOAT32_EPSILON)
    {
      lat_acceleration_f_ = 0.f;
    }
    else
    {
      lat_acceleration_f_ = ego_motion_data_st.linear_acceleration.y;
    }
  }

 private:
  bc::bool_t valid_b_;
  bc::float64_t timestamp_f64_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t heading_rad_f_;  // Radian
  bc::float32_t yawrate_f_;
  bc::float32_t curvature_f_;
  bc::float32_t long_velocity_f_;
  bc::float32_t lat_velocity_f_;
  bc::float32_t long_acceleration_f_;
  bc::float32_t lat_acceleration_f_;
  TransForm transform_st_;
};

class EgoPoseCollection
{
 public:
  EgoPoseCollection() : valid_ctns_u8_(0), ego_poses_ar_()
  {
    // Init ego poses array with default value
    for (bc::uint8_t i = 0; i < kMaxEgoPoseNum; ++i)
    {
      ego_poses_ar_[i] = EgoPose();
    }
  }
  virtual ~EgoPoseCollection() = default;

  /** @brief Remove all recorded ego poses and reset valid_ctns_u8_
   */
  void ClearEgoPose()
  {
    for (bc::uint8_t i = 0; i < valid_ctns_u8_; ++i)
    {
      ego_poses_ar_[i] = EgoPose();
    }
    valid_ctns_u8_ = 0;
  }

  /** @brief Add new ego pose to index 0 and remove the last one if the
   *  container is full
   *  @param ego_motion_data_st Raw Ego Motion data
   *  @return boolean
   */
  bc::bool_t AddEgoPose(const EgoMotionData& ego_motion_data_st, const bc::float32_t time_cycle_f)
  {
    bc::float64_t t_delta_time_f = time_cycle_f;
    // bc::float64_t t_delta_time_f = ego_motion_data_st.timestamp - ego_poses_ar_[0].GetTimestamp();
    // Timestamp cannot less than or equal to 0 and
    // skip the new ego motion data if the timestamp is too close to the last cycle
    if (ego_motion_data_st.timestamp < FLOAT64_EPSILON || fabs(t_delta_time_f) < FLOAT64_EPSILON)
    {
      return bc::false_v;
    }
    bc::float32_t t_rotation_f = 0.f;
    bc::float32_t t_translation_dx_f = 0.f;
    bc::float32_t t_translation_dy_f = 0.f;
    bc::float32_t t_trans_time_f = 0.f;
    if (valid_ctns_u8_ > 0)
    {
      // Clear ego pose collection in case the time gap is too large or
      // timestamp is back to the past(for debug only)
      if (t_delta_time_f < 0.f || t_delta_time_f > kEgoPoseTimeGap)
      {
        ClearEgoPose();
      }
      else
      {
        // Calculte the transformation between current and last ego motion
        t_trans_time_f = t_delta_time_f;
        // t_rotation_f = ego_motion_data_st.heading - ego_poses_ar_[0].GetHeading();
        /// TODO: Use yawrate to calculate difference of rotation, need to be discussed
        bc::float32_t t_mean_yawrate_f = 0.5 * (ego_motion_data_st.twist.linear.x * ego_motion_data_st.kappa +
                                                ego_poses_ar_[0].GetLongVelocity() * ego_poses_ar_[0].GetCurvature());
        t_rotation_f = t_mean_yawrate_f * t_delta_time_f;
        bc::float32_t t_mean_vel_f = 0.5 * (ego_motion_data_st.twist.linear.x + ego_poses_ar_[0].GetLongVelocity());
        if (std::fabs(t_mean_yawrate_f) < FLOAT32_EPSILON)
        {
          t_translation_dx_f = t_mean_vel_f * t_trans_time_f;
          t_translation_dy_f = 0.f;
        }
        else
        {
          bc::float32_t t_l_radius_f = t_mean_vel_f / t_mean_yawrate_f;
          t_translation_dx_f = t_l_radius_f * sinf(t_rotation_f);
          t_translation_dy_f = t_l_radius_f * (1 - cosf(t_rotation_f));
        }
        // t_translation_dx_f = t_mean_vel_f * t_trans_time_f;
        // t_translation_dy_f = 0.5 * t_mean_vel_f * t_mean_yawrate_f * t_trans_time_f * t_trans_time_f;
        // Move each ego pose backforward by one position
        if (valid_ctns_u8_ == kMaxEgoPoseNum)
        {
          for (bc::uint8_t i = valid_ctns_u8_ - 1; i >= 1; --i)
          {
            ego_poses_ar_[i] = ego_poses_ar_[i - 1];
          }
        }
        else
        {
          for (bc::int8_t i = valid_ctns_u8_ - 1; i >= 0; --i)
          {
            ego_poses_ar_[i + 1] = ego_poses_ar_[i];
          }
        }
      }
    }
    ego_poses_ar_[0].SetEgoMotionData(ego_motion_data_st);
    ego_poses_ar_[0].SetTransform(t_rotation_f, t_translation_dx_f, t_translation_dy_f, t_trans_time_f);
    // Update valid_ctns_u8_ if the container is not full
    if (valid_ctns_u8_ < kMaxEgoPoseNum)
    {
      valid_ctns_u8_++;
    }
    return bc::true_v;
  }

  /** @brief Get the newest ego motion data in collection.
   *  Note that the returned ego pose may have no valid data.
   *  @return EgoPose
   */
  EgoPose& GetCurrentEgoPose() { return ego_poses_ar_[0]; }
  EgoPose GetCurrentEgoPoseReadOnly() const { return ego_poses_ar_[0]; }

 private:
  bc::uint8_t valid_ctns_u8_;
  bc::TCArray<EgoPose, kMaxEgoPoseNum> ego_poses_ar_;
};
}
}
#endif