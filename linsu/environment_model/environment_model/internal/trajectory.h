#ifndef POINT_BASED_EM_TRAJECTORY_H
#define POINT_BASED_EM_TRAJECTORY_H

#include <iostream>
#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/em_data.h"
#include "common/data/em_param.h"
#include "common/data/zone_shared_data.h"
#include "math/boundary_smooth.h"
namespace zone {
namespace environment_model {

using ::zone::common::Point2D;
using ::zone::data::em_data::kFusMaxObjNum;
using ::zone::data::em_data::kInvalidObjectId;
using ::zone::data::em_data::kMaxObjectId;
using ::zone::helper::math::BoundarySmoother;

static const bc::uint8_t kMaxTrajectoryPoint = 50;
static const bc::float32_t kMinTrajDxGap = 1.f;
static const bc::uint8_t kTransMatrixNum = 4;
static const bc::uint8_t kTrajectoryBoundaryNum = 4;
static const bc::uint8_t kSmoothWidth = 3;
static const bc::uint8_t kFrontFilterWidth = 4;
static const bc::uint8_t kBackFilterWidth = 1;
struct TrajectoryBoundary
{
  TrajectoryBoundary() : left_x_ar_(), left_y_ar_(), right_x_ar_(), right_y_ar_(), valid_point_ctns_u8_(0) {}
  bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> left_x_ar_;
  bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> left_y_ar_;
  bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> right_x_ar_;
  bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> right_y_ar_;
  bc::uint8_t valid_point_ctns_u8_;
};

class Trajectory
{
 public:
  Trajectory()
      : id_u16_(kInvalidObjectId),
        valid_point_ctns_u8_(0),
        trajectory_points_ar_(),
        after_choosed_accumu_point_num_(1),
        is_choosed_b_(bc::false_v)
  {
    for (bc::uint8_t i = 0; i < kMaxTrajectoryPoint; ++i)
    {
      trajectory_points_ar_[i] = Point2D();
      lst_cut_flag_ar_[i] = bc::false_v;
    }
    t_first_generate_traj_b_ = bc::true_v;
    lst_end_idx_u8_ = 0;
    lst_dx_f_ = 0;
  }
  virtual ~Trajectory() = default;

  bc::uint16_t GetTrajectoryId() const { return id_u16_; }

  void SetTrajectoryId(bc::uint16_t id_u16) { id_u16_ = id_u16; }

  bc::uint8_t GetTotalPointNum() const { return valid_point_ctns_u8_; }

  /** @brief Add new point to index 0 and remove the last one if the container is full
   *  @param new_point_cs (x, y)
   */
  void AddPoint(Point2D new_point_cs)
  {
    if (valid_point_ctns_u8_ > 0)
    {
      bc::float32_t t_difference_dx_f = new_point_cs.x - trajectory_points_ar_[0].x;
      if (t_difference_dx_f > kMinTrajDxGap)
      {
        // Always add a new point if there is not enough points to compare
        if (valid_point_ctns_u8_ == 1)
        {
          trajectory_points_ar_[1] = trajectory_points_ar_[0];
          valid_point_ctns_u8_++;
        }
        // Repalce the first point in trajectory if the first two stored points are too close
        // and add a new one if the distance of the first two points is larger than kMinTrajDxGap
        else if (trajectory_points_ar_[0].x - trajectory_points_ar_[1].x > kMinTrajDxGap)
        {
          // Move each point backforward by one position
          if (valid_point_ctns_u8_ == kMaxTrajectoryPoint)
          {
            for (bc::uint8_t i = valid_point_ctns_u8_ - 1; i >= 1; --i)
            {
              trajectory_points_ar_[i] = trajectory_points_ar_[i - 1];
            }
          }
          else
          {
            for (bc::int8_t i = valid_point_ctns_u8_ - 1; i >= 0; --i)
            {
              trajectory_points_ar_[i + 1] = trajectory_points_ar_[i];
            }
            valid_point_ctns_u8_++;
          }
        }
        if (is_choosed_b_)
        {
          after_choosed_accumu_point_num_++;
          if(after_choosed_accumu_point_num_ > kMaxTrajectoryPoint)
          {
            after_choosed_accumu_point_num_ = kMaxTrajectoryPoint;
          }
        }
      }
      // Consider the point that is not ahead of the previous one as invalid point and do nothing
      else
      {
        return;
      }
    }
    else
    {
      valid_point_ctns_u8_++;
    }
    trajectory_points_ar_[0] = new_point_cs;
  };

  /** @brief Do coordinate transformation on each point in trajectory
   *  @param trans_matrix_ar {cos(rot), sin(rot), -sin(rot), cos(rot)}
   *  @param translation_dx_f
   *  @param translation_dy_f
   */
  void CoordTransformation(bc::TCArray<bc::float32_t, kTransMatrixNum> trans_matrix_ar, bc::float32_t translation_dx_f,
                           bc::float32_t translation_dy_f)
  {
    bc::float32_t t_dx_f = 0.f;
    bc::float32_t t_dy_f = 0.f;
    for (bc::uint8_t i = 0; i < valid_point_ctns_u8_; ++i)
    {
      // Translation
      trajectory_points_ar_[i].x -= translation_dx_f;
      trajectory_points_ar_[i].y -= translation_dy_f;
      t_dx_f = trans_matrix_ar[0] * trajectory_points_ar_[i].x + trans_matrix_ar[1] * trajectory_points_ar_[i].y;
      t_dy_f = trans_matrix_ar[2] * trajectory_points_ar_[i].x + trans_matrix_ar[3] * trajectory_points_ar_[i].y;
      // Debug purpose only
      if (std::isnan(t_dx_f) || std::isnan(t_dy_f))
      {
#ifdef STD_COUT_ENABLE
        std::cout << "NAN in Environment Model trajectory transformation >> t_dy_f: " << t_dy_f << " t_dx_f:" << t_dx_f
                  << std::endl;
#endif
      }
      trajectory_points_ar_[i].x = t_dx_f;
      trajectory_points_ar_[i].y = t_dy_f;
    }
  }

  /** @brief Calculate left and right boundary points of the vehicle given host lane width;
   *  @param lane_width_f Host lane width
   *  @return TrajectoryBoundary, <left boundary dx, left boundary dy, right boundary dx, right boundary dy>
   */
  TrajectoryBoundary GetTrajectoryBoundary(bc::float32_t lane_width_f)
  {
    TrajectoryBoundary t_output_st;
    bc::uint8_t t_valid_point_after_choosed_num_u8 = 0;
    if (after_choosed_accumu_point_num_ == 0)
    {
      // Do nothing
    }
    else if (after_choosed_accumu_point_num_ == 1)
    {
      t_output_st.left_x_ar_[0] = trajectory_points_ar_[0].x;
      t_output_st.left_y_ar_[0] = trajectory_points_ar_[0].y - 0.5 * lane_width_f;
      t_output_st.right_x_ar_[0] = trajectory_points_ar_[0].x;
      t_output_st.right_y_ar_[0] = trajectory_points_ar_[0].y + 0.5 * lane_width_f;
      t_valid_point_after_choosed_num_u8 = after_choosed_accumu_point_num_;
    }
    else
    {
      t_valid_point_after_choosed_num_u8 = bc::min(kMaxTrajectoryPoint, after_choosed_accumu_point_num_);
      bc::uint8_t t_idx_u8 = 0;
      bc::TCArray<bc::uint8_t, 2> t_filter_width;
      t_filter_width[0] = kFrontFilterWidth;
      t_filter_width[1] = kBackFilterWidth;
      bc::TCArray<Point2D, kMaxTrajectoryPoint> t_smooth_traj_points = BoundarySmoother<Point2D, kMaxTrajectoryPoint>(
          t_valid_point_after_choosed_num_u8, 0, 3, 8.0, t_filter_width, trajectory_points_ar_);

      bc::uint8_t t_end_idx_u8 = t_valid_point_after_choosed_num_u8 - 1;
      bc::float32_t t_thresh_f = 0.3;
      bc::uint8_t t_cut_point_num = 0;
      bc::TCArray<bc::bool_t, kMaxTrajectoryPoint> t_cut_flag_ar;
      bc::TCArray<Point2D, kMaxTrajectoryPoint> t_cut_point_ar;
      bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> t_smooth_s_ar;
      bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> t_raw_s_ar;
      bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> t_smooth_s_cut_ar;
      bc::TCArray<bc::float32_t, kMaxTrajectoryPoint> t_w_ar;
      bc::float32_t t_sum_w_f = 0;
      t_smooth_s_ar[0] = 0;
      for (bc::uint8_t t_idx_u8 = 1; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
      {
        t_smooth_s_ar[t_idx_u8] =
            t_smooth_s_ar[t_idx_u8 - 1] +
            bc::sqrt(bc::pow(t_smooth_traj_points[t_idx_u8].x - t_smooth_traj_points[t_idx_u8 - 1].x, 2) +
                     bc::pow(t_smooth_traj_points[t_idx_u8].y - t_smooth_traj_points[t_idx_u8 - 1].y, 2));
      }
      t_raw_s_ar[0] = 0;
      for (bc::uint8_t t_idx_u8 = 1; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
      {
        t_raw_s_ar[t_idx_u8] =
            t_raw_s_ar[t_idx_u8 - 1] +
            bc::sqrt(bc::pow(trajectory_points_ar_[t_idx_u8].x - trajectory_points_ar_[t_idx_u8 - 1].x, 2) +
                     bc::pow(trajectory_points_ar_[t_idx_u8].y - trajectory_points_ar_[t_idx_u8 - 1].y, 2));
      }
      if (t_first_generate_traj_b_)
      {
        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxTrajectoryPoint; t_idx_u8++)
        {
          t_cut_flag_ar[t_idx_u8] = bc::false_v;
        }
      }
      else
      {
        bc::uint8_t t_add_num_u8 = 0;
        if (t_end_idx_u8 > lst_end_idx_u8_)
        {
          t_add_num_u8 = t_end_idx_u8 - lst_end_idx_u8_;
          for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < t_add_num_u8; ++t_idx_u8)
          {
            t_cut_flag_ar[t_idx_u8] = bc::false_v;
          }
          for (bc::uint8_t t_idx_u8 = t_add_num_u8; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
          {
            t_cut_flag_ar[t_idx_u8] = lst_cut_flag_ar_[t_idx_u8 - t_add_num_u8];
          }
        }
        else
        {
          if (trajectory_points_ar_[0].x > lst_dx_f_)
          {
            for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
            {
              if (trajectory_points_ar_[t_idx_u8].x > lst_dx_f_)
              {
                t_add_num_u8++;
              }
              else
              {
                break;
              }
            }
            for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < t_add_num_u8; t_idx_u8++)
            {
              t_cut_flag_ar[t_idx_u8] = bc::false_v;
            }
            for (bc::uint8_t t_idx_u8 = t_add_num_u8; t_idx_u8 < t_end_idx_u8; t_idx_u8++)
            {
              t_cut_flag_ar[t_idx_u8] = lst_cut_flag_ar_[t_idx_u8 - t_add_num_u8];
            }
          }
          else
          {
            t_cut_flag_ar = lst_cut_flag_ar_;
          }
        }
      }

      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
      {
        if (t_cut_flag_ar[t_idx_u8])
        {
          t_thresh_f = 0.15f;
        }
        else
        {
          t_thresh_f = 0.3f;
        }

        if (fabsf(t_smooth_traj_points[t_idx_u8].y - trajectory_points_ar_[t_idx_u8].y) < t_thresh_f)
        {
          t_cut_flag_ar[t_idx_u8] = bc::false_v;
          t_cut_point_ar[t_cut_point_num] = t_smooth_traj_points[t_idx_u8];
          t_smooth_s_cut_ar[t_cut_point_num] = t_smooth_s_ar[t_idx_u8];
          t_cut_point_num++;
        }
        else
        {
          t_cut_flag_ar[t_idx_u8] = bc::true_v;
        }
      }

      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 <= t_end_idx_u8; t_idx_u8++)
      {
        t_smooth_traj_points[t_idx_u8].y = 0;
        t_sum_w_f = 0.f;
        for (bc::uint8_t t_s_idx_u8 = 0; t_s_idx_u8 < t_cut_point_num; t_s_idx_u8++)
        {
          t_w_ar[t_s_idx_u8] = bc::exp(-(1.0f / (2.0f * kSmoothWidth * kSmoothWidth)) *
                                       pow(t_smooth_s_ar[t_idx_u8] - t_smooth_s_cut_ar[t_s_idx_u8], 2));
          t_sum_w_f += t_w_ar[t_s_idx_u8];
        }
        for (bc::uint8_t t_s_idx_u8 = 0; t_s_idx_u8 < t_cut_point_num; t_s_idx_u8++)
        {
          t_w_ar[t_s_idx_u8] = t_w_ar[t_s_idx_u8] / bc::max(t_sum_w_f, 0.000001f);
          t_smooth_traj_points[t_idx_u8].y += t_w_ar[t_s_idx_u8] * t_cut_point_ar[t_s_idx_u8].y;
        }
      }
      lst_cut_flag_ar_ = t_cut_flag_ar;
      lst_end_idx_u8_ = t_end_idx_u8;
      t_first_generate_traj_b_ = bc::false_v;

      // bc::TCArray<Point2D, kMaxTrajectoryPoint> t_filtered_traj_points = t_smooth_traj_points;
      // for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < valid_point_ctns_u8_; t_idx_u8++)
      // {
      //   if (valid_point_ctns_u8_ > 3)
      //   {
      //     if (t_idx_u8 == 0)
      //     {
      //       t_filtered_traj_points[t_idx_u8].y =
      //           (t_smooth_traj_points[t_idx_u8].y + t_smooth_traj_points[t_idx_u8 + 1].y) / 2.f;
      //     }
      //     else if (t_idx_u8 == valid_point_ctns_u8_ - 1)
      //     {
      //       t_filtered_traj_points[t_idx_u8].y =
      //           (t_smooth_traj_points[t_idx_u8].y + t_smooth_traj_points[t_idx_u8 - 1].y) / 2.f;
      //     }
      //     else
      //     {
      //       t_filtered_traj_points[t_idx_u8].y =
      //           (t_smooth_traj_points[t_idx_u8 - 1].y + t_smooth_traj_points[t_idx_u8].y +
      //           t_smooth_traj_points[t_idx_u8 + 1].y) / 3.f;
      //     }
      //   }else if(valid_point_ctns_u8_ == 3){
      //     t_filtered_traj_points[0].y =(t_smooth_traj_points[0].y + t_smooth_traj_points[1].y) / 2.f;
      //     t_filtered_traj_points[1].y =(t_smooth_traj_points[0].y + t_smooth_traj_points[1].y +
      //     t_smooth_traj_points[2].y) / 3.f;
      //     t_filtered_traj_points[2].y =(t_smooth_traj_points[1].y + t_smooth_traj_points[2].y) / 2.f;
      //   }else {
      //     ///do nothing
      //   }
      // }
      for (bc::uint8_t i = t_valid_point_after_choosed_num_u8 - 1; i >= 1; --i)
      {
        bc::float32_t t_cur_x_f = t_smooth_traj_points[i].x;
        bc::float32_t t_cur_y_f = t_smooth_traj_points[i].y;
        /// TODO: Use a more reliable way to find the slope of the current index of point in the whole trajectory
        // bc::float32_t t_theta_f =
        //     atan2f(trajectory_points_ar_[i - 1].y - t_cur_y_f, trajectory_points_ar_[i - 1].x - t_cur_x_f);
        bc::float32_t t_theta_f = 0.f;
        bc::float32_t t_move_x_f = 0.5 * lane_width_f * sinf(t_theta_f);
        bc::float32_t t_move_y_f = 0.5 * lane_width_f * cosf(t_theta_f);
        t_output_st.left_x_ar_[t_idx_u8] = t_cur_x_f - t_move_x_f;
        t_output_st.left_y_ar_[t_idx_u8] = t_cur_y_f + t_move_y_f;
        t_output_st.right_x_ar_[t_idx_u8] = t_cur_x_f + t_move_x_f;
        t_output_st.right_y_ar_[t_idx_u8] = t_cur_y_f - t_move_y_f;
        t_idx_u8++;
        if (i == 1)
        {
          t_output_st.left_x_ar_[t_idx_u8] = t_smooth_traj_points[0].x - t_move_x_f;
          t_output_st.left_y_ar_[t_idx_u8] = t_smooth_traj_points[0].y + t_move_y_f;
          t_output_st.right_x_ar_[t_idx_u8] = t_smooth_traj_points[0].x + t_move_x_f;
          t_output_st.right_y_ar_[t_idx_u8] = t_smooth_traj_points[0].y - t_move_y_f;
        }
      }
    }
    t_output_st.valid_point_ctns_u8_ = t_valid_point_after_choosed_num_u8;
    return t_output_st;
  }
  void SetIsChoosedBool(bc::bool_t is_choosed_b) { is_choosed_b_ = is_choosed_b; }
  bc::bool_t GetIsChoosedBool() { return is_choosed_b_; }
  void ResetAfterChoosedTotalPointNum() { after_choosed_accumu_point_num_ = 1; }

 private:
  bc::uint16_t id_u16_;
  bc::uint8_t valid_point_ctns_u8_;
  bc::TCArray<Point2D, kMaxTrajectoryPoint> trajectory_points_ar_;
  bc::bool_t t_first_generate_traj_b_;
  bc::uint8_t lst_end_idx_u8_;
  bc::TCArray<bc::bool_t, kMaxTrajectoryPoint> lst_cut_flag_ar_;
  bc::float32_t lst_dx_f_;
  bc::uint8_t after_choosed_accumu_point_num_;
  bc::bool_t is_choosed_b_;
};

class ObjectTrajectoryCollection
{
 public:
  ObjectTrajectoryCollection() : valid_object_trajectory_ctns_u8_(0), object_idx_ar_(), trajectory_collection_ar_()
  {
    for (bc::uint8_t i = 0; i < kFusMaxObjNum; ++i)
    {
      object_idx_ar_[i] = kFusMaxObjNum;
      trajectory_collection_ar_[i] = Trajectory();
    }
  }
  virtual ~ObjectTrajectoryCollection() = default;

  /** @brief If the given index exists and ids are matching, add the new point to existed trajectory;
   *  Othervise, add new point to a new trajectory or replace the old trajectory with a new one.
   *  @param index_u8 Object index
   *  @param point_cs (x, y)
   *  @param id_u16 Object id
   */
  void AddPointToTrajectoryByIdx(bc::uint8_t index_u8, Point2D point_cs, bc::uint16_t id_u16)
  {
    // Add new trajectory point to existed object trajectory
    if (trajectory_collection_ar_[index_u8].GetTrajectoryId() == id_u16)
    {
      trajectory_collection_ar_[index_u8].AddPoint(point_cs);
    }
    // Add a brand new object trajectoty from the start
    else if (trajectory_collection_ar_[index_u8].GetTrajectoryId() == kInvalidObjectId)
    {
      trajectory_collection_ar_[index_u8].AddPoint(point_cs);
      trajectory_collection_ar_[index_u8].SetTrajectoryId(id_u16);
      object_idx_ar_[valid_object_trajectory_ctns_u8_] = index_u8;
      valid_object_trajectory_ctns_u8_++;
    }
    // Replace an old object trajectoty with a new one
    else
    {
      trajectory_collection_ar_[index_u8] = Trajectory();
      trajectory_collection_ar_[index_u8].AddPoint(point_cs);
      trajectory_collection_ar_[index_u8].SetTrajectoryId(id_u16);
    }
  }

  /** @brief If the given index exists, reset the trajectory buffer; Othervise, do nothing.
   *  @param index_u8 Object index
   */
  void ResetTrajectoryByIdx(bc::uint8_t index_u8)
  {
    trajectory_collection_ar_[index_u8] = Trajectory();
    // Update valid object index array
    for (bc::uint8_t i = 0U; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      if (object_idx_ar_[i] == index_u8)
      {
        // Move rest valid object index one position ahead to fill empty position
        for (; i < valid_object_trajectory_ctns_u8_ - 1; ++i)
        {
          object_idx_ar_[i] = object_idx_ar_[i + 1];
        }
        object_idx_ar_[i] = kFusMaxObjNum;
        // Update valid_object_trajectory_ctns_u8_
        valid_object_trajectory_ctns_u8_--;
        return;
      }
    }
  }

  /** @brief Do coordinate transformation for each valid trajectory;
   *  @param rotation_f
   *  @param translation_dx_f
   *  @param translation_dy_f
   */
  void DoCoordTransToTrajectory(bc::float32_t rotation_f, bc::float32_t translation_dx_f,
                                bc::float32_t translation_dy_f)
  {
    bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(rotation_f), sinf(rotation_f),
                                                                     -sinf(rotation_f), cosf(rotation_f)};
    for (bc::uint8_t i = 0; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      trajectory_collection_ar_[object_idx_ar_[i]].CoordTransformation(t_trans_matrix_ar, translation_dx_f,
                                                                       translation_dy_f);
    }
  }

  /** @brief Check whether the selected trajectory is available;
   *  @param index_u8 Object index
   *  @return boolean
   */
  bc::bool_t GetTrajectoryValidityByIdx(bc::uint8_t index_u8)
  {
    return trajectory_collection_ar_[index_u8].GetTrajectoryId() != kInvalidObjectId;
  }
  void SetTrajectoryInValidityByIdx(bc::uint8_t index_u8)
  {
    trajectory_collection_ar_[index_u8].SetTrajectoryId(kInvalidObjectId);
  }

  /** @brief Calculate left and right boundary points of the leading vehicle given host lane width;
   *  @param index_u8 Object index
   *  @param lane_width_f Host lane width
   *  @return TrajectoryBoundary, <left boundary dx, left boundary dy, right boundary dx, right boundary dy>
   */
  TrajectoryBoundary GetTargetVehicleLaneBoundary(bc::uint8_t index_u8, bc::float32_t lane_width_f)
  {
    return trajectory_collection_ar_[index_u8].GetTrajectoryBoundary(lane_width_f);
  }

  bc::uint8_t GetValidObjectTractoryCtns() const { return valid_object_trajectory_ctns_u8_; }

  bc::TCArray<bc::uint8_t, kFusMaxObjNum> GetObjectIndices() const { return object_idx_ar_; }

  /** @brief Get number of trajectory point with given object index;
   *  @param index_u8 Object index
   *  @return uint8
   */
  bc::uint8_t GetTrajectoryPointNumByIdx(bc::uint8_t index_u8) const
  {
    return trajectory_collection_ar_[index_u8].GetTotalPointNum();
  }

  Trajectory& GetTargetTrajectoryByIdx(bc::uint8_t index_u8) { return trajectory_collection_ar_[index_u8]; }

 private:
  bc::uint8_t valid_object_trajectory_ctns_u8_;
  bc::TCArray<bc::uint8_t, kFusMaxObjNum> object_idx_ar_;
  bc::TCArray<Trajectory, kFusMaxObjNum> trajectory_collection_ar_;
};
}
}
#endif