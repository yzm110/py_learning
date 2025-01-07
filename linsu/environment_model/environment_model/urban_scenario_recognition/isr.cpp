#include "isr.h"

namespace zone {
namespace environment_model {

IntersectionRecognition::IntersectionRecognition(EmData& em_collection)
    : em_collection_ptr_(&em_collection), time_cycle_f_(0.0f)

{
  ResetIntersection();
}

void IntersectionRecognition::Run(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                                  const StaticObjProcess& static_obj_process_cs)
{
  SetInput(static_obj_process_cs, ego_motion_cs);

  /** 1. Tracking intersection bounds by ego motion. */
  TrackingIntersectionBound();

  UpdateIntersectionManager(static_obj_process_cs);
  switch (its_manager_cs_.state_en_)
  {
    case IntersectionManagerState::kNoIts:
      its_build_success_b_ = bc::false_v;
      ResetIntersection();
      break;

    case IntersectionManagerState::kBuildingIts:
      BuildingIntersection();
      break;

    case IntersectionManagerState::kPassingIts:
      ExploringIntersection();
      break;
    case IntersectionManagerState::kExitingIts:

      break;
    default:
      break;
  }
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 4; t_idx_u8++)
  {
    its_bound_pts_lst1_ar_[t_idx_u8] = its_bound_pts_ar_[t_idx_u8];
    its_bound_lst1_ar_[t_idx_u8] = its_bound_ar_[t_idx_u8];
  }
}

void IntersectionRecognition::SetInput(const StaticObjProcess& static_obj_process_cs,
                                       const EgoPoseCollection& ego_motion_cs)
{
  cross_stop_line_vt_ = static_obj_process_cs.cross_stop_line_output_vt_;
  traffic_light_vt_ = static_obj_process_cs.traffic_light_output_vt_;
  v_ego_f_ = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetLongVelocity();
  transform_st = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetTransformFromLastCycle();
}
void IntersectionRecognition::ResetIntersection()
{
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 4; t_idx_u8++)
  {
    its_bound_pts_ar_[t_idx_u8] = IntersectionBoundPoint();
    its_bound_ar_[t_idx_u8] = IntersectionBound();
    its_entry_exit_pts_ar_[t_idx_u8] = IntersectionRefPoint();
  }
}
void IntersectionRecognition::TrackingIntersectionBound()
{

  bc::float32_t t_rotation_f = transform_st.rotation_f_;
  bc::float32_t t_dx_trans_f = transform_st.translation_dx_f_;
  bc::float32_t t_dy_trans_f = transform_st.translation_dy_f_;
  bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f),
                                                                   -sinf(t_rotation_f), cosf(t_rotation_f)};
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 4; t_idx_u8++)
  {
    if (its_build_success_b_ == bc::true_v)
    {
      /** TODO: update dx,dy by ego motion*/
      its_bound_pts_ar_[t_idx_u8].point_cs_.x = its_bound_pts_ar_[t_idx_u8].point_cs_.x - t_dx_trans_f;
      its_bound_pts_ar_[t_idx_u8].point_cs_.y = its_bound_pts_ar_[t_idx_u8].point_cs_.y - t_dy_trans_f;
      bc::float32_t t_dx_f = t_trans_matrix_ar[0] * its_bound_pts_ar_[t_idx_u8].point_cs_.x +
                             t_trans_matrix_ar[1] * its_bound_pts_ar_[t_idx_u8].point_cs_.y;
      bc::float32_t t_dy_f = t_trans_matrix_ar[2] * its_bound_pts_ar_[t_idx_u8].point_cs_.x +
                             t_trans_matrix_ar[3] * its_bound_pts_ar_[t_idx_u8].point_cs_.y;

      its_bound_pts_ar_[t_idx_u8].point_cs_.x = t_dx_f;
      its_bound_pts_ar_[t_idx_u8].point_cs_.y = t_dy_f;

      its_bound_ar_[t_idx_u8].center_point_cs_.x = its_bound_ar_[t_idx_u8].center_point_cs_.x - t_dx_trans_f;
      its_bound_ar_[t_idx_u8].center_point_cs_.y = its_bound_ar_[t_idx_u8].center_point_cs_.y - t_dy_trans_f;
      t_dx_f = t_trans_matrix_ar[0] * its_bound_ar_[t_idx_u8].center_point_cs_.x +
               t_trans_matrix_ar[1] * its_bound_ar_[t_idx_u8].center_point_cs_.y;
      t_dy_f = t_trans_matrix_ar[2] * its_bound_ar_[t_idx_u8].center_point_cs_.x +
               t_trans_matrix_ar[3] * its_bound_ar_[t_idx_u8].center_point_cs_.y;

      its_bound_ar_[t_idx_u8].center_point_cs_.x = t_dx_f;
      its_bound_ar_[t_idx_u8].center_point_cs_.y = t_dy_f;
      its_bound_ar_[t_idx_u8].heading_norm_f_ = its_bound_ar_[t_idx_u8].heading_norm_f_ - t_rotation_f;

      its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.x =
          its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.x - t_dx_trans_f;
      its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.y =
          its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.y - t_dy_trans_f;

      t_dx_f = t_trans_matrix_ar[0] * its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.x +
               t_trans_matrix_ar[1] * its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.y;
      t_dy_f = t_trans_matrix_ar[2] * its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.x +
               t_trans_matrix_ar[3] * its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.y;
      its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.x = t_dx_f;
      its_entry_exit_pts_ar_[t_idx_u8].ref_point_cs_.y = t_dy_f;
    }
  }
}
void IntersectionRecognition::UpdateIntersectionManager(const StaticObjProcess& static_obj_process_cs)
{
  bc::bool_t t_exiting_b = bc::false_v;
  switch (its_manager_cs_.state_en_)
  {
    case IntersectionManagerState::kNoIts:

      if ((!cross_stop_line_vt_.empty()) && (!traffic_light_vt_.empty()))
      {
        its_manager_cs_.state_en_ = IntersectionManagerState::kBuildingIts;
      }
      break;

    case IntersectionManagerState::kBuildingIts:
      if (life_time_f_ < time_cycle_f_)
      {
        its_manager_cs_.state_en_ = IntersectionManagerState::kNoIts;
      }
      else if ((((v_ego_f_ > 1.0) && (its_entry_exit_pts_ar_[kIdxBoundEntry].ref_point_cs_.x < 1.0)) ||
                (its_entry_exit_pts_ar_[kIdxBoundEntry].ref_point_cs_.x < -1.0)) &&
               (its_build_success_b_ == bc::true_v))
      {
        its_manager_cs_.state_en_ = IntersectionManagerState::kPassingIts;
      }

      break;

    case IntersectionManagerState::kPassingIts:

      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 4; t_idx_u8++)
      {
        if (its_bound_pts_ar_[t_idx_u8].point_cs_.x > 0.0)
        {
          t_exiting_b = bc::true_v;
        }
      }
      if (t_exiting_b == bc::false_v)
      {
        its_manager_cs_.state_en_ = IntersectionManagerState::kExitingIts;
      }
      break;

    case IntersectionManagerState::kExitingIts:
      its_manager_cs_.state_en_ = IntersectionManagerState::kNoIts;

      break;
    default:
      break;
  }
}
void IntersectionRecognition::BuildingIntersection()
{
  bc::float32_t t_dx_cross_stop_line_f = FLOAT32_MAX;
  bc::float32_t t_dx_traffic_light_f = FLOAT32_MAX;

  bc::float32_t t_dy_crossline_left_f = FLOAT32_MIN;
  bc::float32_t t_dy_crossline_right_f = FLOAT32_MAX;
  bc::bool_t t_cross_stop_line_found_b = bc::false_v;
  EmCrossStopLineType t_cross_stop_line_found_type_en = EmCrossStopLineType::kUnknown;
  bc::bool_t t_traffic_light_found_b = bc::false_v;
  bc::bool_t t_crossline_left_found_b = bc::false_v;
  bc::bool_t t_crossline_right_found_b = bc::false_v;
  bc::uint8_t t_idx_cross_stop_line_ahead_u8 = 0;
  bc::uint8_t t_idx_traffic_light_ahead_u8 = 0;

  for (auto iter_ptr = traffic_light_vt_.begin(); iter_ptr != traffic_light_vt_.end(); iter_ptr++)
  {
    if ((iter_ptr->distilled_b_ == bc::true_v) && (iter_ptr->dx_f_ > 0.0) && (iter_ptr->dx_f_ < t_dx_traffic_light_f))
    {
      t_dx_traffic_light_f = iter_ptr->dx_f_;
      t_traffic_light_found_b = bc::true_v;
      t_idx_traffic_light_ahead_u8 = std::distance(traffic_light_vt_.begin(), iter_ptr);
    }
  }

  for (auto iter_ptr = cross_stop_line_vt_.begin(); iter_ptr != cross_stop_line_vt_.end(); iter_ptr++)
  {
    bc::bool_t t_cross_replace_stop_line_b = bc::false_v;

    if (((t_cross_stop_line_found_type_en == EmCrossStopLineType::kUnknown) ||
         (t_cross_stop_line_found_type_en == EmCrossStopLineType::kStopLine)) &&
        ((iter_ptr->type_en_ == EmCrossStopLineType::kCrossAndStopLine) ||
         (iter_ptr->type_en_ == EmCrossStopLineType::kCrossLine)))
    {
      t_cross_replace_stop_line_b = bc::true_v;
    }

    if ((iter_ptr->distilled_b_ == bc::true_v) && (iter_ptr->dx_f_ > -20.0) &&
        ((iter_ptr->dx_f_ < t_dx_cross_stop_line_f) || (t_cross_replace_stop_line_b == bc::true_v)) &&
        (iter_ptr->dx_f_ < (t_dx_traffic_light_f - 10.0)) && (t_traffic_light_found_b == bc::true_v))
    {
      t_cross_stop_line_found_type_en = iter_ptr->type_en_;
      t_dx_cross_stop_line_f = iter_ptr->dx_f_;
      t_cross_stop_line_found_b = bc::true_v;
      t_idx_cross_stop_line_ahead_u8 = std::distance(cross_stop_line_vt_.begin(), iter_ptr);
    }

    if ((iter_ptr->dx_f_ > 0.0) && (iter_ptr->dy_f_ > 0.0) && (iter_ptr->dy_f_ > t_dy_crossline_left_f) &&
        ((iter_ptr->type_en_ == EmCrossStopLineType::kCrossAndStopLine) ||
         (iter_ptr->type_en_ == EmCrossStopLineType::kCrossLine)) &&
        ((iter_ptr->heading_f_ > (G_PI / 3.0)) || (iter_ptr->heading_f_ < (-G_PI / 3.0))))
    {
      t_dy_crossline_left_f = iter_ptr->dy_f_;
      t_crossline_left_found_b = bc::true_v;
    }

    if ((iter_ptr->dx_f_ > 0.0) && (iter_ptr->dy_f_ < 0.0) && (iter_ptr->dy_f_ < t_dy_crossline_right_f) &&
        ((iter_ptr->type_en_ == EmCrossStopLineType::kCrossAndStopLine) ||
         (iter_ptr->type_en_ == EmCrossStopLineType::kCrossLine)) &&
        ((iter_ptr->heading_f_ > (G_PI / 3.0)) || (iter_ptr->heading_f_ < (-G_PI / 3.0))))
    {
      t_dy_crossline_right_f = iter_ptr->dy_f_;
      t_crossline_right_found_b = bc::true_v;
    }
  }

  if ((t_cross_stop_line_found_b == bc::true_v) && (t_traffic_light_found_b == bc::true_v) &&
      (t_dx_cross_stop_line_f < (t_dx_traffic_light_f - 5.0)))
  {
    /** intersection triggered. */

    bc::float32_t t_dx_f = cross_stop_line_vt_[t_idx_cross_stop_line_ahead_u8].dx_f_;
    bc::float32_t t_dy_f = cross_stop_line_vt_[t_idx_cross_stop_line_ahead_u8].dy_f_;
    bc::float32_t t_length_f = cross_stop_line_vt_[t_idx_cross_stop_line_ahead_u8].length_side_f_;
    bc::float32_t t_heading_f_ = cross_stop_line_vt_[t_idx_cross_stop_line_ahead_u8].heading_f_;

    its_entry_exit_pts_ar_[kIdxBoundEntry].ref_point_cs_.x = t_dx_f;
    its_entry_exit_pts_ar_[kIdxBoundEntry].ref_point_cs_.y = t_dy_f;

    bc::float32_t t_dx_btw_upper_lower_f = t_dx_traffic_light_f - t_dx_cross_stop_line_f;
    t_dx_btw_upper_lower_f = t_dx_btw_upper_lower_f * 0.9;
    bc::float32_t t_length_left_f = 0.5 * t_length_f + t_length_f;
    bc::float32_t t_length_left_by_dx_f = 0.3 * t_dx_btw_upper_lower_f;
    bc::float32_t t_length_left_by_neighbor_cross_f = 0.0;
    if (t_crossline_left_found_b == bc::true_v)
    {
      t_length_left_by_neighbor_cross_f = std::abs(t_dy_crossline_left_f - t_dy_f);
    }
    bc::float32_t t_length_right_by_neighbor_cross_f = 0.0;
    if (t_crossline_right_found_b == bc::true_v)
    {
      t_length_right_by_neighbor_cross_f = std::abs(t_dy_crossline_right_f - t_dy_f);
    }

    t_length_left_f = std::max(t_length_left_f, t_length_left_by_dx_f);
    t_length_left_f = std::max(t_length_left_f, t_length_left_by_neighbor_cross_f);
    bc::float32_t t_length_right_f = 0.5 * t_length_f + 3.5;
    t_length_right_f = std::max(t_length_right_f, t_length_right_by_neighbor_cross_f);

    bc::float32_t t_delta_dx_left_f = -t_length_left_f * sinf(t_heading_f_);
    bc::float32_t t_delta_dy_left_f = t_length_left_f * cosf(t_heading_f_);
    bc::float32_t t_delta_dx_right_f = t_length_right_f * sinf(t_heading_f_);
    bc::float32_t t_delta_dy_right_f = -t_length_right_f * cosf(t_heading_f_);

    bc::float32_t t_dx_lower_left_f = t_dx_f + t_delta_dx_left_f;
    bc::float32_t t_dy_lower_left_f = t_dy_f + t_delta_dy_left_f;
    if (its_bound_pts_ar_[kIdxBoundPtLowerLeft].valid_b_ == bc::false_v)
    {
      its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.x = t_dx_lower_left_f;
      its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.y = t_dy_lower_left_f;
    }
    else
    {
      bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, life_time_f_, Lin_Interp_Method::kFlat);
      its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.x =
          t_k_f * its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.x + (1 - t_k_f) * t_dx_lower_left_f;
      its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.y =
          t_k_f * its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.y + (1 - t_k_f) * t_dy_lower_left_f;
    }

    bc::float32_t t_dx_lower_right_f = t_dx_f + t_delta_dx_right_f;
    bc::float32_t t_dy_lower_right_f = t_dy_f + t_delta_dy_right_f;
    if (its_bound_pts_ar_[kIdxBoundPtLowerRight].valid_b_ == bc::false_v)
    {
      its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.x = t_dx_lower_right_f;
      its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.y = t_dy_lower_right_f;
    }
    else
    {
      bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, life_time_f_, Lin_Interp_Method::kFlat);
      its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.x =
          t_k_f * its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.x + (1 - t_k_f) * t_dx_lower_right_f;
      its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.y =
          t_k_f * its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.y + (1 - t_k_f) * t_dy_lower_right_f;
    }

    life_time_f_ = life_time_f_ + time_cycle_f_;

    /** calc intersection bound points UpperRight and UpperLeft */
    bc::float32_t t_alpha_norm_f = atan(
        -(its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.x - its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.x) /
        (its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.y - its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.y));

    bc::float32_t t_delta_dx_f = t_dx_btw_upper_lower_f * cosf(t_alpha_norm_f);
    bc::float32_t t_delta_dy_f = t_dx_btw_upper_lower_f * sinf(t_alpha_norm_f);

    its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_.x =
        its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.x + t_delta_dx_f;
    its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_.y =
        its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_.y + t_delta_dy_f;

    its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_.x =
        its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.x + t_delta_dx_f;
    its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_.y =
        its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_.y + t_delta_dy_f;

    /** exit_straight_ref_point */
    UpdateEntryExitPoints(kIdxBoundStraight);
    UpdateEntryExitPoints(kIdxBoundLeft);
    UpdateEntryExitPoints(kIdxBoundRight);

    its_build_success_b_ = bc::true_v;
    UpdateBoundByBoundPoints(kIdxBoundEntry);
    UpdateBoundByBoundPoints(kIdxBoundStraight);
    UpdateBoundByBoundPoints(kIdxBoundLeft);
    UpdateBoundByBoundPoints(kIdxBoundRight);
  }
  else
  {
    life_time_f_ = life_time_f_ - time_cycle_f_;
    life_time_f_ = std::max(life_time_f_, (bc::float32_t)(0.0));
  }
}

void IntersectionRecognition::ExploringIntersection()
{
  /** update bounds*/
  for (auto iter_ptr = cross_stop_line_vt_.begin(); iter_ptr != cross_stop_line_vt_.end(); iter_ptr++)
  {
    if ((iter_ptr->distilled_b_ == bc::false_v) && ((iter_ptr->type_en_ == EmCrossStopLineType::kCrossAndStopLine) ||
                                                    (iter_ptr->type_en_ == EmCrossStopLineType::kCrossLine)) &&
        (iter_ptr->updated_b_ == bc::true_v))
    {
      CrosslineBoundMatching(iter_ptr, its_bound_ar_[kIdxBoundEntry]);
      CrosslineBoundMatching(iter_ptr, its_bound_ar_[kIdxBoundStraight]);
      CrosslineBoundMatching(iter_ptr, its_bound_ar_[kIdxBoundLeft]);
      CrosslineBoundMatching(iter_ptr, its_bound_ar_[kIdxBoundRight]);
    }
  }

  /** update bound points*/
  // static const bc::uint8_t kIdxBoundPtLowerLeft = 0;
  // static const bc::uint8_t kIdxBoundPtLowerRight = 1;
  // static const bc::uint8_t kIdxBoundPtUpperRight = 2;
  // static const bc::uint8_t kIdxBoundPtUpperLeft = 3;
  UpdateBoundPointsByBound(kIdxBoundPtLowerLeft);
  UpdateBoundPointsByBound(kIdxBoundPtLowerRight);
  UpdateBoundPointsByBound(kIdxBoundPtUpperRight);
  UpdateBoundPointsByBound(kIdxBoundPtUpperLeft);

  /** update bounds again*/
  UpdateBoundByBoundPoints(kIdxBoundEntry);
  UpdateBoundByBoundPoints(kIdxBoundStraight);
  UpdateBoundByBoundPoints(kIdxBoundLeft);
  UpdateBoundByBoundPoints(kIdxBoundRight);

  /** update entry and exit point */
  UpdateEntryExitPoints(kIdxBoundStraight);
  UpdateEntryExitPoints(kIdxBoundLeft);
  UpdateEntryExitPoints(kIdxBoundRight);
}

void IntersectionRecognition::CrosslineBoundMatching(const CrossStopLineInfo* iter_ptr, IntersectionBound& its_bound_cs)
{
  Point3D t_pt_start_cs = Point3D();
  Point3D t_pt_end_cs = Point3D();
  Point3D t_pt_start_dir_cs = Point3D();
  Point3D t_pt_end_dir_cs = Point3D();
  Point3D t_pt_cs = Point3D();

  t_pt_cs.x = iter_ptr->dx_f_;
  t_pt_cs.y = iter_ptr->dy_f_;
  bc::float32_t t_length_crossline_f = iter_ptr->length_side_f_;
  bc::float32_t t_heading_norm_crossline_f = iter_ptr->heading_f_;

  /** calc distance between pt and (pt_start,pt_end)*/

  bc::float32_t t_heading_norm_bound_f = its_bound_cs.heading_norm_f_;
  bc::float32_t t_length_bound_f = its_bound_cs.length_f_;
  bc::float32_t t_delta_x_f = -0.5 * t_length_bound_f * sinf(t_heading_norm_bound_f);
  bc::float32_t t_delta_y_f = 0.5 * t_length_bound_f * cos(t_heading_norm_bound_f);
  t_pt_start_cs.x = its_bound_cs.center_point_cs_.x + t_delta_x_f;
  t_pt_start_cs.y = its_bound_cs.center_point_cs_.y + t_delta_y_f;

  bc::float32_t t_vec_norm_bound_x_f = cosf(t_heading_norm_bound_f);
  bc::float32_t t_vec_norm_bound_y_f = sinf(t_heading_norm_bound_f);

  /** create a vector of bound point and crossline point*/
  bc::float32_t t_vec_x_f = t_pt_start_cs.x - t_pt_cs.x;
  bc::float32_t t_vec_y_f = t_pt_start_cs.y - t_pt_cs.y;

  /** calc the projection of this vector on unit norm vector*/
  bc::float32_t t_d_proj_f = std::abs(t_vec_x_f * t_vec_norm_bound_x_f + t_vec_y_f * t_vec_norm_bound_y_f);

  HeadingRangeNormalize(t_heading_norm_crossline_f);

  HeadingRangeNormalize(t_heading_norm_bound_f);

  bc::float32_t t_heading_norm_diff_f = std::abs(t_heading_norm_crossline_f - t_heading_norm_bound_f);
  if (t_heading_norm_diff_f > G_PI_2)
  {
    if (t_heading_norm_crossline_f < 0.0)
    {
      t_heading_norm_crossline_f = t_heading_norm_crossline_f + G_PI;
    }
    else
    {
      t_heading_norm_crossline_f = t_heading_norm_crossline_f - G_PI;
    }
    t_heading_norm_diff_f = std::abs(t_heading_norm_crossline_f - t_heading_norm_bound_f);
  }

  if ((t_d_proj_f < 20.0) && (t_heading_norm_diff_f < G_PI / 6.0))
  {
    /** matched, update its bound points */
    its_bound_cs.center_point_cs_.x = 0.95 * its_bound_cs.center_point_cs_.x + 0.05 * t_pt_cs.x;
    its_bound_cs.center_point_cs_.y = 0.95 * its_bound_cs.center_point_cs_.y + 0.05 * t_pt_cs.y;
    its_bound_cs.heading_norm_f_ = 0.95 * t_heading_norm_bound_f + 0.05 * t_heading_norm_crossline_f;
    HeadingRangeNormalize(its_bound_cs.heading_norm_f_);
  }
}

void IntersectionRecognition::UpdateBoundByBoundPoints(const bc::uint8_t bound_type_u8)
{
  // static const bc::uint8_t kIdxBoundEntry = 0;
  // static const bc::uint8_t kIdxBoundStraight = 1;
  // static const bc::uint8_t kIdxBoundLeft = 2;
  // static const bc::uint8_t kIdxBoundRight = 3;
  Point3D t_pt_1_cs = Point3D();
  Point3D t_pt_2_cs = Point3D();
  switch (bound_type_u8)
  {
    // static const bc::uint8_t kIdxBoundPtLowerLeft = 0;
    // static const bc::uint8_t kIdxBoundPtLowerRight = 1;
    // static const bc::uint8_t kIdxBoundPtUpperRight = 2;
    // static const bc::uint8_t kIdxBoundPtUpperLeft = 3;
    case kIdxBoundEntry:
      t_pt_1_cs = its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_;
      t_pt_2_cs = its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_;
      break;

    case kIdxBoundStraight:
      t_pt_1_cs = its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_;
      t_pt_2_cs = its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_;
      break;

    case kIdxBoundLeft:
      t_pt_1_cs = its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_;
      t_pt_2_cs = its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_;
      break;

    case kIdxBoundRight:
      t_pt_1_cs = its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_;
      t_pt_2_cs = its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_;
      break;

    default:
      return;
      break;
  }

  its_bound_ar_[bound_type_u8].center_point_cs_.x = 0.5 * (t_pt_1_cs.x + t_pt_2_cs.x);
  its_bound_ar_[bound_type_u8].center_point_cs_.y = 0.5 * (t_pt_1_cs.y + t_pt_2_cs.y);

  bc::float32_t t_alpha_norm_f = atan(-(t_pt_1_cs.x - t_pt_2_cs.x) / (t_pt_1_cs.y - t_pt_2_cs.y));
  HeadingRangeNormalize(t_alpha_norm_f);
  its_bound_ar_[bound_type_u8].heading_norm_f_ = t_alpha_norm_f;
  bc::float32_t t_dx_f = t_pt_1_cs.x - t_pt_2_cs.x;
  bc::float32_t t_dy_f = t_pt_1_cs.y - t_pt_2_cs.y;
  its_bound_ar_[bound_type_u8].length_f_ = sqrt(t_dx_f * t_dx_f + t_dy_f * t_dy_f);
  its_bound_ar_[bound_type_u8].valid_b_ = bc::true_v;
}

void IntersectionRecognition::UpdateBoundPointsByBound(const bc::uint8_t point_type_u8)
{
  // static const bc::uint8_t kIdxBoundPtLowerLeft = 0;
  // static const bc::uint8_t kIdxBoundPtLowerRight = 1;
  // static const bc::uint8_t kIdxBoundPtUpperRight = 2;
  // static const bc::uint8_t kIdxBoundPtUpperLeft = 3;

  // static const bc::uint8_t kIdxBoundEntry = 0;
  // static const bc::uint8_t kIdxBoundStraight = 1;
  // static const bc::uint8_t kIdxBoundLeft = 2;
  // static const bc::uint8_t kIdxBoundRight = 3;
  IntersectionBound t_bound_base_cs = IntersectionBound();
  IntersectionBound t_bound_cross_cs = IntersectionBound();
  switch (point_type_u8)
  {
    case kIdxBoundPtLowerLeft:
      t_bound_base_cs = its_bound_ar_[kIdxBoundEntry];
      t_bound_cross_cs = its_bound_ar_[kIdxBoundLeft];
      break;

    case kIdxBoundPtLowerRight:
      t_bound_base_cs = its_bound_ar_[kIdxBoundEntry];
      t_bound_cross_cs = its_bound_ar_[kIdxBoundRight];
      break;

    case kIdxBoundPtUpperRight:
      t_bound_base_cs = its_bound_ar_[kIdxBoundStraight];
      t_bound_cross_cs = its_bound_ar_[kIdxBoundRight];
      break;

    case kIdxBoundPtUpperLeft:
      t_bound_base_cs = its_bound_ar_[kIdxBoundStraight];
      t_bound_cross_cs = its_bound_ar_[kIdxBoundLeft];
      break;

    default:
      return;
      break;
  }

  HeadingRangeNormalize(t_bound_base_cs.heading_norm_f_);
  HeadingRangeNormalize(t_bound_cross_cs.heading_norm_f_);
  bc::float32_t t_heading_diff_f = std::abs(t_bound_base_cs.heading_norm_f_ - t_bound_cross_cs.heading_norm_f_);
  if (t_heading_diff_f < 0.057)
  {
    /** almost parallel, return.*/
    return;
  }

  bc::float32_t t_dx_base_f = t_bound_base_cs.center_point_cs_.x;
  bc::float32_t t_dy_base_f = t_bound_base_cs.center_point_cs_.y;
  bc::float32_t t_heading_base_center_f = G_PI_2 + t_bound_base_cs.heading_norm_f_;
  HeadingRangeNormalize(t_heading_base_center_f);
  bc::float32_t t_unit_base_x_f = cosf(t_heading_base_center_f);
  bc::float32_t t_unit_base_y_f = sinf(t_heading_base_center_f);

  bc::float32_t t_dx_cross_f = t_bound_cross_cs.center_point_cs_.x;
  bc::float32_t t_dy_cross_f = t_bound_cross_cs.center_point_cs_.y;
  bc::float32_t t_heading_cross_center_f = G_PI_2 + t_bound_cross_cs.heading_norm_f_;
  HeadingRangeNormalize(t_heading_cross_center_f);
  bc::float32_t t_unit_cross_x_f = cosf(t_heading_cross_center_f);
  bc::float32_t t_unit_cross_y_f = sinf(t_heading_cross_center_f);

  bc::float32_t t_k_f = (t_dy_base_f * t_unit_cross_x_f - t_dy_cross_f * t_unit_cross_x_f -
                         t_dx_base_f * t_unit_cross_y_f + t_dx_cross_f * t_unit_cross_y_f) /
                        (t_unit_base_x_f * t_unit_cross_y_f - t_unit_base_y_f * t_unit_cross_x_f);
  its_bound_pts_ar_[point_type_u8].point_cs_.x = t_dx_base_f + t_k_f * t_unit_base_x_f;
  its_bound_pts_ar_[point_type_u8].point_cs_.y = t_dy_base_f + t_k_f * t_unit_base_y_f;
}

void IntersectionRecognition::UpdateEntryExitPoints(const bc::uint8_t point_type_u8)
{
  Point3D t_base_pt_cs = Point3D();
  Point3D t_end_pt_cs = Point3D();
  switch (point_type_u8)
  {

    case kIdxBoundStraight:
      t_base_pt_cs = its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_;
      t_end_pt_cs = its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_;
      break;

    case kIdxBoundLeft:
      t_base_pt_cs = its_bound_pts_ar_[kIdxBoundPtLowerLeft].point_cs_;
      t_end_pt_cs = its_bound_pts_ar_[kIdxBoundPtUpperLeft].point_cs_;
      break;

    case kIdxBoundRight:
      t_base_pt_cs = its_bound_pts_ar_[kIdxBoundPtUpperRight].point_cs_;
      t_end_pt_cs = its_bound_pts_ar_[kIdxBoundPtLowerRight].point_cs_;
      break;
    default:
      return;
      break;
  }

  /** exit_left_ref_point, unit vector is LowerLeft to UpperLeft */
  bc::float32_t t_unit_vector_x_f = t_end_pt_cs.x - t_base_pt_cs.x;
  bc::float32_t t_unit_vector_y_f = t_end_pt_cs.y - t_base_pt_cs.y;
  bc::float32_t t_length_f = sqrt(t_unit_vector_x_f * t_unit_vector_x_f + t_unit_vector_y_f * t_unit_vector_y_f);

  t_unit_vector_x_f = t_unit_vector_x_f / t_length_f;
  t_unit_vector_y_f = t_unit_vector_y_f / t_length_f;

  bc::float32_t t_ratio_f = 2.0 / 3.0;
  its_entry_exit_pts_ar_[point_type_u8].ref_point_cs_.x = t_base_pt_cs.x + t_ratio_f * t_length_f * t_unit_vector_x_f;
  its_entry_exit_pts_ar_[point_type_u8].ref_point_cs_.y = t_base_pt_cs.y + t_ratio_f * t_length_f * t_unit_vector_y_f;
  its_entry_exit_pts_ar_[point_type_u8].valid_b_ = bc::true_v;
}

void IntersectionRecognition::HeadingRangeNormalize(bc::float32_t& heading_f)
{
  if (heading_f >= G_PI_2)
  {
    heading_f = heading_f - G_PI;
  }
  else if (heading_f < -G_PI_2)
  {
    heading_f = heading_f + G_PI;
  }
}

}  // namespace environment_model
}  // namespace zone
