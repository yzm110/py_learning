#include "sop.h"

namespace zone {
namespace environment_model {

StaticObjProcess::StaticObjProcess(EmData& em_collection)
    : em_collection_ptr_(&em_collection),
      timestamp_s64_(0),
      time_cycle_f_(0.0f),
      v_ego_f_(0.0),
      d_after_intersection_f_(0.0),
      traffic_light_enable_b_(bc::true_v),
      is_in_map_b_(bc::false_v)

{

  ResetStaticObj();

  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kCircleMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kLeftArrowMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kRightArrowMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kRightArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kUpArrowMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUpArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kDownArrowMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kDownArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kUturnMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUturnMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kForwardAndLeftMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask)));
  traffic_light_type_map_.insert(std::make_pair(BulbTypeMask::kForwardAndLeftMask,
                                                static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kForwardAndRightMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask)));
  traffic_light_type_map_.insert(std::make_pair(BulbTypeMask::kForwardAndRightMask,
                                                static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kRightArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kPedestrainMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kNonMotorMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCycMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kTimeMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kTimeCountdownMsk)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kLeftAndUturnMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kLeftAndUturnMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUturnMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kNoDriveIntoMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kProhibitMsk)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kTextOfAllowPedMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kSignOfAllowPedMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kTestOfForbidPedMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask)));
  traffic_light_type_map_.insert(std::make_pair(BulbTypeMask::kTestOfForbidPedMask,
                                                static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kProhibitMsk)));
  traffic_light_type_map_.insert(
      std::make_pair(BulbTypeMask::kSignOfForbidPedMask, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask)));
  traffic_light_type_map_.insert(std::make_pair(BulbTypeMask::kSignOfForbidPedMask,
                                                static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kProhibitMsk)));

  bulb_type_mask_ar_ = {BulbTypeMask::kCircleMask,         BulbTypeMask::kLeftArrowMask,
                        BulbTypeMask::kRightArrowMask,     BulbTypeMask::kUpArrowMask,
                        BulbTypeMask::kDownArrowMask,      BulbTypeMask::kUturnMask,
                        BulbTypeMask::kForwardAndLeftMask, BulbTypeMask::kForwardAndRightMask,
                        BulbTypeMask::kPedestrainMask,     BulbTypeMask::kNonMotorMask,
                        BulbTypeMask::kTimeMask,           BulbTypeMask::kLeftAndUturnMask,
                        BulbTypeMask::kNoDriveIntoMask,    BulbTypeMask::kTextOfAllowPedMask,
                        BulbTypeMask::kSignOfAllowPedMask, BulbTypeMask::kTestOfForbidPedMask,
                        BulbTypeMask::kSignOfForbidPedMask};

  traffic_light_dy_ar_[kTrafficLightSemanticStraight] = 0.0;
  traffic_light_dy_ar_[kTrafficLightSemanticLeft] = 3.0;
  traffic_light_dy_ar_[kTrafficLightSemanticRight] = -3.0;
  traffic_light_dy_ar_[kTrafficLightSemanticUturn] = 6.0;
  traffic_light_dy_ar_[kTrafficLightSemanticCyc] = -6.0;
  traffic_light_dy_ar_[kTrafficLightSemanticPed] = -9.0;
}

void StaticObjProcess::StaticObjMatching(const InputDiagInfo& static_obj_diag_cs,
                                         const StaticObjectData& per_static_obj_cs)
{
  /* matching input static objs to EM static objs according to id. (per_static_obj_cs -> static_obj_ar_)
  The static obj with same id should be stored in same index.
  1. should be executed after validity check and before perception line process.
  2. for each input static obj, loop static obj list in last cycle and find the same type & id.
  3. after matching, insert new static objs into absent index.
  */
  ResetStaticObj();

  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  { /** only process when status is not error confirmed. */
    timestamp_s64_ = per_static_obj_cs.exposure_time_stamp_;

    /** 1. create a list, indicating input static objs are matched with last cycle. */
    bc::TCArray<bc::bool_t, kMaxStaticObjNum> t_input_match_list_ar;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      t_input_match_list_ar[t_idx_u8] = bc::false_v;
    }

    /** 2. loop static objs in last cycle, if the last cycle obj and the current input obj has the same type and id,
     * they are matched. */
    for (bc::uint8_t t_idx_lst_u8 = 0; t_idx_lst_u8 < kMaxStaticObjNum; t_idx_lst_u8++)
    {
      /** check if last cycle obj is valid */
      /** NOTICE: here do not consider lanemarking because it will be handled later. */
      /** NOTICE: 0803: do not consider cross/stop line because it will be handled later. */
      /** NOTICE: 0807: do not consider traffic light because it will be handled later. */
      bc::int32_t t_type_lst1_s32 = static_obj_lst1_ar_[t_idx_lst_u8].type_;
      bc::int32_t t_id_lst1_s32 = static_obj_lst1_ar_[t_idx_lst_u8].id_;
      if ((t_type_lst1_s32 != 0) && (t_type_lst1_s32 != 2) && (t_type_lst1_s32 != 4) && (t_type_lst1_s32 != 5) &&
          (t_type_lst1_s32 != 8) && (t_id_lst1_s32 != kInvalidStaticObjId))
      {
        /** if last cycle obj is valid, loop input obj list and try to match an input obj. */
        for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** check if input obj is valid */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_type_new_s32 != 0) && (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** two objs are matched if their types and ids are the same. */
            if ((t_type_lst1_s32 == t_type_new_s32) && (t_id_lst1_s32 == t_id_new_s32))
            {
              /** set a flag in match list, and set input obj into the internal obj array with the old index. */
              t_input_match_list_ar[t_idx_new_u8] = bc::true_v;
              SetInput(per_static_obj_cs.objects_[t_idx_new_u8], t_idx_lst_u8, static_obj_ar_[t_idx_lst_u8]);
            }
          }
        }
      }
    }

    /** 3. after above loop, all matched static objs are stored into the internal obj array with the old index.
     * Below code is to insert remaining new static objs into absent position in internal obj array.
     */
    bc::uint8_t t_idx_start_u8 = 0;
    /** loop internal obj array */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      bc::int32_t t_type_s32 = static_obj_ar_[t_idx_u8].type_;
      bc::int32_t t_id_s32 = static_obj_ar_[t_idx_u8].id_;
      if ((t_type_s32 == 0) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          /** NOTICE: here do not consider lanemarking as above code. */
          /** NOTICE: 0803: here do not consider cross/stop line as above code. */
          /** NOTICE: 0807: here do not consider traffic light as above code. */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_input_match_list_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_s32 != 0) && (t_type_new_s32 != 2) &&
              (t_type_new_s32 != 4) && (t_type_new_s32 != 5) && (t_type_new_s32 != 8) &&
              (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            SetInput(per_static_obj_cs.objects_[t_idx_new_u8], t_idx_u8, static_obj_ar_[t_idx_u8]);
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 > kMaxStaticObjNum)
      {
        break;
      }
    }
  }
  else
  {
    /** input invalid, reset timestamp. static obj array keep default value. */
    timestamp_s64_ = 0;
  }
  static_obj_lst1_ar_ = static_obj_ar_;
}

void StaticObjProcess::ResetStaticObj()
{
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
  {
    static_obj_ar_[t_idx_u8] = EmStaticObject();
    static_obj_lane_assign_ar_[t_idx_u8] = kUnknownLane;
  }

  bc::TPair<bc::int32_t, EmTrafficLightColor> t_reset_pair(kInvalidStaticObjId, EmTrafficLightColor::kUnknown);
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kNumTrafficLightSemantic; t_idx_u8++)
  {
    traffic_light_semantic_type_ar_[t_idx_u8] = t_reset_pair;
  }

  lane_marking_vt_.clear();
  cross_stop_line_vt_.clear();
  traffic_light_vt_.clear();
  cross_stop_line_output_vt_.clear();
  traffic_light_output_vt_.clear();

  is_in_map_b_ = bc::false_v;
}

void StaticObjProcess::SetInput(const StaticObject& obj_in_cs, const bc::uint8_t idx_u8, EmStaticObject& obj_out_cs)
{
  obj_out_cs.id_ = obj_in_cs.id_;
  obj_out_cs.type_ = obj_in_cs.type_;
  obj_out_cs.sub_type_ = obj_in_cs.sub_type_;
  obj_out_cs.conf_ = obj_in_cs.conf_;
  obj_out_cs.left_time_ = obj_in_cs.life_time_;
  obj_out_cs.age_ = obj_in_cs.age_;
  obj_out_cs.attr_.value_ = obj_in_cs.attr_.value_;
  obj_out_cs.attr_.struct_type_ = obj_in_cs.attr_.struct_type_;
  obj_out_cs.attr_.pole_lift_angle_ = obj_in_cs.attr_.pole_lift_angle_;
  obj_out_cs.child_ids_ = obj_in_cs.child_ids_;
  obj_out_cs.position_.x = obj_in_cs.position_.x_;
  obj_out_cs.position_.y = obj_in_cs.position_.y_;
  obj_out_cs.position_.z = obj_in_cs.position_.z_;
  obj_out_cs.child_types_ = obj_in_cs.child_types_;
  obj_out_cs.in_cur_lane_ = obj_in_cs.in_cur_lane_;
  obj_out_cs.drive_start_up_ = obj_in_cs.drive_start_up_;
  obj_out_cs.generation_time_ = obj_in_cs.generation_time_;
  for (bc::uint8_t t_border_pts_index_u8 = 0; t_border_pts_index_u8 < 4; ++t_border_pts_index_u8)
  {
    obj_out_cs.border_.points_[t_border_pts_index_u8].x_ = obj_in_cs.border_.points_[t_border_pts_index_u8].x_;
    obj_out_cs.border_.points_[t_border_pts_index_u8].y_ = obj_in_cs.border_.points_[t_border_pts_index_u8].y_;
    obj_out_cs.border_.points_[t_border_pts_index_u8].z_ = obj_in_cs.border_.points_[t_border_pts_index_u8].z_;
    obj_out_cs.border_.points_[t_border_pts_index_u8].cov_ = obj_in_cs.border_.points_[t_border_pts_index_u8].cov_;
  }
  obj_out_cs.border_.edgeline_width_ = obj_in_cs.border_.edgeline_width_;
  obj_out_cs.border_.normal_.x_ = obj_in_cs.border_.normal_.x_;
  obj_out_cs.border_.normal_.y_ = obj_in_cs.border_.normal_.y_;
  obj_out_cs.border_.normal_.z_ = obj_in_cs.border_.normal_.z_;
  obj_out_cs.border_.normal_.cov_ = obj_in_cs.border_.normal_.cov_;
  obj_out_cs.border_.orientation_.x_ = obj_in_cs.border_.orientation_.x_;
  obj_out_cs.border_.orientation_.y_ = obj_in_cs.border_.orientation_.y_;
  obj_out_cs.border_.orientation_.z_ = obj_in_cs.border_.orientation_.z_;
  obj_out_cs.border_.orientation_.cov_ = obj_in_cs.border_.orientation_.cov_;

  // 计算停止线和斑马线的长/宽/航向角
  if (obj_out_cs.type_ == 5 || obj_out_cs.type_ == 8)
  {
    bc::float64_t t_x1_f64 = 0.0;
    bc::float64_t t_y1_f64 = 0.0;
    bc::float64_t t_x2_f64 = 0.0;
    bc::float64_t t_y2_f64 = 0.0;
    bc::float64_t t_heading1_f64 = 0.0;
    bc::float64_t t_heading2_f64 = 0.0;
    // 当point[0]和point[1]的连线是长边时
    if (Distance(obj_in_cs.border_.points_[0], obj_in_cs.border_.points_[1]) >
        Distance(obj_in_cs.border_.points_[1], obj_in_cs.border_.points_[2]))
    {
      obj_out_cs.length_ = (Distance(obj_in_cs.border_.points_[0], obj_in_cs.border_.points_[1]) +
                            Distance(obj_in_cs.border_.points_[2], obj_in_cs.border_.points_[3])) *
                           0.50;
      obj_out_cs.width_ = (Distance(obj_in_cs.border_.points_[1], obj_in_cs.border_.points_[2]) +
                           Distance(obj_in_cs.border_.points_[0], obj_in_cs.border_.points_[3])) *
                          0.50;
      // 求取值范围在0到pi之间的最长边的法向量
      // point[0]和point[1]连接而成的长边的法向量
      if (obj_out_cs.border_.points_[0].x_ > obj_out_cs.border_.points_[1].x_)
      {
        t_x1_f64 = obj_out_cs.border_.points_[1].y_ - obj_out_cs.border_.points_[0].y_;
        t_y1_f64 = obj_out_cs.border_.points_[0].x_ - obj_out_cs.border_.points_[1].x_;
      }
      else
      {
        t_x1_f64 = obj_out_cs.border_.points_[0].y_ - obj_out_cs.border_.points_[1].y_;
        t_y1_f64 = obj_out_cs.border_.points_[1].x_ - obj_out_cs.border_.points_[0].x_;
      }
      // point[2]和point[3]连接而成的长边的法向量
      if (obj_out_cs.border_.points_[2].x_ > obj_out_cs.border_.points_[3].x_)
      {
        t_x2_f64 = obj_out_cs.border_.points_[3].y_ - obj_out_cs.border_.points_[2].y_;
        t_y2_f64 = obj_out_cs.border_.points_[2].x_ - obj_out_cs.border_.points_[3].x_;
      }
      else
      {
        t_x2_f64 = obj_out_cs.border_.points_[2].y_ - obj_out_cs.border_.points_[3].y_;
        t_y2_f64 = obj_out_cs.border_.points_[3].x_ - obj_out_cs.border_.points_[2].x_;
      }
      t_heading1_f64 = acos(t_x1_f64 / sqrt(t_x1_f64 * t_x1_f64 + t_y1_f64 * t_y1_f64));
      t_heading2_f64 = acos(t_x2_f64 / sqrt(t_x2_f64 * t_x2_f64 + t_y2_f64 * t_y2_f64));
      obj_out_cs.heading_ = (t_heading1_f64 + t_heading2_f64) / 2;
    }
    // 当point[0]和point[3]的连线是长边时
    else
    {
      obj_out_cs.length_ = (Distance(obj_in_cs.border_.points_[1], obj_in_cs.border_.points_[2]) +
                            Distance(obj_in_cs.border_.points_[0], obj_in_cs.border_.points_[3])) *
                           0.50;
      obj_out_cs.width_ = (Distance(obj_in_cs.border_.points_[0], obj_in_cs.border_.points_[1]) +
                           Distance(obj_in_cs.border_.points_[2], obj_in_cs.border_.points_[3])) *
                          0.50;
      // 求取值范围在0到pi之间的最长边的法向量
      // point[0]和point[3]连接而成的长边的法向量
      if (obj_out_cs.border_.points_[0].x_ > obj_out_cs.border_.points_[3].x_)
      {
        t_x1_f64 = obj_out_cs.border_.points_[3].y_ - obj_out_cs.border_.points_[0].y_;
        t_y1_f64 = obj_out_cs.border_.points_[0].x_ - obj_out_cs.border_.points_[3].x_;
      }
      else
      {
        t_x1_f64 = obj_out_cs.border_.points_[0].y_ - obj_out_cs.border_.points_[3].y_;
        t_y1_f64 = obj_out_cs.border_.points_[3].x_ - obj_out_cs.border_.points_[0].x_;
      }
      // point[1]和point[2]连接而成的长边的法向量
      if (obj_out_cs.border_.points_[2].x_ > obj_out_cs.border_.points_[1].x_)
      {
        t_x2_f64 = obj_out_cs.border_.points_[1].y_ - obj_out_cs.border_.points_[2].y_;
        t_y2_f64 = obj_out_cs.border_.points_[2].x_ - obj_out_cs.border_.points_[1].x_;
      }
      else
      {
        t_x2_f64 = obj_out_cs.border_.points_[2].y_ - obj_out_cs.border_.points_[1].y_;
        t_y2_f64 = obj_out_cs.border_.points_[1].x_ - obj_out_cs.border_.points_[2].x_;
      }
      t_heading1_f64 = acos(t_x1_f64 / sqrt(t_x1_f64 * t_x1_f64 + t_y1_f64 * t_y1_f64));
      t_heading2_f64 = acos(t_x2_f64 / sqrt(t_x2_f64 * t_x2_f64 + t_y2_f64 * t_y2_f64));
      obj_out_cs.heading_ = (t_heading1_f64 + t_heading2_f64) / 2;
    }
    if (obj_out_cs.heading_ > G_PI / 2)
    {
      obj_out_cs.heading_ -= G_PI;
    }
  }
  else
  {
    // set as default value
    obj_out_cs.length_ = 0.0;
    obj_out_cs.width_ = 0.0;
    obj_out_cs.heading_ = 0.0;
  }

  // if ((obj_in_cs.type_ == 4) && (obj_in_cs.sub_type_ != 21))
  // {
  //   lane_marking_idx_vt_.push_back(idx_u8);
  // }
}

void StaticObjProcess::StaticObjLaneAssignment()
{
  /* lane assignment for static objects, especially for lane markings.
  1. should be executed after em lane process, can follow olr.
  2. simply check lane assignment.
  */
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    bc::uint8_t t_idx_lane_assign_u8 = kUnknownLane;
    if ((static_obj_ar_[t_idx_obj_u8].id_ != kInvalidStaticObjId) &&
        (static_obj_ar_[t_idx_obj_u8].type_ == 2 || static_obj_ar_[t_idx_obj_u8].type_ == 4))
    {
      for (bc::uint8_t t_idx_lane_u8 = 0; t_idx_lane_u8 < kMaxLaneNum; t_idx_lane_u8++)
      {
        if ((em_collection_ptr_->lanes_[t_idx_lane_u8].lane_valid_ == bc::true_v) &&
            (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].element_valid_ == bc::true_v))
        {
          LaneBoundary& t_left_bound_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].left_boundary_;
          LaneBoundary& t_right_bound_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].right_boundary_;
          ReferenceLine& t_ref_line_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].reference_line_;
          if ((t_left_bound_cs.existence_ == bc::true_v) && (t_right_bound_cs.existence_ == bc::true_v) &&
              (t_ref_line_cs.available_ == bc::true_v))
          {
            bc::float32_t t_dx_f = static_obj_ar_[t_idx_obj_u8].position_.x;
            bc::float32_t t_dy_f = static_obj_ar_[t_idx_obj_u8].position_.y;
            bc::uint16_t t_idx_u16 = 0;
            bc::bool_t t_find_idx_b = FindNearestRefIdx(t_ref_line_cs, t_dx_f, t_idx_u16);
            if (t_find_idx_b == bc::true_v)
            {
              if ((t_dy_f > t_right_bound_cs.pts_[t_idx_u16].y) && (t_dy_f < t_left_bound_cs.pts_[t_idx_u16].y))
              {
                t_idx_lane_assign_u8 = t_idx_lane_u8;
                break;
              }
            }
            else
            {
            }
          }
        }
      }
    }
    static_obj_lane_assign_ar_[t_idx_obj_u8] = t_idx_lane_assign_u8;
  }
}

void StaticObjProcess::Run(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                           const StaticObjectData& per_static_obj_cs, const SemanticPack& semantic_pack_cs)
{
  is_in_map_b_ = semantic_pack_cs.map_geofence_.is_in_map_;
  v_ego_f_ = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetLongVelocity();
  LaneMarkingStablization(static_obj_diag_cs, ego_motion_cs, per_static_obj_cs, semantic_pack_cs.map_lane_marking_ar_);
  CrossStopLineStablization(static_obj_diag_cs, ego_motion_cs, per_static_obj_cs,
                            semantic_pack_cs.map_cross_stop_line_ar_);
  TrafficLightStablization(static_obj_diag_cs, ego_motion_cs, per_static_obj_cs,
                           semantic_pack_cs.map_traffic_light_ar_);
  ConeClusterAndConvexHullGeneration(static_obj_diag_cs, ego_motion_cs, per_static_obj_cs);
}

void StaticObjProcess::ConeClusterAndConvexHullGeneration(const InputDiagInfo& static_obj_diag_cs,
                                                          const EgoPoseCollection& ego_motion_cs,
                                                          const StaticObjectData& per_static_obj_cs)
{
  TrackingConeArrayAndCluster(ego_motion_cs);
  FilterCone(static_obj_diag_cs, per_static_obj_cs);
  UpdateConeMarkingArray(static_obj_diag_cs, per_static_obj_cs);
  ConeClustering();
  UpdateCluster(ego_motion_cs);
  ConeOutput();
}

void StaticObjProcess::FilterCone(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs)
{
  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  {
    cone_fliter_cs_ = per_static_obj_cs;
    LaneBoundary t_left_laneboundary = em_collection_ptr_->lanes_[2].lane_elements_[0].left_boundary_;
    LaneBoundary t_right_laneboundary = em_collection_ptr_->lanes_[2].lane_elements_[0].right_boundary_;
    bc::bool_t t_left_valid =
        t_left_laneboundary.existence_ &&
        (GetBitU8(t_left_laneboundary.source_type_, LaneBoundary::kSrcMaskPerception) == bc::true_v ||
         GetBitU8(t_left_laneboundary.source_type_, LaneBoundary::kSrcMaskMap) == bc::true_v);
    bc::bool_t t_right_valid =
        t_right_laneboundary.existence_ &&
        (GetBitU8(t_right_laneboundary.source_type_, LaneBoundary::kSrcMaskPerception) == bc::true_v ||
         GetBitU8(t_right_laneboundary.source_type_, LaneBoundary::kSrcMaskMap) == bc::true_v);
    CurvParam<bc::float32_t, kMaxBoundaryPoint> t_left_boundary_cur;
    CurvParam<bc::float32_t, kMaxBoundaryPoint> t_right_boundary_cur;
    bc::float32_t t_cone_y_f = 0.f;
    if (t_left_valid)
    {
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
      {
        t_left_boundary_cur.x_arr[t_idx_u8] = t_left_laneboundary.pts_[t_idx_u8].x;
        t_left_boundary_cur.y_arr[t_idx_u8] = t_left_laneboundary.pts_[t_idx_u8].y;
      }
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; ++t_idx_u8)
      {
        if (cone_fliter_cs_.objects_[t_idx_u8].type_ == kConeType &&
            cone_fliter_cs_.objects_[t_idx_u8].sub_type_ != kConeInvalidSubtype &&
            cone_fliter_cs_.objects_[t_idx_u8].position_.y_ > 0)
        {
          t_cone_y_f = InterpCurv(t_left_boundary_cur, bc::float32_t(cone_fliter_cs_.objects_[t_idx_u8].position_.x_));
          if (t_cone_y_f + kConeFilterDis < cone_fliter_cs_.objects_[t_idx_u8].position_.y_)
          {
            cone_fliter_cs_.objects_[t_idx_u8] = StaticObject();
          }
        }
      }
    }

    if (t_right_valid)
    {
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
      {
        t_right_boundary_cur.x_arr[t_idx_u8] = t_right_laneboundary.pts_[t_idx_u8].x;
        t_right_boundary_cur.y_arr[t_idx_u8] = t_right_laneboundary.pts_[t_idx_u8].y;
      }
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; ++t_idx_u8)
      {
        if (cone_fliter_cs_.objects_[t_idx_u8].type_ == kConeType &&
            cone_fliter_cs_.objects_[t_idx_u8].sub_type_ != kConeInvalidSubtype &&
            cone_fliter_cs_.objects_[t_idx_u8].position_.y_ < 0)
        {
          t_cone_y_f = InterpCurv(t_right_boundary_cur, bc::float32_t(cone_fliter_cs_.objects_[t_idx_u8].position_.x_));
          if (t_cone_y_f - kConeFilterDis > cone_fliter_cs_.objects_[t_idx_u8].position_.y_)
          {
            cone_fliter_cs_.objects_[t_idx_u8] = StaticObject();
          }
        }
      }
    }
  }
}

void StaticObjProcess::TrackingConeArrayAndCluster(const EgoPoseCollection& ego_motion_cs)
{
  // 把上个cycle中还有效的cone坐标变换到当前自车坐标系作为预测量，和当前cycle同一个锥桶的量测进行滤波
  TransForm t_transform_st = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::float32_t t_dx_trans_f = t_transform_st.translation_dx_f_;
  bc::float32_t t_dy_trans_f = t_transform_st.translation_dy_f_;
  bc::float32_t t_dx_f = 0.f;
  bc::float32_t t_dy_f = 0.f;
  bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f),
                                                                   -sinf(t_rotation_f), cosf(t_rotation_f)};
  for (bc::uint8_t t_obj_idx_u8 = 0; t_obj_idx_u8 < kMaxStaticObjNum; t_obj_idx_u8++)
  {
    if (cone_ar_[t_obj_idx_u8].id_s32_ != kInvalidStaticObjId && cone_ar_[t_obj_idx_u8].type_u16_ != kConeInvalidtype)
    {
      cone_ar_[t_obj_idx_u8].dx_f_ = cone_ar_[t_obj_idx_u8].dx_f_ - t_dx_trans_f;
      cone_ar_[t_obj_idx_u8].dy_f_ = cone_ar_[t_obj_idx_u8].dy_f_ - t_dy_trans_f;

      t_dx_f =
          t_trans_matrix_ar[0] * cone_ar_[t_obj_idx_u8].dx_f_ + t_trans_matrix_ar[1] * cone_ar_[t_obj_idx_u8].dy_f_;
      t_dy_f =
          t_trans_matrix_ar[2] * cone_ar_[t_obj_idx_u8].dx_f_ + t_trans_matrix_ar[3] * cone_ar_[t_obj_idx_u8].dy_f_;

      cone_ar_[t_obj_idx_u8].dx_f_ = t_dx_f;
      cone_ar_[t_obj_idx_u8].dy_f_ = t_dy_f;
      cone_ar_[t_obj_idx_u8].updated_b_ = bc::false_v;
    }
  }
  // 清空cluster
  cone_cluster_vt_.clear();
}

void StaticObjProcess::UpdateConeMarkingArray(const InputDiagInfo& static_obj_diag_cs,
                                              const StaticObjectData& per_static_obj_cs)
{
  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  {

    // 记录新锥桶和上个cycle的锥桶匹配上的
    bc::TCArray<bc::bool_t, kMaxStaticObjNum> t_input_match_list_ar;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; ++t_idx_u8)
    {
      t_input_match_list_ar[t_idx_u8] = bc::false_v;
    }

    // 为新锥桶和上个cycle锥桶匹配上的更新属性值
    bc::int32_t t_type_lst1_s32 = 0;
    bc::int32_t t_id_lst1_s32 = 0;
    bc::int32_t t_new_type_s32 = 0;
    bc::int32_t t_new_sub_type_s32 = 0;
    bc::int32_t t_new_id_s32 = 0;
    bc::bool_t t_new_cone_b = bc::false_v;
    bc::uint8_t t_matched_num_u8 = 0;
    for (bc::uint8_t t_idx_lst_u8 = 0; t_idx_lst_u8 < kMaxStaticObjNum; t_idx_lst_u8++)
    {
      t_type_lst1_s32 = cone_ar_[t_idx_lst_u8].type_u16_;
      t_id_lst1_s32 = cone_ar_[t_idx_lst_u8].id_s32_;
      if (t_type_lst1_s32 != kConeInvalidtype && t_id_lst1_s32 != kInvalidStaticObjId)
      {
        for (bc::uint8_t t_new_idx_u8 = 0; t_new_idx_u8 < kMaxStaticObjNum; t_new_idx_u8++)
        {
          t_new_type_s32 = cone_fliter_cs_.objects_[t_new_idx_u8].type_;
          t_new_sub_type_s32 = cone_fliter_cs_.objects_[t_new_idx_u8].sub_type_;
          t_new_id_s32 = cone_fliter_cs_.objects_[t_new_idx_u8].id_;
          if (t_new_type_s32 == kConeType && t_new_sub_type_s32 != kConeInvalidSubtype &&
              t_new_id_s32 != kInvalidStaticObjId)
          {
            if (t_id_lst1_s32 == t_new_id_s32)
            {
              t_input_match_list_ar[t_new_idx_u8] = bc::true_v;
              t_new_cone_b = bc::false_v;
              SetConeInput(cone_fliter_cs_.objects_[t_new_idx_u8], t_new_cone_b, cone_ar_[t_idx_lst_u8]);
              cone_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
              t_matched_num_u8 += 1;
            }
          }
        }
      }
    }
    // 将上个cycle中的一些不符合条件的cone删掉，此时空出来的index就会这个cycle可以新添加的
    for (bc::uint8_t t_idx_lst1_u8 = 0; t_idx_lst1_u8 < kMaxStaticObjNum; t_idx_lst1_u8++)
    {
      if (((cone_ar_[t_idx_lst1_u8].type_u16_ != kConeInvalidtype) &&
           (cone_ar_[t_idx_lst1_u8].id_s32_ != kInvalidStaticObjId) &&
           (cone_ar_[t_idx_lst1_u8].updated_b_ == bc::false_v) &&
           (cone_ar_[t_idx_lst1_u8].life_time_f_ < time_cycle_f_)) ||
          (cone_ar_[t_idx_lst1_u8].dx_f_ < kDropoutDistance))
      {
        cone_ar_[t_idx_lst1_u8] = ConeInfo();
      }
    }

    // 往cone_ar中还空的index存入cone,但是会有一个问题，会存在新更新的还没有放进去(判断有没有放完)，所以考虑这一步做完对lifetime进行排序，删掉尾部没有update的
    bc::uint8_t t_idx_start_u8 = 0;
    bc::uint8_t t_new_add_num = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      t_type_lst1_s32 = cone_ar_[t_idx_u8].type_u16_;
      t_id_lst1_s32 = cone_ar_[t_idx_u8].id_s32_;
      if (t_type_lst1_s32 == kConeInvalidtype || t_id_lst1_s32 == kInvalidStaticObjId)
      {
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          t_new_sub_type_s32 = cone_fliter_cs_.objects_[t_idx_new_u8].sub_type_;
          t_new_type_s32 = cone_fliter_cs_.objects_[t_idx_new_u8].type_;
          t_new_id_s32 = cone_fliter_cs_.objects_[t_idx_new_u8].id_;
          if (t_input_match_list_ar[t_idx_new_u8] == bc::false_v && t_new_type_s32 == kConeType &&
              t_new_sub_type_s32 != kConeInvalidSubtype && t_new_id_s32 != kInvalidStaticObjId)
          {
            t_new_cone_b = bc::true_v;
            SetConeInput(cone_fliter_cs_.objects_[t_idx_new_u8], t_new_cone_b, cone_ar_[t_idx_u8]);
            cone_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            t_new_add_num += 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 > kMaxStaticObjNum)
      {
        break;
      }
    }

    // 对于锥桶，先进行远近排序，存入。先判断cone_ar中还有没有空位置和新更新的cone还有没有存完，对cone_ar用lifetime进行排序删掉尾部的没有update的

    // 对lifetime进行更新
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (cone_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId && cone_ar_[t_idx_u8].type_u16_ != kConeInvalidtype)
      {
        if (cone_ar_[t_idx_u8].updated_b_ == bc::true_v)
        {
          cone_ar_[t_idx_u8].life_time_f_ += time_cycle_f_;
        }
        else
        {
          cone_ar_[t_idx_u8].life_time_f_ = std::min(cone_ar_[t_idx_u8].life_time_f_, bc::float32_t(0.5));
          cone_ar_[t_idx_u8].life_time_f_ -= time_cycle_f_;
        }
      }
    }
  }
}

void StaticObjProcess::SetConeInput(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b, ConeInfo& obj_out_cs)
{
  obj_out_cs.id_s32_ = obj_in_cs.id_;
  obj_out_cs.updated_b_ = bc::true_v;

  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y_;
    obj_out_cs.life_time_f_ = 0.3;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y_;
  }

  obj_out_cs.type_u16_ = obj_in_cs.type_;
}

void StaticObjProcess::ConeClustering()
{
  bc::TCArray<bc::uint8_t, kMaxStaticObjNum> t_is_clustered_ar;
  for (bc::uint8_t& data : t_is_clustered_ar)
  {
    data = kMaxClusterNumCone;
  }
  std::list<bc::uint8_t> t_cluster_list;
  bc::TCArray<bc::TCArray<bc::float32_t, kMaxStaticObjNum>, kMaxStaticObjNum> t_dis_ar;
  bc::float32_t t_dis_f = 0;
  bc::uint8_t t_start_idx_u8 = 0;
  bc::uint8_t t_cluster_index_u8 = 0;
  bc::uint8_t t_idx_not_cluster_u8 = kMaxStaticObjNum;
  for (bc::uint8_t t_row_idx_u8 = 0; t_row_idx_u8 < kMaxStaticObjNum; t_row_idx_u8++)
  {
    for (bc::uint8_t t_col_idx_u8 = 0; t_col_idx_u8 < kMaxStaticObjNum; t_col_idx_u8++)
    {
      if (cone_ar_[t_row_idx_u8].type_u16_ != kConeInvalidtype &&
          cone_ar_[t_col_idx_u8].type_u16_ != kConeInvalidtype &&
          cone_ar_[t_row_idx_u8].id_s32_ != kInvalidStaticObjId &&
          cone_ar_[t_col_idx_u8].id_s32_ != kInvalidStaticObjId)
      {
        t_dis_f = std::sqrt((cone_ar_[t_row_idx_u8].dx_f_ - cone_ar_[t_col_idx_u8].dx_f_) *
                                (cone_ar_[t_row_idx_u8].dx_f_ - cone_ar_[t_col_idx_u8].dx_f_) +
                            (cone_ar_[t_row_idx_u8].dy_f_ - cone_ar_[t_col_idx_u8].dy_f_) *
                                (cone_ar_[t_row_idx_u8].dy_f_ - cone_ar_[t_col_idx_u8].dy_f_));
        t_dis_ar[t_row_idx_u8][t_col_idx_u8] = t_dis_f;
        t_dis_ar[t_col_idx_u8][t_row_idx_u8] = t_dis_f;
      }
    }
  }

  auto iter = std::find_if(cone_ar_.begin(), cone_ar_.end(), [](const ConeInfo a) {
    return a.type_u16_ != kConeInvalidtype && a.id_s32_ != kInvalidStaticObjId;
  });
  if (iter == cone_ar_.end())
  {
    return;
  }
  else
  {
    t_start_idx_u8 = std::distance(cone_ar_.begin(), iter);
  }
  t_cluster_list.push_back(t_start_idx_u8);
  ConeCluster t_first_cluster;
  t_first_cluster.valid_b_ = bc::true_v;
  cone_cluster_vt_.push_back(std::move(t_first_cluster));
  cone_cluster_vt_[0].cone_pack_vt_.push_back(cone_ar_[t_start_idx_u8]);
  while (t_cluster_list.size() != 0)
  {
    t_start_idx_u8 = t_cluster_list.front();
    if (t_cluster_list.size() != 0)
    {
      t_cluster_list.pop_front();
    }
    t_is_clustered_ar[t_start_idx_u8] = t_cluster_index_u8;

    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (t_idx_u8 != t_start_idx_u8 && cone_ar_[t_idx_u8].type_u16_ != kConeInvalidtype &&
          cone_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId && t_is_clustered_ar[t_idx_u8] == kMaxClusterNumCone)
      {
        if (t_dis_ar[t_start_idx_u8][t_idx_u8] < kConeToClusterMaxDis)
        {
          t_is_clustered_ar[t_idx_u8] = t_cluster_index_u8;
          cone_ar_[t_idx_u8].assigned_cluster_idx_ = t_cluster_index_u8;
          if (cone_cluster_vt_[t_cluster_index_u8].cone_pack_vt_.size() < kMaxStaticObjNum)
          {
            cone_cluster_vt_[t_cluster_index_u8].cone_pack_vt_.push_back(cone_ar_[t_idx_u8]);
            t_cluster_list.push_back(t_idx_u8);
          }
        }
      }
    }
    t_idx_not_cluster_u8 = kMaxStaticObjNum;
    if (t_cluster_list.size() == 0)
    {

      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
      {
        if (t_is_clustered_ar[t_idx_u8] == kMaxClusterNumCone && cone_ar_[t_idx_u8].type_u16_ != kConeInvalidtype &&
            cone_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId)
        {
          t_idx_not_cluster_u8 = t_idx_u8;
          break;
        }
      }
      if (t_idx_not_cluster_u8 != kMaxStaticObjNum)
      {
        t_cluster_list.push_back(t_idx_not_cluster_u8);
        t_cluster_index_u8++;
        if (cone_cluster_vt_.size() == kMaxClusterNumCone)
        {
          break;
        }
        ConeCluster t_cluster_temp;
        t_cluster_temp.valid_b_ = bc::true_v;
        cone_cluster_vt_.push_back(std::move(t_cluster_temp));
        cone_cluster_vt_[t_cluster_index_u8].cone_pack_vt_.push_back(cone_ar_[t_idx_not_cluster_u8]);
      }
    }
  }
}

void StaticObjProcess::UpdateCluster(const EgoPoseCollection& ego_motion_cs)
{
  EgoPose t_current_pose = ego_motion_cs.GetCurrentEgoPoseReadOnly();
  for (auto& cluster_info : cone_cluster_vt_)
  {
    GenerateConvexHullForCluster(cluster_info, t_current_pose);
  }
}

void StaticObjProcess::GenerateConvexHullForCluster(ConeCluster& cluster_info, const EgoPose& ego_pose)
{
  bc::uint8_t t_method_flag = 2;
  ConvexHull convex_hull(t_method_flag);
  ConvexHullInfo convex_hull_info;
  bc::TFixedVector<Point3D, kMaxStaticObjNum> t_pts_ar;
  t_pts_ar.clear();
  Point3D t_point;
  bc::bool_t t_solved_b = bc::false_v;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < cluster_info.cone_pack_vt_.size(); t_idx_u8++)
  {
    t_point.x = cluster_info.cone_pack_vt_[t_idx_u8].dx_f_;
    t_point.y = cluster_info.cone_pack_vt_[t_idx_u8].dy_f_;
    t_pts_ar.push_back(t_point);
  }
  t_solved_b = convex_hull.Solve(t_pts_ar, convex_hull_info, ego_pose);
  if (t_solved_b && convex_hull_info.valid_b_)
  {
    cluster_info.dx_f_ = convex_hull_info.dx_f_;
    cluster_info.dy_f_ = convex_hull_info.dy_f_;
    cluster_info.heading_f_ = convex_hull_info.heading_f;
    cluster_info.length_f_ = convex_hull_info.length_f;
    cluster_info.width_f_ = convex_hull_info.width_f;
  }
}

void StaticObjProcess::ConeOutput()
{
  EmConeCollection& t_cone_collection_cs = em_collection_ptr_->static_object_.cone_collection_;
  EmCone t_em_cone;
  EmConeCluster t_em_cone_cluster;
  bc::uint8_t t_cone_num = 0;
  bc::uint8_t t_cone_cluster_num = 0;
  for (bc::uint8_t t_cone_idx_u8 = 0; t_cone_idx_u8 < kMaxStaticObjNum; ++t_cone_idx_u8)
  {
    if (cone_ar_[t_cone_idx_u8].id_s32_ != kInvalidStaticObjId && cone_ar_[t_cone_idx_u8].type_u16_ != kConeInvalidtype)
    {
      t_cone_num += 1;
      t_em_cone.position_.x = cone_ar_[t_cone_idx_u8].dx_f_;
      t_em_cone.position_.y = cone_ar_[t_cone_idx_u8].dy_f_;
      t_em_cone.valid_ = bc::true_v;
      t_em_cone.idx_cluster_assigned_ = cone_ar_[t_cone_idx_u8].assigned_cluster_idx_;
      t_cone_collection_cs.cone_ar_[t_cone_idx_u8] = t_em_cone;
    }
  }

  for (bc::uint8_t t_cluster_idx_u8 = 0; t_cluster_idx_u8 < cone_cluster_vt_.size(); t_cluster_idx_u8++)
  {
    if (cone_cluster_vt_[t_cluster_idx_u8].valid_b_)
    {
      t_cone_cluster_num += 1;
      t_em_cone_cluster.heading_ = cone_cluster_vt_[t_cluster_idx_u8].heading_f_;
      t_em_cone_cluster.length_ = cone_cluster_vt_[t_cluster_idx_u8].length_f_;
      t_em_cone_cluster.width_ = cone_cluster_vt_[t_cluster_idx_u8].width_f_;
      t_em_cone_cluster.valid_ = cone_cluster_vt_[t_cluster_idx_u8].valid_b_;
      t_em_cone_cluster.position_.x = cone_cluster_vt_[t_cluster_idx_u8].dx_f_;
      t_em_cone_cluster.position_.y = cone_cluster_vt_[t_cluster_idx_u8].dy_f_;
      t_cone_collection_cs.cone_cluster_ar_[t_cluster_idx_u8] = t_em_cone_cluster;
    }
  }
  t_cone_collection_cs.num_cone_ = t_cone_num;
  t_cone_collection_cs.num_cone_cluster_ = t_cone_cluster_num;
}

void StaticObjProcess::LaneMarkingStablization(
    const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
    const StaticObjectData& per_static_obj_cs,
    const bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum>& map_lane_marking_ar)
{
  /* lane markings are always vibrating and should be stablized.
  1. check lane assignment and replace y position by the center of assigned lane.
  2. check lane markings across all lanes, cluster them and calculate a reasonable x position for all lane markings
  clustered.
  3. each following cycle, track x position instead of using input x.
  */

  /** if successfully lane assigned, find nearest reference points and calculate y by interpolation.
   * currently only for lane marking.
   */
  TrackingLaneMarkingArrayAndCluster(ego_motion_cs);
  UpdateLaneMarkingArray(static_obj_diag_cs, per_static_obj_cs, map_lane_marking_ar);
  LaneMarkingLaneAssignment();
  LaneMarkingArray2Vector();
  LaneMarkingClustering();
  UpdateLaneMarkingClusterList();
  LaneMarkingDistillation();
  LaneMarkingOutput();
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((static_obj_ar_[t_idx_obj_u8].id_ != kInvalidStaticObjId) && (static_obj_ar_[t_idx_obj_u8].type_ == 4) &&
        (static_obj_lane_assign_ar_[t_idx_obj_u8]) != kUnknownLane)
    {
      bc::float32_t t_dx_f = static_obj_ar_[t_idx_obj_u8].position_.x;
      bc::uint8_t t_idx_lane_u8 = static_obj_lane_assign_ar_[t_idx_obj_u8];
      ReferenceLine& t_ref_line_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].reference_line_;

      /** if static obj is out of the range of reference line, use y of the bounding point.
       * not a good strategy, try in the first version.
       */
      if (t_dx_f < t_ref_line_cs.ref_line_pts_[0].pos_.x)
      {
        static_obj_ar_[t_idx_obj_u8].position_.y = t_ref_line_cs.ref_line_pts_[0].pos_.y;
      }
      else if (t_dx_f > t_ref_line_cs.ref_line_pts_[kMaxRefLinePtsNum - 1].pos_.x)
      {
        static_obj_ar_[t_idx_obj_u8].position_.y = t_ref_line_cs.ref_line_pts_[kMaxRefLinePtsNum - 1].pos_.y;
      }
      else
      {
        /** interpolation if the static obj locates in the range of reference line. */
        for (bc::uint16_t t_idx_pt_u16 = 1; t_idx_pt_u16 < kMaxRefLinePtsNum; t_idx_pt_u16++)
        {
          if ((t_dx_f > t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.x) &&
              (t_dx_f < t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.x))
          {
            static_obj_ar_[t_idx_obj_u8].position_.y = LinearInterpolation(
                t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.x,
                t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.y, t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.x,
                t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.y, t_dx_f, Lin_Interp_Method::kFlat);
            break;
          }
        }
      }
    }
  }
}

void StaticObjProcess::TrackingLaneMarkingArrayAndCluster(const EgoPoseCollection& ego_motion_cs)
{
  TransForm t_transform_st = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::float32_t t_dx_trans_f = t_transform_st.translation_dx_f_;
  bc::float32_t t_dy_trans_f = t_transform_st.translation_dy_f_;
  bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f),
                                                                   -sinf(t_rotation_f), cosf(t_rotation_f)};
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((lane_marking_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (lane_marking_ar_[t_idx_obj_u8].type_u16_ != 0))
    {
      /** TODO: update dx,dy by ego motion*/

      lane_marking_ar_[t_idx_obj_u8].dx_f_ = lane_marking_ar_[t_idx_obj_u8].dx_f_ - t_dx_trans_f;
      lane_marking_ar_[t_idx_obj_u8].dy_f_ = lane_marking_ar_[t_idx_obj_u8].dy_f_ - t_dy_trans_f;

      bc::float32_t t_dx_f = t_trans_matrix_ar[0] * lane_marking_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[1] * lane_marking_ar_[t_idx_obj_u8].dy_f_;
      bc::float32_t t_dy_f = t_trans_matrix_ar[2] * lane_marking_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[3] * lane_marking_ar_[t_idx_obj_u8].dy_f_;

      lane_marking_ar_[t_idx_obj_u8].dx_f_ = t_dx_f;
      lane_marking_ar_[t_idx_obj_u8].dy_f_ = t_dy_f;
      lane_marking_ar_[t_idx_obj_u8].updated_b_ = bc::false_v;
    }
  }

  for (bc::uint8_t t_idx_cluster_u8 = 0; t_idx_cluster_u8 < lane_marking_cluster_vt_.size(); t_idx_cluster_u8++)
  {
    lane_marking_cluster_vt_[t_idx_cluster_u8].lane_marking_subcluster_vt_.clear();
    lane_marking_cluster_vt_[t_idx_cluster_u8].num_lane_marking_ = 0;
    lane_marking_cluster_vt_[t_idx_cluster_u8].new_create_b_ = bc::false_v;
    lane_marking_cluster_vt_[t_idx_cluster_u8].dx_mean_f_ = 0.0;

    /** TODO: update dx,dy by ego motion*/
    lane_marking_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ =
        lane_marking_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ -
        sqrt(t_dx_trans_f * t_dx_trans_f + t_dy_trans_f * t_dy_trans_f);
  }
}

void StaticObjProcess::UpdateLaneMarkingArray(const InputDiagInfo& static_obj_diag_cs,
                                              const StaticObjectData& per_static_obj_cs,
                                              const bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum>& map_lane_marking_ar)
{

  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  { /** only process when status is not error confirmed. */

    /** 1. create a list, indicating input static objs are matched with last cycle. */
    bc::TCArray<bc::bool_t, kMaxStaticObjNum> t_input_match_per_ar;
    bc::TCArray<bc::bool_t, kMaxLaneMarkingNum> t_input_match_map_ar;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      t_input_match_per_ar[t_idx_u8] = bc::false_v;
    }
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLaneMarkingNum; t_idx_u8++)
    {
      t_input_match_map_ar[t_idx_u8] = bc::false_v;
    }

    /** 2. loop lane_marking_ar_ in last cycle, the last cycle lane marking and the current input obj are matched if
     * a) same type
     * b) same subtype
     * c) same id
     */
    /** NOTICE: lane_marking_ar_ now stores info from last cycle, so temp variables below are named by "lst1" */
    for (bc::uint8_t t_idx_lst_u8 = 0; t_idx_lst_u8 < kMaxStaticObjNum; t_idx_lst_u8++)
    {
      /** check if last cycle lane marking is valid */
      bc::int32_t t_type_lst1_u16 = lane_marking_ar_[t_idx_lst_u8].type_u16_;
      bc::int32_t t_id_lst1_s32 = lane_marking_ar_[t_idx_lst_u8].id_s32_;
      StaticObjSource t_source_lst1_en = lane_marking_ar_[t_idx_lst_u8].source_en_;
      if ((t_type_lst1_u16 != 0) && (t_id_lst1_s32 != kInvalidStaticObjId))
      {
        if (t_source_lst1_en == StaticObjSource::kPerception)
        {
          /** if last cycle lane marking is valid, loop input obj list and try to match an input obj. */
          for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
          {
            /** check if input obj is valid */
            bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
            bc::int32_t t_sub_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].sub_type_;
            bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
            if ((t_type_new_s32 == 4) && (t_sub_type_new_s32 != 0) && (t_id_new_s32 != kInvalidStaticObjId))
            {
              /** two objs are matched if their types and ids are the same. */
              if ((t_id_lst1_s32 == t_id_new_s32))
              {
                /** set a flag in match list, and set input obj into the internal obj array with the old index. */
                t_input_match_per_ar[t_idx_new_u8] = bc::true_v;
                bc::bool_t t_new_lane_marking_b = bc::false_v;
                SetLaneMarkingInputPer(per_static_obj_cs.objects_[t_idx_new_u8], t_new_lane_marking_b,
                                       lane_marking_ar_[t_idx_lst_u8]);
                lane_marking_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
                break;
              }
            }
          }
        }
        else if (t_source_lst1_en == StaticObjSource::kMap)
        {
          for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxLaneMarkingNum; t_idx_new_u8++)
          {
            /** check if input obj is valid */
            bc::uint16_t t_type_new_u16 = map_lane_marking_ar[t_idx_new_u8].type_;
            bc::int32_t t_id_new_s32 = map_lane_marking_ar[t_idx_new_u8].id_;
            bc::bool_t t_valid_b = map_lane_marking_ar[t_idx_new_u8].valid_;
            if ((t_type_new_u16 != 0) && (t_id_new_s32 != kInvalidStaticObjId) && (t_valid_b == bc::true_v))
            {
              /** two objs are matched if their types and ids are the same. */
              if ((t_id_lst1_s32 == t_id_new_s32))
              {
                /** set a flag in match list, and set input obj into the internal obj array with the old index. */
                t_input_match_map_ar[t_idx_new_u8] = bc::true_v;
                bc::bool_t t_new_lane_marking_b = bc::false_v;
                SetLaneMarkingInputMap(map_lane_marking_ar[t_idx_new_u8], t_new_lane_marking_b,
                                       lane_marking_ar_[t_idx_lst_u8]);
                lane_marking_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
                break;
              }
            }
          }
        }
        else
        {
          /** do nothing*/
        }
      }
      else
      {
        /** do nothing*/
      }
    }

    /** 3. delete lane markings in lane_marking_ar_ if its update is false and life time is exhausted.*/
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (((lane_marking_ar_[t_idx_u8].type_u16_ != 0) && (lane_marking_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId) &&
           (lane_marking_ar_[t_idx_u8].updated_b_ == bc::false_v) &&
           (lane_marking_ar_[t_idx_u8].life_time_f_ < time_cycle_f_)) ||
          (lane_marking_ar_[t_idx_u8].dx_f_ < -20.0))
      {
        lane_marking_ar_[t_idx_u8] = LaneMarkingInfo();
      }
    }

    /** 4. after above loop, all matched lane markings are stored into the lane_marking_ar_ with the old index.
     * Below code is to insert remaining new lane markings into absent position in lane_marking_ar_.
     */
    bc::uint8_t t_idx_start_u8 = 0;
    /** loop internal obj array */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      bc::uint16_t t_type_u16 = lane_marking_ar_[t_idx_u8].type_u16_;
      bc::int32_t t_id_s32 = lane_marking_ar_[t_idx_u8].id_s32_;
      if ((t_type_u16 == 0) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_sub_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].sub_type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_input_match_per_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_s32 == 4) &&
              (t_sub_type_new_s32 != 0) && (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            bc::bool_t t_new_lane_marking_b = bc::true_v;
            SetLaneMarkingInputPer(per_static_obj_cs.objects_[t_idx_new_u8], t_new_lane_marking_b,
                                   lane_marking_ar_[t_idx_u8]);
            lane_marking_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 >= kMaxStaticObjNum)
      {
        break;
      }
    }

    t_idx_start_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      bc::uint16_t t_type_u16 = lane_marking_ar_[t_idx_u8].type_u16_;
      bc::int32_t t_id_s32 = lane_marking_ar_[t_idx_u8].id_s32_;
      if ((t_type_u16 == 0) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxLaneMarkingNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          bc::uint16_t t_type_new_u16 = map_lane_marking_ar[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = map_lane_marking_ar[t_idx_new_u8].id_;
          bc::float32_t t_dx_new_f = map_lane_marking_ar[t_idx_new_u8].position_.x;
          bc::bool_t t_valid_b = map_lane_marking_ar[t_idx_new_u8].valid_;
          if ((t_input_match_map_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_u16 != 0) &&
              (t_id_new_s32 != kInvalidStaticObjId) && (t_dx_new_f < 100.0) && (t_valid_b == bc::true_v))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            bc::bool_t t_new_lane_marking_b = bc::true_v;
            SetLaneMarkingInputMap(map_lane_marking_ar[t_idx_new_u8], t_new_lane_marking_b, lane_marking_ar_[t_idx_u8]);
            lane_marking_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 >= kMaxLaneMarkingNum)
      {
        break;
      }
    }

    /** 5. update life time according to update_b flag. */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if ((lane_marking_ar_[t_idx_u8].type_u16_ != 0) && (lane_marking_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId))
      {
        if (lane_marking_ar_[t_idx_u8].updated_b_ == bc::true_v)
        {
          lane_marking_ar_[t_idx_u8].life_time_f_ = lane_marking_ar_[t_idx_u8].life_time_f_ + time_cycle_f_;
        }
        else
        {
          lane_marking_ar_[t_idx_u8].life_time_f_ = lane_marking_ar_[t_idx_u8].life_time_f_ - time_cycle_f_;
        }
      }
    }
  }
  else
  { /** input invalid, reset timestamp. static obj array keep default value. */
  }
}

void StaticObjProcess::SetLaneMarkingInputPer(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b,
                                              LaneMarkingInfo& obj_out_cs)
{
  obj_out_cs.id_s32_ = obj_in_cs.id_;
  obj_out_cs.updated_b_ = bc::true_v;

  /** currently life_time is still from last cycle. */
  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y_;
    obj_out_cs.life_time_f_ = 0.3;
    obj_out_cs.source_en_ = StaticObjSource::kPerception;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y_;
  }

  obj_out_cs.type_u16_ = LaneMarkingTypeMapping(obj_in_cs.sub_type_);
  obj_out_cs.lane_assigned_u8_ = kUnknownLane;
  bc::float64_t t_01_f64 = (obj_in_cs.border_.points_[0].x_ + obj_in_cs.border_.points_[1].x_) * 0.50;
  bc::float64_t t_23_f64 = (obj_in_cs.border_.points_[2].x_ + obj_in_cs.border_.points_[3].x_) * 0.50;
  if (t_01_f64 >= t_23_f64)
  {
    obj_out_cs.heading_f_ = 0.f;
  }
  else
  {
    obj_out_cs.heading_f_ = G_PI;
  }
}
void StaticObjProcess::SetLaneMarkingInputMap(const EmLaneMarking& obj_in_cs, const bc::bool_t new_obj_b,
                                              LaneMarkingInfo& obj_out_cs)
{
  obj_out_cs.id_s32_ = obj_in_cs.id_;
  obj_out_cs.updated_b_ = bc::true_v;

  /** currently life_time is still from last cycle. */
  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y;
    obj_out_cs.life_time_f_ = 0.3;
    obj_out_cs.source_en_ = StaticObjSource::kMap;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.95, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y;
  }

  obj_out_cs.type_u16_ = obj_in_cs.type_;
  obj_out_cs.lane_assigned_u8_ = kUnknownLane;
}

void StaticObjProcess::LaneMarkingLaneAssignment()
{
  /* lane assignment for static objects, especially for lane markings.
  1. should be executed after em lane process, can follow olr.
  2. simply check lane assignment.
  */
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    bc::uint8_t t_idx_lane_assign_u8 = kUnknownLane;
    if ((lane_marking_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (lane_marking_ar_[t_idx_obj_u8].type_u16_ != 0))
    {
      for (bc::uint8_t t_idx_lane_u8 = 0; t_idx_lane_u8 < kMaxLaneNum; t_idx_lane_u8++)
      {
        if ((em_collection_ptr_->lanes_[t_idx_lane_u8].lane_valid_ == bc::true_v) &&
            (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].element_valid_ == bc::true_v))
        {
          LaneBoundary& t_left_bound_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].left_boundary_;
          LaneBoundary& t_right_bound_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].right_boundary_;
          ReferenceLine& t_ref_line_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].reference_line_;
          if ((t_left_bound_cs.existence_ == bc::true_v) && (t_right_bound_cs.existence_ == bc::true_v) &&
              (t_ref_line_cs.available_ == bc::true_v))
          {
            bc::float32_t t_dx_f = lane_marking_ar_[t_idx_obj_u8].dx_f_;
            bc::float32_t t_dy_f = lane_marking_ar_[t_idx_obj_u8].dy_f_;
            bc::uint16_t t_idx_u16 = 0;
            bc::bool_t t_find_idx_b = FindNearestRefIdx(t_ref_line_cs, t_dx_f, t_idx_u16);
            if (t_find_idx_b == bc::true_v)
            {
              if ((t_dy_f > t_right_bound_cs.pts_[t_idx_u16].y) && (t_dy_f < t_left_bound_cs.pts_[t_idx_u16].y))
              {
                t_idx_lane_assign_u8 = t_idx_lane_u8;
                break;
              }
            }
            else
            {
            }
          }
        }
      }
    }
    lane_marking_ar_[t_idx_obj_u8].lane_assigned_u8_ = t_idx_lane_assign_u8;
  }
}

void StaticObjProcess::LaneMarkingArray2Vector()
{
  /** only store lane markings with valid lane assignment into lane marking vector. */
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((lane_marking_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (lane_marking_ar_[t_idx_obj_u8].type_u16_ != 0) &&
        (lane_marking_ar_[t_idx_obj_u8].lane_assigned_u8_ != kUnknownLane) &&
        (lane_marking_ar_[t_idx_obj_u8].dx_f_ > -10.0))
    {
      lane_marking_vt_.push_back(lane_marking_ar_[t_idx_obj_u8]);
    }
  }
  /** sort by dx, in ascend order.*/
  if (!lane_marking_vt_.empty())
  {
    std::sort(lane_marking_vt_.begin(), lane_marking_vt_.end(), LaneMarkingDxCompare);
  }
}

void StaticObjProcess::LaneMarkingClustering()
{
  for (bc::uint8_t t_cnt_marking_u8 = 0; t_cnt_marking_u8 < lane_marking_vt_.size(); t_cnt_marking_u8++)
  {
    /** if there is no any cluster, directly create a new cluster by current index. */
    if (lane_marking_cluster_vt_.empty() == bc::true_v)
    {
      CreateLaneMarkingCluster(lane_marking_vt_[t_cnt_marking_u8]);
    }
    else
    {
      /** there is at least one existing cluster. Let's loop all cluster.*/
      bc::bool_t t_cluster_found_b = bc::false_v;
      for (bc::uint8_t t_cnt_clst_u8 = 0; t_cnt_clst_u8 < lane_marking_cluster_vt_.size(); t_cnt_clst_u8++)
      {
        /** if the dx is close to the mean value of this cluster, add obj into this cluster. */
        bc::float32_t t_dx_diff_f =
            lane_marking_vt_[t_cnt_marking_u8].dx_f_ - lane_marking_cluster_vt_[t_cnt_clst_u8].dx_tracking_f_;
        if (fabs(t_dx_diff_f) < 10.0)
        {
          InsertLaneMarkingToCluster(lane_marking_vt_[t_cnt_marking_u8], lane_marking_cluster_vt_[t_cnt_clst_u8]);
          t_cluster_found_b = bc::true_v;
          break;
        }
      }
      if ((t_cluster_found_b == bc::false_v) && (lane_marking_cluster_vt_.size() < kMaxClusterNumLaneMarking))
      {
        CreateLaneMarkingCluster(lane_marking_vt_[t_cnt_marking_u8]);
      }
    }
  }
}

void StaticObjProcess::CreateLaneMarkingCluster(const LaneMarkingInfo& lane_marking_cs)
{

  LaneMarkingCluster t_cluster_cs = LaneMarkingCluster();
  LaneMarkingSubCluster t_subcluster_cs = LaneMarkingSubCluster();
  t_subcluster_cs.lane_marking_pack_vt_.push_back(lane_marking_cs);
  t_subcluster_cs.lane_assigned_u8_ = lane_marking_cs.lane_assigned_u8_;
  t_cluster_cs.lane_marking_subcluster_vt_.push_back(t_subcluster_cs);
  t_cluster_cs.num_lane_marking_ = 1;
  t_cluster_cs.new_create_b_ = bc::true_v;
  t_cluster_cs.dx_mean_f_ = lane_marking_cs.dx_f_;
  lane_marking_cluster_vt_.push_back(t_cluster_cs);
}

void StaticObjProcess::InsertLaneMarkingToCluster(const LaneMarkingInfo& lane_marking_cs,
                                                  LaneMarkingCluster& lane_marking_cluster_cs)
{
  bc::bool_t t_subcluster_found_b = bc::false_v;
  bc::bool_t t_insert_b = bc::false_v;
  for (bc::uint8_t t_cnt_u8 = 0; t_cnt_u8 < lane_marking_cluster_cs.lane_marking_subcluster_vt_.size(); t_cnt_u8++)
  {
    LaneMarkingSubCluster& t_subcluster_cs = lane_marking_cluster_cs.lane_marking_subcluster_vt_[t_cnt_u8];

    if (lane_marking_cs.lane_assigned_u8_ == t_subcluster_cs.lane_assigned_u8_)
    {
      if (t_subcluster_cs.lane_marking_pack_vt_.size() < kMaxNumLaneMarkingPack)
      {
        t_subcluster_cs.lane_marking_pack_vt_.push_back(lane_marking_cs);
        t_insert_b = bc::true_v;
      }
      else
      {
        std::sort(t_subcluster_cs.lane_marking_pack_vt_.begin(), t_subcluster_cs.lane_marking_pack_vt_.end(),
                  LaneMarkingLifeTimeCompare);
        if (t_subcluster_cs.lane_marking_pack_vt_.back().updated_b_ == bc::false_v)
        {
          t_subcluster_cs.lane_marking_pack_vt_.pop_back();
          t_subcluster_cs.lane_marking_pack_vt_.push_back(lane_marking_cs);
          t_insert_b = bc::true_v;
        }
        else
        {
          t_insert_b = bc::false_v;
        }
      }

      t_subcluster_found_b = bc::true_v;
      break;
    }
  }
  /** create a new subcluster */
  if (t_subcluster_found_b == bc::false_v)
  {
    LaneMarkingSubCluster t_subcluster_cs = LaneMarkingSubCluster();
    t_subcluster_cs.lane_marking_pack_vt_.push_back(lane_marking_cs);
    t_subcluster_cs.lane_assigned_u8_ = lane_marking_cs.lane_assigned_u8_;
    lane_marking_cluster_cs.lane_marking_subcluster_vt_.push_back(t_subcluster_cs);
    t_insert_b = bc::true_v;
  }

  if (t_insert_b == bc::true_v)
  {
    lane_marking_cluster_cs.dx_mean_f_ =
        lane_marking_cluster_cs.dx_mean_f_ * lane_marking_cluster_cs.num_lane_marking_ + lane_marking_cs.dx_f_;
    lane_marking_cluster_cs.dx_mean_f_ =
        lane_marking_cluster_cs.dx_mean_f_ / (lane_marking_cluster_cs.num_lane_marking_ + 1);

    lane_marking_cluster_cs.num_lane_marking_++;
  }
}

void StaticObjProcess::UpdateLaneMarkingClusterList()
{
  for (auto iter_ptr = lane_marking_cluster_vt_.begin(); iter_ptr != lane_marking_cluster_vt_.end();)
  {
    /** delete empty cluster if it contains no lane marking and its life time exhausted. */
    bc::bool_t t_del_old_cluster_b =
        (iter_ptr->new_create_b_ == bc::false_v) &&
        (((iter_ptr->num_lane_marking_ == 0) && (iter_ptr->life_time_f_ < time_cycle_f_)) ||
         (iter_ptr->dx_tracking_f_ < -10.0));
    bc::bool_t t_del_new_cluster_b = (iter_ptr->new_create_b_ == bc::true_v) && (iter_ptr->dx_mean_f_ < -0.0);
    if (t_del_old_cluster_b || t_del_new_cluster_b)
    {
      /** iter_ptr automatically increase by the return of erase method. */
      iter_ptr = lane_marking_cluster_vt_.erase(iter_ptr);
    }
    else
    {
      /** 1. update dx in cluster */
      if (iter_ptr->new_create_b_ == bc::true_v)
      {
        /** new cluster, dx_tracking_f_ directly equals to dx_mean_f_.
         * life time accumulated.
         */
        iter_ptr->dx_tracking_f_ = iter_ptr->dx_mean_f_;
        iter_ptr->life_time_f_ = 0.3;
      }
      else
      {
        /** if the cluster contains any lane marking in current cycle, update dx_tracking_f_ by filtering dx_mean_f_ and
         * accumulate life time. Otherwise keep dx_tracking_f_ and drease life time. dx_tracking_f_ will be updated at
         * the beginning*/
        if (iter_ptr->num_lane_marking_ > 0)
        {
          if (v_ego_f_ > 0.5)
          {
            bc::float32_t t_k_f =
                LinearInterpolation(0.3, 0.9, 2.0, 0.99, iter_ptr->life_time_f_, Lin_Interp_Method::kFlat);
            iter_ptr->dx_tracking_f_ = t_k_f * iter_ptr->dx_tracking_f_ + (1 - t_k_f) * iter_ptr->dx_mean_f_;
          }
          else
          {
            /** very low speed, do nothing, keep dx_tracking_f_ as pure tracking value*/
          }
          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ + time_cycle_f_;
        }
        else
        {
          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ - time_cycle_f_;
        }
      }

      /** 2. update dy in subcluster according to dx calculated above */
      for (bc::uint8_t t_cnt_u8 = 0; t_cnt_u8 < iter_ptr->lane_marking_subcluster_vt_.size(); t_cnt_u8++)
      {
        bc::uint8_t t_idx_lane_u8 = iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_assigned_u8_;
        ReferenceLine& t_ref_line_cs = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].reference_line_;
        bc::bool_t t_opposite_lane_b = em_collection_ptr_->lanes_[t_idx_lane_u8].opposite_lane_;

        bc::float32_t t_dx_f = iter_ptr->dx_tracking_f_;
        bc::float32_t t_dy_f = 0.0;

        if (t_dx_f <= t_ref_line_cs.ref_line_pts_[0].pos_.x)
        {
          t_dy_f = t_ref_line_cs.ref_line_pts_[0].pos_.y;
        }
        else if (t_dx_f >= t_ref_line_cs.ref_line_pts_[kMaxRefLinePtsNum - 1].pos_.x)
        {
          t_dy_f = t_ref_line_cs.ref_line_pts_[kMaxRefLinePtsNum - 1].pos_.y;
        }
        else
        {
          for (bc::uint16_t t_idx_pt_u16 = 1; t_idx_pt_u16 < kMaxRefLinePtsNum; t_idx_pt_u16++)
          {
            if ((t_dx_f > t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.x) &&
                (t_dx_f < t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.x))
            {
              t_dy_f = LinearInterpolation(t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.x,
                                           t_ref_line_cs.ref_line_pts_[t_idx_pt_u16 - 1].pos_.y,
                                           t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.x,
                                           t_ref_line_cs.ref_line_pts_[t_idx_pt_u16].pos_.y, t_dx_f,
                                           Lin_Interp_Method::kFlat);
              break;
            }
          }
        }
        iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].dy_mean_f_ = t_dy_f;

        if (t_opposite_lane_b == bc::false_v)
        {
          if (!iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.empty() &&
              iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_assigned_u8_ != kUnknownLane)
          {
            for (auto iter_lane_marking_ptr =
                     iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.begin();
                 iter_lane_marking_ptr != iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.end();
                 iter_lane_marking_ptr++)
            {
              if (iter_lane_marking_ptr->heading_f_ > FLOAT32_EPSILON)
              {
                iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].heading_f_ = G_PI;
                break;
              }
              else
              {
                // do nothing
              }
            }
          }
          else
          {
            // do nothing
          }
        }
        else
        {
          iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].heading_f_ = G_PI;
        }

        /** 3. sort lane_marking_pack_vt_ in subcluster by their life time, in descend order */
        if (!iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.empty())
        {
          std::sort(iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.begin(),
                    iter_ptr->lane_marking_subcluster_vt_[t_cnt_u8].lane_marking_pack_vt_.end(),
                    LaneMarkingLifeTimeCompare);
        };
      }
      /** iter_ptr accumulated. */
      iter_ptr++;
    }
  }
}

void StaticObjProcess::LaneMarkingDistillation()
{
  lane_marking_vt_.clear();
  for (auto iter_cluster_ptr = lane_marking_cluster_vt_.begin(); iter_cluster_ptr != lane_marking_cluster_vt_.end();
       iter_cluster_ptr++)
  {
    bc::float32_t t_dx_clustered_f = iter_cluster_ptr->dx_tracking_f_;
    if (!iter_cluster_ptr->lane_marking_subcluster_vt_.empty())
    {
      for (auto iter_subcluster_ptr = iter_cluster_ptr->lane_marking_subcluster_vt_.begin();
           iter_subcluster_ptr != iter_cluster_ptr->lane_marking_subcluster_vt_.end(); iter_subcluster_ptr++)
      {
        bc::float32_t t_dy_clustered_f = iter_subcluster_ptr->dy_mean_f_;
        bc::float32_t t_heading_clustered_f = iter_subcluster_ptr->heading_f_;

        if (!iter_subcluster_ptr->lane_marking_pack_vt_.empty())
        {
          /** only choose the first lane marking in lane_marking_pack_vt_ because the vector is already sorted
           * by life time. */
          LaneMarkingInfo t_lane_marking_cs = iter_subcluster_ptr->lane_marking_pack_vt_[0];
          if (t_lane_marking_cs.life_time_f_ > 0.1)
          {
            t_lane_marking_cs.dx_f_ = t_dx_clustered_f;
            t_lane_marking_cs.dy_f_ = t_dy_clustered_f;
            t_lane_marking_cs.heading_f_ = t_heading_clustered_f;
            lane_marking_vt_.push_back(t_lane_marking_cs);
          }
        }
      }
    }
  }
}

void StaticObjProcess::LaneMarkingOutput()
{
  bc::uint8_t t_cnt_lane_marking_u8 = 0;
  if (!lane_marking_vt_.empty())
  {
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (t_cnt_lane_marking_u8 < lane_marking_vt_.size())
      {
        bc::int32_t t_type_s32 = static_obj_ar_[t_idx_u8].type_;
        bc::int32_t t_id_s32 = static_obj_ar_[t_idx_u8].id_;
        if ((t_type_s32 == 0) || (t_id_s32 == kInvalidStaticObjId))
        {
          EmStaticObject t_em_static_obj_cs = EmStaticObject();
          t_em_static_obj_cs.id_ = lane_marking_vt_[t_cnt_lane_marking_u8].id_s32_;
          t_em_static_obj_cs.type_ = 4;
          t_em_static_obj_cs.sub_type_ =
              LaneMarkingTypeMappingEm2Horizon(lane_marking_vt_[t_cnt_lane_marking_u8].type_u16_);
          t_em_static_obj_cs.position_.x = lane_marking_vt_[t_cnt_lane_marking_u8].dx_f_;
          t_em_static_obj_cs.position_.y = lane_marking_vt_[t_cnt_lane_marking_u8].dy_f_;
          t_em_static_obj_cs.heading_ = lane_marking_vt_[t_cnt_lane_marking_u8].heading_f_;
          static_obj_ar_[t_idx_u8] = t_em_static_obj_cs;
          t_cnt_lane_marking_u8++;
        }
      }
      else
      {
        break;
      }
    }

    bc::uint8_t t_count_lane_marking_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLaneMarkingNum; t_idx_u8++)
    {
      if (t_count_lane_marking_u8 < lane_marking_vt_.size())
      {
        EmLaneMarking& t_em_lane_marking_cs = em_collection_ptr_->static_object_.lane_marking_ar_[t_idx_u8];
        t_em_lane_marking_cs.id_ = lane_marking_vt_[t_count_lane_marking_u8].id_s32_;
        t_em_lane_marking_cs.type_ = lane_marking_vt_[t_count_lane_marking_u8].type_u16_;
        t_em_lane_marking_cs.lane_assigned_ = lane_marking_vt_[t_count_lane_marking_u8].lane_assigned_u8_;
        t_em_lane_marking_cs.position_.x = lane_marking_vt_[t_count_lane_marking_u8].dx_f_;
        t_em_lane_marking_cs.position_.y = lane_marking_vt_[t_count_lane_marking_u8].dy_f_;
        t_em_lane_marking_cs.heading_ = lane_marking_vt_[t_count_lane_marking_u8].heading_f_;
        t_em_lane_marking_cs.valid_ = bc::true_v;
        t_count_lane_marking_u8++;
      }
      else
      {
        break;
      }
    }
  }
}

bc::int32_t StaticObjProcess::LaneMarkingTypeMappingEm2Horizon(const bc::uint16_t type_em_u16)
{
  // enum class EmLaneMarkingTypeMsk : bc::uint8_t
  // {
  //   kForwardMsk = 0,
  //   kTurnLeftMsk = 1,
  //   kTurnRightMsk = 2,
  //   kUTurnMsk = 3,
  //   kMergeLeftMsk = 4,
  //   kMergeRightMsk = 5,
  //   kProhibitMsk = 6
  // };

  /** temporarily mapping from em to horizon, delete later, so prohibit mark is not mapping here. */
  bc::int32_t t_type_horizon_s32 = 0;
  switch (type_em_u16)
  {
    case 1:
      t_type_horizon_s32 = 2;
      break;
    case 2:
      t_type_horizon_s32 = 1;
      break;
    case 3:
      t_type_horizon_s32 = 4;
      break;
    case 4:
      t_type_horizon_s32 = 3;
      break;
    case 5:
      t_type_horizon_s32 = 5;
      break;
    case 6:
      t_type_horizon_s32 = 6;
      break;
    case 8:
      t_type_horizon_s32 = 7;
      break;
    case 9:
      t_type_horizon_s32 = 8;
      break;
    case 10:
      t_type_horizon_s32 = 9;
      break;
    case 16:
      t_type_horizon_s32 = 10;
      break;
    case 32:
      t_type_horizon_s32 = 11;
      break;
    default:
      t_type_horizon_s32 = 0;
      break;
  }
  return (t_type_horizon_s32);
}
bc::uint16_t StaticObjProcess::LaneMarkingTypeMapping(const bc::int32_t type_horizon_s32)
{
  bc::uint16_t t_type_u16 = 0;

  // enum class EmLaneMarkingTypeMsk : bc::uint8_t {
  //   kForwardMsk = 0,
  //   kTurnLeftMsk = 1,
  //   kTurnRightMsk = 2,
  //   kUTurnMsk = 3,
  //   kMergeLeftMsk = 4,
  //   kMergeRightMsk = 5,
  //   kProhibitMsk = 6
  // };

  switch (type_horizon_s32)
  {
    case 1:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      break;

    case 2:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kForwardMsk, bc::true_v, t_type_u16);
      break;

    case 3:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      break;

    case 4:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kForwardMsk, bc::true_v, t_type_u16);
      break;

    case 5:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kForwardMsk, bc::true_v, t_type_u16);
      break;

    case 6:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      break;

    case 7:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kUTurnMsk, bc::true_v, t_type_u16);
      break;

    case 8:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kUTurnMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kForwardMsk, bc::true_v, t_type_u16);
      break;

    case 9:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kUTurnMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      break;

    case 10:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kMergeLeftMsk, bc::true_v, t_type_u16);
      break;

    case 11:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kMergeRightMsk, bc::true_v, t_type_u16);
      break;

    case 15:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      break;

    case 16:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      break;

    case 17:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      break;

    case 18:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      break;

    case 19:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnLeftMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kUTurnMsk, bc::true_v, t_type_u16);
      break;

    case 20:
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kProhibitMsk, bc::true_v, t_type_u16);
      SetBitU16((bc::uint8_t)EmLaneMarkingTypeMsk::kTurnRightMsk, bc::true_v, t_type_u16);
      break;

    default:
      t_type_u16 = 0;
  }
  return t_type_u16;
}

void StaticObjProcess::CrossStopLineStablization(
    const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
    const StaticObjectData& per_static_obj_cs,
    const bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum>(&map_cross_stop_line_ar))
{
  TrackingCrossStopLineArrayAndCluster(ego_motion_cs);
  UpdateCrossStopLineArray(static_obj_diag_cs, per_static_obj_cs, map_cross_stop_line_ar);
  CrossStopLineRoadAssignment();
  CrossStopLineArray2Vector();
  CrossStopLineClustering();
  UpdateCrossStopLineClusterList();
  CrossStopLineDistillation();
  CrossStopLineOutput();
}

void StaticObjProcess::TrackingCrossStopLineArrayAndCluster(const EgoPoseCollection& ego_motion_cs)
{
  TransForm t_transform_st = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::float32_t t_dx_trans_f = t_transform_st.translation_dx_f_;
  bc::float32_t t_dy_trans_f = t_transform_st.translation_dy_f_;
  bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f),
                                                                   -sinf(t_rotation_f), cosf(t_rotation_f)};
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((cross_stop_line_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (cross_stop_line_ar_[t_idx_obj_u8].type_en_ != EmCrossStopLineType::kUnknown))
    {
      /** TODO: update dx,dy by ego motion*/

      cross_stop_line_ar_[t_idx_obj_u8].dx_f_ = cross_stop_line_ar_[t_idx_obj_u8].dx_f_ - t_dx_trans_f;
      cross_stop_line_ar_[t_idx_obj_u8].dy_f_ = cross_stop_line_ar_[t_idx_obj_u8].dy_f_ - t_dy_trans_f;

      bc::float32_t t_dx_f = t_trans_matrix_ar[0] * cross_stop_line_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[1] * cross_stop_line_ar_[t_idx_obj_u8].dy_f_;
      bc::float32_t t_dy_f = t_trans_matrix_ar[2] * cross_stop_line_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[3] * cross_stop_line_ar_[t_idx_obj_u8].dy_f_;

      cross_stop_line_ar_[t_idx_obj_u8].dx_f_ = t_dx_f;
      cross_stop_line_ar_[t_idx_obj_u8].dy_f_ = t_dy_f;
      cross_stop_line_ar_[t_idx_obj_u8].heading_f_ = cross_stop_line_ar_[t_idx_obj_u8].heading_f_ - t_rotation_f;
      HeadingRangeNormalize(cross_stop_line_ar_[t_idx_obj_u8].heading_f_);
      cross_stop_line_ar_[t_idx_obj_u8].updated_b_ = bc::false_v;
    }
  }

  for (bc::uint8_t t_idx_cluster_u8 = 0; t_idx_cluster_u8 < cross_stop_line_cluster_vt_.size(); t_idx_cluster_u8++)
  {
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].cross_stop_line_pack_vt_.clear();
    // cross_stop_line_cluster_vt_[t_idx_cluster_u8].cluster_type_u8_ = 0;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].num_cross_stop_line_ = 0;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].new_create_b_ = bc::false_v;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_mean_f_ = 0.0;

    /** TODO: update dx,dy by ego motion*/
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ =
        cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ - t_dx_trans_f;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ =
        cross_stop_line_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ - t_dy_trans_f;
    bc::float32_t t_dx_f = t_trans_matrix_ar[0] * cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ +
                           t_trans_matrix_ar[1] * cross_stop_line_cluster_vt_[t_idx_cluster_u8].dy_mean_f_;
    bc::float32_t t_dy_f = t_trans_matrix_ar[2] * cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ +
                           t_trans_matrix_ar[3] * cross_stop_line_cluster_vt_[t_idx_cluster_u8].dy_mean_f_;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ = t_dx_f;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ = t_dy_f;
    cross_stop_line_cluster_vt_[t_idx_cluster_u8].heading_mean_f_ =
        cross_stop_line_cluster_vt_[t_idx_cluster_u8].heading_mean_f_ - t_rotation_f;
    HeadingRangeNormalize(cross_stop_line_cluster_vt_[t_idx_cluster_u8].heading_mean_f_);
  }
}

void StaticObjProcess::UpdateCrossStopLineArray(
    const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs,
    const bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum>(&map_cross_stop_line_ar))
{
  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  { /** only process when status is not error confirmed. */

    /** 1. create a list, indicating input static objs are matched with last cycle. */
    bc::TCArray<bc::bool_t, kMaxStaticObjNum> t_input_match_per_ar;
    bc::TCArray<bc::bool_t, kMaxCrossStopLineNum> t_input_match_map_ar;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      t_input_match_per_ar[t_idx_u8] = bc::false_v;
    }
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxCrossStopLineNum; t_idx_u8++)
    {
      t_input_match_map_ar[t_idx_u8] = bc::false_v;
    }

    /** 2. loop cross_stop_line_ar_ in last cycle, the last cycle cross/stop line and the current input obj are matched
     * if
     * a) same type
     * b) same id
     */
    /** NOTICE: cross_stop_line_ar_ now stores info from last cycle, so temp variables below are named by "lst1" */
    for (bc::uint8_t t_idx_lst_u8 = 0; t_idx_lst_u8 < kMaxStaticObjNum; t_idx_lst_u8++)
    {
      /** check if last cycle cross/stop line is valid */
      EmCrossStopLineType t_type_lst1_en = cross_stop_line_ar_[t_idx_lst_u8].type_en_;
      bc::int32_t t_id_lst1_s32 = cross_stop_line_ar_[t_idx_lst_u8].id_s32_;
      StaticObjSource t_source_lst1_en = cross_stop_line_ar_[t_idx_lst_u8].source_en_;
      if ((t_type_lst1_en != EmCrossStopLineType::kUnknown) && (t_id_lst1_s32 != kInvalidStaticObjId))
      {
        if (t_source_lst1_en == StaticObjSource::kPerception)
        {
          /** if last cycle cross/stop line is valid, loop input obj list and try to match an input obj. */
          for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
          {
            /** check if input obj is valid */
            bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
            bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
            if ((t_type_new_s32 == 5 || t_type_new_s32 == 8) && (t_id_new_s32 != kInvalidStaticObjId))
            {
              /** two objs are matched if their types and ids are the same. */
              if ((t_id_lst1_s32 == t_id_new_s32) &&
                  (((t_type_new_s32 == 5) && (t_type_lst1_en == EmCrossStopLineType::kStopLine)) ||
                   ((t_type_new_s32 == 8) && (t_type_lst1_en == EmCrossStopLineType::kCrossLine))))
              {
                /** set a flag in match list, and set input obj into the internal obj array with the old index. */
                t_input_match_per_ar[t_idx_new_u8] = bc::true_v;
                bc::bool_t t_new_cross_stop_b = bc::false_v;
                SetCrossStopLineInputPer(per_static_obj_cs.objects_[t_idx_new_u8], t_new_cross_stop_b,
                                         cross_stop_line_ar_[t_idx_lst_u8]);
                cross_stop_line_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
                break;
              }
            }
          }
        }
        else if (t_source_lst1_en == StaticObjSource::kMap)
        {
          for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxCrossStopLineNum; t_idx_new_u8++)
          {
            /** check if input obj is valid */
            EmCrossStopLineType t_type_new_en = map_cross_stop_line_ar[t_idx_new_u8].type_;
            bc::int32_t t_id_new_s32 = map_cross_stop_line_ar[t_idx_new_u8].id_;
            bc::bool_t t_valid_b = map_cross_stop_line_ar[t_idx_new_u8].valid_;
            if ((t_type_new_en != EmCrossStopLineType::kUnknown) && (t_id_new_s32 != kInvalidStaticObjId) &&
                (t_valid_b == bc::true_v))
            {
              /** two objs are matched if their types and ids are the same. */
              if ((t_id_lst1_s32 == t_id_new_s32) && (t_type_lst1_en == t_type_new_en))
              {
                /** set a flag in match list, and set input obj into the internal obj array with the old index. */
                t_input_match_map_ar[t_idx_new_u8] = bc::true_v;
                bc::bool_t t_new_cross_stop_b = bc::false_v;
                SetCrossStopLineInputMap(map_cross_stop_line_ar[t_idx_new_u8], t_new_cross_stop_b,
                                         cross_stop_line_ar_[t_idx_lst_u8]);
                cross_stop_line_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
                break;
              }
            }
          }
        }
      }
    }

    /** 3. delete lane markings in cross_stop_line_ar_ if its update is false and life time is exhausted.*/
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (((cross_stop_line_ar_[t_idx_u8].type_en_ != EmCrossStopLineType::kUnknown) &&
           (cross_stop_line_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId) &&
           (cross_stop_line_ar_[t_idx_u8].updated_b_ == bc::false_v) &&
           (cross_stop_line_ar_[t_idx_u8].life_time_f_ < time_cycle_f_)) ||
          (cross_stop_line_ar_[t_idx_u8].dx_f_ < -20.0))
      {
        cross_stop_line_ar_[t_idx_u8] = CrossStopLineInfo();
      }
    }

    /** 4. after above loop, all matched cross/stop line are stored into the cross_stop_line_ar_ with the old index.
     * Below code is to insert remaining new cross/stop line into absent position in cross_stop_line_ar_.
     */
    bc::uint8_t t_idx_start_u8 = 0;
    /** loop internal obj array */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      EmCrossStopLineType t_type_en = cross_stop_line_ar_[t_idx_u8].type_en_;
      bc::int32_t t_id_s32 = cross_stop_line_ar_[t_idx_u8].id_s32_;
      if ((t_type_en == EmCrossStopLineType::kUnknown) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_input_match_per_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_s32 == 5 || t_type_new_s32 == 8) &&
              (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            bc::bool_t t_new_cross_stop_b = bc::true_v;
            SetCrossStopLineInputPer(per_static_obj_cs.objects_[t_idx_new_u8], t_new_cross_stop_b,
                                     cross_stop_line_ar_[t_idx_u8]);
            cross_stop_line_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 >= kMaxStaticObjNum)
      {
        break;
      }
    }

    t_idx_start_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      EmCrossStopLineType t_type_en = cross_stop_line_ar_[t_idx_u8].type_en_;
      bc::int32_t t_id_s32 = cross_stop_line_ar_[t_idx_u8].id_s32_;
      if ((t_type_en == EmCrossStopLineType::kUnknown) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxCrossStopLineNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          EmCrossStopLineType t_type_new_en = map_cross_stop_line_ar[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = map_cross_stop_line_ar[t_idx_new_u8].id_;
          bc::bool_t t_valid_b = map_cross_stop_line_ar[t_idx_new_u8].valid_;
          if ((t_input_match_map_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_en != EmCrossStopLineType::kUnknown) &&
              (t_id_new_s32 != kInvalidStaticObjId) && (t_valid_b == bc::true_v))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            bc::bool_t t_new_cross_stop_b = bc::true_v;
            SetCrossStopLineInputMap(map_cross_stop_line_ar[t_idx_new_u8], t_new_cross_stop_b,
                                     cross_stop_line_ar_[t_idx_u8]);
            cross_stop_line_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 >= kMaxCrossStopLineNum)
      {
        break;
      }
    }

    /** 5. update life time according to update_b flag. */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if ((cross_stop_line_ar_[t_idx_u8].type_en_ != EmCrossStopLineType::kUnknown) &&
          (cross_stop_line_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId))
      {
        if (cross_stop_line_ar_[t_idx_u8].updated_b_ == bc::true_v)
        {
          cross_stop_line_ar_[t_idx_u8].life_time_f_ = cross_stop_line_ar_[t_idx_u8].life_time_f_ + time_cycle_f_;
        }
        else
        {
          cross_stop_line_ar_[t_idx_u8].life_time_f_ = cross_stop_line_ar_[t_idx_u8].life_time_f_ - time_cycle_f_;
        }
      }
    }
  }
  else
  { /** input invalid, reset timestamp. static obj array keep default value. */
  }
}

void StaticObjProcess::SetCrossStopLineInputPer(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b,
                                                CrossStopLineInfo& obj_out_cs)
{
  obj_out_cs.id_s32_ = obj_in_cs.id_;
  obj_out_cs.updated_b_ = bc::true_v;
  obj_out_cs.distilled_b_ = bc::false_v;
  /** currently life_time is still from last cycle. */
  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y_;
    obj_out_cs.life_time_f_ = 0.3;
    obj_out_cs.source_en_ = StaticObjSource::kPerception;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y_;
  }

  obj_out_cs.type_en_ = CrossStopLineTypeMapping(obj_in_cs.type_);
  CalcLengthWidthHeading(obj_in_cs, obj_out_cs.length_side_f_, obj_out_cs.width_side_f_, obj_out_cs.heading_f_);
}

void StaticObjProcess::SetCrossStopLineInputMap(const EmCrossStopLine& obj_in_cs, const bc::bool_t new_obj_b,
                                                CrossStopLineInfo& obj_out_cs)
{
  obj_out_cs.id_s32_ = obj_in_cs.id_;
  obj_out_cs.updated_b_ = bc::true_v;
  obj_out_cs.distilled_b_ = bc::false_v;
  /** currently life_time is still from last cycle. */
  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y;
    obj_out_cs.life_time_f_ = 0.3;
    obj_out_cs.source_en_ = StaticObjSource::kMap;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.99, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y;
  }

  obj_out_cs.type_en_ = obj_in_cs.type_;
  obj_out_cs.length_side_f_ = obj_in_cs.length_;
  obj_out_cs.width_side_f_ = obj_in_cs.width_;
  obj_out_cs.heading_f_ = obj_in_cs.heading_;
}

void StaticObjProcess::CrossStopLineRoadAssignment()
{
  left_bound_cs_ = LaneBoundary();
  right_bound_cs_ = LaneBoundary();
  bc::bool_t t_left_bound_found_b = bc::false_v;
  bc::bool_t t_right_bound_found_b = bc::false_v;

  for (bc::uint8_t t_idx_lane_u8 = 0; t_idx_lane_u8 < kMaxLaneNum; t_idx_lane_u8++)
  {
    if ((em_collection_ptr_->lanes_[t_idx_lane_u8].lane_valid_ == bc::true_v) &&
        (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].element_valid_ == bc::true_v))
    {
      if (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].left_boundary_.existence_ == bc::true_v)
      {
        left_bound_cs_ = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].left_boundary_;
        t_left_bound_found_b = bc::true_v;
        break;
      }
    }
  }

  for (bc::int8_t t_idx_lane_s8 = kMaxLaneNum - 1; t_idx_lane_s8 >= 0; t_idx_lane_s8--)
  {
    bc::uint8_t t_idx_lane_u8 = (bc::uint8_t)(t_idx_lane_s8);
    if (t_idx_lane_u8 < kMaxLaneNum)
    {
      if ((em_collection_ptr_->lanes_[t_idx_lane_u8].lane_valid_ == bc::true_v) &&
          (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].element_valid_ == bc::true_v))
      {
        if (em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].right_boundary_.existence_ == bc::true_v)
        {
          right_bound_cs_ = em_collection_ptr_->lanes_[t_idx_lane_u8].lane_elements_[0].right_boundary_;
          t_right_bound_found_b = bc::true_v;
          break;
        }
      }
    }
    else
    {
      /** for code safety purpose only, should never enter this logic.*/
      break;
    }
  }

  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {

    if ((cross_stop_line_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (cross_stop_line_ar_[t_idx_obj_u8].type_en_ != EmCrossStopLineType::kUnknown))
    {
      bc::float32_t t_prob_f = 0.0;
      if ((t_left_bound_found_b == bc::true_v) && (t_right_bound_found_b == bc::true_v))
      {
        bc::float32_t t_heading_f = cross_stop_line_ar_[t_idx_obj_u8].heading_f_;
        HeadingRangeNormalize(t_heading_f);

        bc::float32_t t_dy_offset_f = 0.5 * cross_stop_line_ar_[t_idx_obj_u8].length_side_f_ * cos(t_heading_f);
        bc::float32_t t_left_box_point_f = cross_stop_line_ar_[t_idx_obj_u8].dy_f_ + t_dy_offset_f;
        bc::float32_t t_right_box_point_f = cross_stop_line_ar_[t_idx_obj_u8].dy_f_ - t_dy_offset_f;
        bc::float32_t t_dx_f = cross_stop_line_ar_[t_idx_obj_u8].dx_f_;
        bc::float32_t t_left_bound_point_f = 0.0;
        bc::float32_t t_right_bound_point_f = 0.0;
        if ((t_dx_f > left_bound_cs_.pts_[0].x) && (t_dx_f < left_bound_cs_.pts_[kMaxBoundaryPoint - 1].x))
        {
          for (bc::uint16_t t_idx_u16 = 1; t_idx_u16 < kMaxBoundaryPoint; t_idx_u16++)
          {
            if ((t_dx_f > left_bound_cs_.pts_[t_idx_u16 - 1].x) && (t_dx_f < left_bound_cs_.pts_[t_idx_u16].x))
            {
              t_left_bound_point_f = LinearInterpolation(
                  left_bound_cs_.pts_[t_idx_u16 - 1].x, left_bound_cs_.pts_[t_idx_u16 - 1].y,
                  left_bound_cs_.pts_[t_idx_u16].x, left_bound_cs_.pts_[t_idx_u16].y, t_dx_f, Lin_Interp_Method::kFlat);
              t_right_bound_point_f =
                  LinearInterpolation(right_bound_cs_.pts_[t_idx_u16 - 1].x, right_bound_cs_.pts_[t_idx_u16 - 1].y,
                                      right_bound_cs_.pts_[t_idx_u16].x, right_bound_cs_.pts_[t_idx_u16].y, t_dx_f,
                                      Lin_Interp_Method::kFlat);
              break;
            }
          }

          if ((t_right_box_point_f >= t_left_bound_point_f) || (t_left_box_point_f <= t_right_bound_point_f))
          {
            t_prob_f = 0.0;
          }
          else if ((t_right_box_point_f <= t_right_bound_point_f) && (t_left_box_point_f >= t_left_bound_point_f))
          {
            t_prob_f = 1.0;
          }
          else if (t_right_box_point_f > t_right_bound_point_f)
          {
            t_prob_f = (t_left_bound_point_f - t_right_box_point_f) / (t_left_bound_point_f - t_right_bound_point_f);
          }
          else
          {
            t_prob_f = (t_left_box_point_f - t_right_bound_point_f) / (t_left_bound_point_f - t_right_bound_point_f);
          }
        }
        else
        {
          t_prob_f = 0.0;
        }
      }
      else
      {
        t_prob_f = 0.0;
      }

      if ((t_prob_f < 0.0) || (t_prob_f > 1.0))
      {
        /** for safety purpose only, should always not enter here. */
        t_prob_f = 0.0;
      }
      cross_stop_line_ar_[t_idx_obj_u8].prob_road_cover_f_ = t_prob_f;
    }
  }
}

void StaticObjProcess::CrossStopLineArray2Vector()
{
  /** only store cross/stop line with valid road cover prob into cross/stop line vector. */
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    bc::float32_t t_prob_road_thres = 0.0;
    if (cross_stop_line_ar_[t_idx_obj_u8].type_en_ == EmCrossStopLineType::kStopLine)
    {
      t_prob_road_thres = 0.4;
    }
    else
    {
      t_prob_road_thres = 0.5;
    }
    if ((cross_stop_line_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (cross_stop_line_ar_[t_idx_obj_u8].type_en_ != EmCrossStopLineType::kUnknown) &&
        (cross_stop_line_ar_[t_idx_obj_u8].prob_road_cover_f_ > t_prob_road_thres) &&
        (cross_stop_line_ar_[t_idx_obj_u8].dx_f_ > -20.0))
    {
      cross_stop_line_vt_.push_back(cross_stop_line_ar_[t_idx_obj_u8]);
    }
  }
  /** sort by dx, in ascend order.*/
  if (!cross_stop_line_vt_.empty())
  {
    std::sort(cross_stop_line_vt_.begin(), cross_stop_line_vt_.end(), CrossStopLineDxCompare);
  }
}
void StaticObjProcess::CrossStopLineClustering()
{
  for (bc::uint8_t t_cnt_line_u8 = 0; t_cnt_line_u8 < cross_stop_line_vt_.size(); t_cnt_line_u8++)
  {
    /** if there is no any cluster, directly create a new cluster by current index. */
    if (cross_stop_line_cluster_vt_.empty() == bc::true_v)
    {
      CreateCrossStopLineCluster(cross_stop_line_vt_[t_cnt_line_u8]);
    }
    else
    {
      /** there is at least one existing cluster. Let's loop all cluster.*/
      bc::bool_t t_cluster_found_b = bc::false_v;
      for (bc::uint8_t t_cnt_clst_u8 = 0; t_cnt_clst_u8 < cross_stop_line_cluster_vt_.size(); t_cnt_clst_u8++)
      {
        /** if the dx is close to the mean value of this cluster, add obj into this cluster. */
        bc::float32_t t_dx_diff_f =
            cross_stop_line_vt_[t_cnt_line_u8].dx_f_ - cross_stop_line_cluster_vt_[t_cnt_clst_u8].dx_tracking_f_;
        if (fabs(t_dx_diff_f) < 10.0)
        {
          InsertCrossStopLineToCluster(cross_stop_line_vt_[t_cnt_line_u8], cross_stop_line_cluster_vt_[t_cnt_clst_u8]);
          t_cluster_found_b = bc::true_v;
          break;
        }
      }
      if ((t_cluster_found_b == bc::false_v) && (cross_stop_line_cluster_vt_.size() < kMaxClusterNumCrossStopLine))
      {
        CreateCrossStopLineCluster(cross_stop_line_vt_[t_cnt_line_u8]);
      }
    }
  }
}

void StaticObjProcess::CreateCrossStopLineCluster(const CrossStopLineInfo& cross_stop_line_cs)
{

  CrossStopLineCluster t_cluster_cs = CrossStopLineCluster();

  /** dx_tracking_f_ and life_time_f_ will be calculated later. */
  t_cluster_cs.cross_stop_line_pack_vt_.push_back(cross_stop_line_cs);
  t_cluster_cs.dx_mean_f_ = cross_stop_line_cs.dx_f_;
  t_cluster_cs.num_cross_stop_line_ = 1;
  t_cluster_cs.new_create_b_ = bc::true_v;

  t_cluster_cs.UpdateClusterType(cross_stop_line_cs.type_en_);

  cross_stop_line_cluster_vt_.push_back(t_cluster_cs);
}

void StaticObjProcess::InsertCrossStopLineToCluster(const CrossStopLineInfo& cross_stop_line_cs,
                                                    CrossStopLineCluster& cross_stop_line_cluster_cs)
{
  bc::bool_t t_insert_b = bc::false_v;
  if (cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.size() < kMaxNumCrossStopLinePack)
  {
    cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.push_back(cross_stop_line_cs);
    t_insert_b = bc::true_v;
  }
  else
  {
    std::sort(cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.begin(),
              cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.end(), CrossStopLineLifeTimeCompare);
    if (cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.back().updated_b_ == bc::false_v)
    {
      cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.pop_back();
      cross_stop_line_cluster_cs.cross_stop_line_pack_vt_.push_back(cross_stop_line_cs);
      t_insert_b = bc::true_v;
    }
    else
    {
      t_insert_b = bc::false_v;
    }
  }

  if (t_insert_b == bc::true_v)
  {
    cross_stop_line_cluster_cs.dx_mean_f_ =
        cross_stop_line_cluster_cs.dx_mean_f_ * cross_stop_line_cluster_cs.num_cross_stop_line_ +
        cross_stop_line_cs.dx_f_;
    cross_stop_line_cluster_cs.dx_mean_f_ =
        cross_stop_line_cluster_cs.dx_mean_f_ / (cross_stop_line_cluster_cs.num_cross_stop_line_ + 1);
    cross_stop_line_cluster_cs.num_cross_stop_line_++;

    cross_stop_line_cluster_cs.UpdateClusterType(cross_stop_line_cs.type_en_);
  }
  else
  {
    /** this line has not been inserted into pack, do nothing. */
  }
}

void StaticObjProcess::UpdateCrossStopLineClusterList()
{
  for (auto iter_ptr = cross_stop_line_cluster_vt_.begin(); iter_ptr != cross_stop_line_cluster_vt_.end();)
  {
    /** delete empty cluster if it contains no cross/stop line and its life time exhausted. */
    bc::bool_t t_del_old_cluster_b =
        (iter_ptr->new_create_b_ == bc::false_v) &&
        (((iter_ptr->num_cross_stop_line_ == 0) && (iter_ptr->life_time_f_ < time_cycle_f_)) ||
         (iter_ptr->dx_tracking_f_ < -20.0));
    bc::bool_t t_del_new_cluster_b = (iter_ptr->new_create_b_ == bc::true_v) && (iter_ptr->dx_mean_f_ < 0.0);

    if (t_del_old_cluster_b || t_del_new_cluster_b)
    {
      /** iter_ptr automatically increase by the return of erase method. */
      iter_ptr = cross_stop_line_cluster_vt_.erase(iter_ptr);
    }
    else
    {
      bc::float32_t t_heading_norm_raw_mean_f = 0.0;
      bc::uint8_t t_num_cross_stop_line_u8 = 0;
      if (!iter_ptr->cross_stop_line_pack_vt_.empty())
      {
        for (auto iter_line_ptr = iter_ptr->cross_stop_line_pack_vt_.begin();
             iter_line_ptr != iter_ptr->cross_stop_line_pack_vt_.end(); iter_line_ptr++)
        {
          bc::float32_t t_heading_norm_f = iter_line_ptr->heading_f_;
          if ((t_heading_norm_raw_mean_f - t_heading_norm_f) > G_PI_2)
          {
            t_heading_norm_f = t_heading_norm_f + G_PI;
          }
          else if ((t_heading_norm_raw_mean_f - t_heading_norm_f) < -G_PI_2)
          {
            t_heading_norm_f = t_heading_norm_f - G_PI;
          }
          t_heading_norm_raw_mean_f = t_heading_norm_raw_mean_f * t_num_cross_stop_line_u8 + t_heading_norm_f;
          t_num_cross_stop_line_u8++;
          t_heading_norm_raw_mean_f = t_heading_norm_raw_mean_f / ((bc::float32_t)(t_num_cross_stop_line_u8));
        }
      }
      /** 1. update dx in cluster */
      if (iter_ptr->new_create_b_ == bc::true_v)
      {
        /** new cluster, dx_tracking_f_ directly equals to dx_mean_f_.
         * life time accumulated.
         */
        iter_ptr->dx_tracking_f_ = iter_ptr->dx_mean_f_;
        iter_ptr->life_time_f_ = 0.3;
      }
      else
      {
        /** if the cluster contains any cross/stop line in current cycle, update dx_tracking_f_ by filtering dx_mean_f_
         * and
         * accumulate life time. Otherwise keep dx_tracking_f_ and drease life time. dx_tracking_f_ will be updated at
         * the beginning*/
        if (t_num_cross_stop_line_u8 > 0)
        {
          if (v_ego_f_ > 0.5)
          {
            bc::float32_t t_k_f =
                LinearInterpolation(0.3, 0.8, 3.0, 0.99, iter_ptr->life_time_f_, Lin_Interp_Method::kFlat);
            iter_ptr->dx_tracking_f_ = t_k_f * iter_ptr->dx_tracking_f_ + (1 - t_k_f) * iter_ptr->dx_mean_f_;
          }
          else
          {
            /** very low speed, do nothing, keep dx_tracking_f_ as pure tracking value*/
          }

          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ + time_cycle_f_;
        }
        else
        {
          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ - time_cycle_f_;
        }
      }

      /** 2. update dy equals to the road width at the position dx = dx_tracking_f_ */
      bc::float32_t t_dy_mean_f = 0.0;
      bc::float32_t t_length_side_mean_f = 0.0;
      bc::float32_t t_heading_mean_f = 0.0;
      if ((left_bound_cs_.existence_ == bc::true_v) && (right_bound_cs_.existence_ == bc::true_v))
      {
        bc::float32_t t_dx_f = iter_ptr->dx_tracking_f_;
        bc::float32_t t_dy_left_f = 0.0;
        bc::float32_t t_dy_right_f = 0.0;
        bc::float32_t t_heading_left_f = 0.0;
        bc::float32_t t_heading_right_f = 0.0;

        if ((t_dx_f > left_bound_cs_.pts_[0].x) && (t_dx_f < left_bound_cs_.pts_[kMaxBoundaryPoint - 1].x))
        {
          for (bc::uint16_t t_idx_u16 = 1; t_idx_u16 < kMaxBoundaryPoint; t_idx_u16++)
          {
            if ((t_dx_f > left_bound_cs_.pts_[t_idx_u16 - 1].x) && (t_dx_f < left_bound_cs_.pts_[t_idx_u16].x))
            {
              t_dy_left_f = LinearInterpolation(left_bound_cs_.pts_[t_idx_u16 - 1].x,
                                                left_bound_cs_.pts_[t_idx_u16 - 1].y, left_bound_cs_.pts_[t_idx_u16].x,
                                                left_bound_cs_.pts_[t_idx_u16].y, t_dx_f, Lin_Interp_Method::kFlat);
              t_dy_right_f =
                  LinearInterpolation(right_bound_cs_.pts_[t_idx_u16 - 1].x, right_bound_cs_.pts_[t_idx_u16 - 1].y,
                                      right_bound_cs_.pts_[t_idx_u16].x, right_bound_cs_.pts_[t_idx_u16].y, t_dx_f,
                                      Lin_Interp_Method::kFlat);
              t_heading_left_f = (left_bound_cs_.pts_[t_idx_u16].y - left_bound_cs_.pts_[t_idx_u16 - 1].y) /
                                 (left_bound_cs_.pts_[t_idx_u16].x - left_bound_cs_.pts_[t_idx_u16 - 1].x);
              t_heading_left_f = atan(t_heading_left_f);
              t_heading_right_f = (right_bound_cs_.pts_[t_idx_u16].y - right_bound_cs_.pts_[t_idx_u16 - 1].y) /
                                  (right_bound_cs_.pts_[t_idx_u16].x - right_bound_cs_.pts_[t_idx_u16 - 1].x);
              t_heading_right_f = atan(t_heading_right_f);

              t_dy_mean_f = 0.5 * (t_dy_left_f + t_dy_right_f);
              t_length_side_mean_f = fabs(t_dy_left_f - t_dy_right_f);
              t_length_side_mean_f = std::max(t_length_side_mean_f, 7.0f);
              t_heading_mean_f = 0.5 * (t_heading_left_f + t_heading_right_f);
              break;
            }
          }
        }
        else
        {
          t_dy_mean_f = 0.0;
          t_length_side_mean_f = 3.5;
          t_heading_mean_f = 0.0;
        }
      }
      else
      {
        t_dy_mean_f = 0.0;
        t_length_side_mean_f = 3.5;
        t_heading_mean_f = 0.0;
      }

      if ((left_bound_cs_.source_type_ == 4) && (right_bound_cs_.source_type_ == 4))
      {
        t_heading_mean_f = t_heading_norm_raw_mean_f;
      }

      if (iter_ptr->new_create_b_ == bc::true_v)
      {
        iter_ptr->dy_mean_f_ = t_dy_mean_f;
        iter_ptr->length_side_mean_f_ = t_length_side_mean_f;
        iter_ptr->heading_mean_f_ = t_heading_mean_f;
      }
      else
      {
        if (v_ego_f_ > 0.5)
        {
          bc::float32_t t_k_f =
              LinearInterpolation(0.3, 0.9, 2.0, 0.99, iter_ptr->life_time_f_, Lin_Interp_Method::kFlat);
          // iter_ptr->dy_mean_f_ = t_k_f * iter_ptr->dy_mean_f_ + (1 - t_k_f) * t_dy_mean_f;
          iter_ptr->dy_mean_f_ = t_dy_mean_f;
          iter_ptr->length_side_mean_f_ = t_k_f * iter_ptr->length_side_mean_f_ + (1 - t_k_f) * t_length_side_mean_f;
          // iter_ptr->heading_mean_f_ = t_k_f * iter_ptr->heading_mean_f_ + (1 - t_k_f) * t_heading_mean_f;
          iter_ptr->heading_mean_f_ = t_heading_mean_f;
        }
        else
        {
        }
      }
      HeadingRangeNormalize(iter_ptr->heading_mean_f_);

      if (!iter_ptr->cross_stop_line_pack_vt_.empty())
      {
        std::sort(iter_ptr->cross_stop_line_pack_vt_.begin(), iter_ptr->cross_stop_line_pack_vt_.end(),
                  CrossStopLineLifeTimeCompare);
      }
      /** iter_ptr accumulated. */
      iter_ptr++;
    }
  }
}

void StaticObjProcess::CrossStopLineDistillation()
{
  cross_stop_line_vt_.clear();
  for (auto iter_cluster_ptr = cross_stop_line_cluster_vt_.begin();
       iter_cluster_ptr != cross_stop_line_cluster_vt_.end(); iter_cluster_ptr++)
  {
    bc::float32_t t_dx_clustered_f = iter_cluster_ptr->dx_tracking_f_;
    bc::float32_t t_dy_clustered_f = iter_cluster_ptr->dy_mean_f_;
    bc::float32_t t_length_side_clustered_f = iter_cluster_ptr->length_side_mean_f_;
    bc::float32_t t_heading_clustered_f = iter_cluster_ptr->heading_mean_f_;
    EmCrossStopLineType t_type_clustered_en = iter_cluster_ptr->cluster_type_en_;

    if ((!iter_cluster_ptr->cross_stop_line_pack_vt_.empty()) && (t_type_clustered_en != EmCrossStopLineType::kUnknown))
    {
      bc::uint8_t t_idx_u8 = iter_cluster_ptr->cross_stop_line_pack_vt_[0].idx_u8_;
      bc::int32_t t_id_s32 = iter_cluster_ptr->cross_stop_line_pack_vt_[0].id_s32_;

      CrossStopLineInfo t_cross_stop_line_cs = CrossStopLineInfo();

      if (iter_cluster_ptr->life_time_f_ > 0.1)
      {
        t_cross_stop_line_cs.idx_u8_ = t_idx_u8;
        t_cross_stop_line_cs.id_s32_ = t_id_s32;
        t_cross_stop_line_cs.dx_f_ = t_dx_clustered_f;
        t_cross_stop_line_cs.dy_f_ = t_dy_clustered_f;
        t_cross_stop_line_cs.length_side_f_ = t_length_side_clustered_f;
        t_cross_stop_line_cs.heading_f_ = t_heading_clustered_f;
        t_cross_stop_line_cs.type_en_ = t_type_clustered_en;
        t_cross_stop_line_cs.updated_b_ = bc::true_v;
        t_cross_stop_line_cs.life_time_f_ = iter_cluster_ptr->life_time_f_;

        switch (t_type_clustered_en)
        {
          case EmCrossStopLineType::kStopLine:
          default:
            t_cross_stop_line_cs.width_side_f_ = 0.2;
            break;
          case EmCrossStopLineType::kCrossLine:
            t_cross_stop_line_cs.width_side_f_ = 3.5;
            break;
          case EmCrossStopLineType::kCrossAndStopLine:
            t_cross_stop_line_cs.width_side_f_ = 4.0;
            break;
        }
        t_cross_stop_line_cs.distilled_b_ = bc::true_v;
        cross_stop_line_vt_.push_back(t_cross_stop_line_cs);
      }
    }
  }
}

void StaticObjProcess::CrossStopLineOutput()
{
  bc::uint8_t t_cnt_cross_stop_line_u8 = 0;
  if (!cross_stop_line_vt_.empty())
  {
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (t_cnt_cross_stop_line_u8 < cross_stop_line_vt_.size())
      {
        bc::int32_t t_type_s32 = static_obj_ar_[t_idx_u8].type_;
        bc::int32_t t_id_s32 = static_obj_ar_[t_idx_u8].id_;
        if ((t_type_s32 == 0) || (t_id_s32 == kInvalidStaticObjId))
        {
          EmStaticObject t_em_static_obj_cs = EmStaticObject();
          t_em_static_obj_cs.id_ = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].id_s32_;

          switch (cross_stop_line_vt_[t_cnt_cross_stop_line_u8].type_en_)
          {
            case EmCrossStopLineType::kUnknown:
            default:
              t_em_static_obj_cs.type_ = 0;
              break;
            case EmCrossStopLineType::kCrossLine:
              t_em_static_obj_cs.type_ = 8;
              break;
            case EmCrossStopLineType::kStopLine:
              t_em_static_obj_cs.type_ = 5;
              break;
            case EmCrossStopLineType::kCrossAndStopLine:
              t_em_static_obj_cs.type_ = 100;
              break;
          }

          t_em_static_obj_cs.position_.x = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].dx_f_;
          t_em_static_obj_cs.position_.y = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].dy_f_;
          t_em_static_obj_cs.length_ = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].length_side_f_;
          t_em_static_obj_cs.width_ = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].width_side_f_;
          t_em_static_obj_cs.heading_ = cross_stop_line_vt_[t_cnt_cross_stop_line_u8].heading_f_;

          static_obj_ar_[t_idx_u8] = t_em_static_obj_cs;
          t_cnt_cross_stop_line_u8++;
        }
      }
      else
      {
        break;
      }
    }

    bc::uint8_t t_count_cross_stop_line_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxCrossStopLineNum; t_idx_u8++)
    {
      if (t_count_cross_stop_line_u8 < cross_stop_line_vt_.size())
      {
        EmCrossStopLine& t_em_cross_stop_line_cs = em_collection_ptr_->static_object_.cross_stop_line_ar_[t_idx_u8];
        t_em_cross_stop_line_cs.id_ = cross_stop_line_vt_[t_count_cross_stop_line_u8].id_s32_;
        t_em_cross_stop_line_cs.type_ = cross_stop_line_vt_[t_count_cross_stop_line_u8].type_en_;
        t_em_cross_stop_line_cs.position_.x = cross_stop_line_vt_[t_count_cross_stop_line_u8].dx_f_;
        t_em_cross_stop_line_cs.position_.y = cross_stop_line_vt_[t_count_cross_stop_line_u8].dy_f_;
        t_em_cross_stop_line_cs.length_ = cross_stop_line_vt_[t_count_cross_stop_line_u8].length_side_f_;
        t_em_cross_stop_line_cs.width_ = cross_stop_line_vt_[t_count_cross_stop_line_u8].width_side_f_;
        t_em_cross_stop_line_cs.heading_ = cross_stop_line_vt_[t_count_cross_stop_line_u8].heading_f_;
        t_em_cross_stop_line_cs.valid_ = bc::true_v;

        cross_stop_line_output_vt_.push_back(cross_stop_line_vt_[t_count_cross_stop_line_u8]);
        t_count_cross_stop_line_u8++;
      }
      else
      {
        break;
      }
    }

    for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
    {
      if (cross_stop_line_output_vt_.size() < kMaxStaticObjNum)
      {
        if ((cross_stop_line_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
            (cross_stop_line_ar_[t_idx_obj_u8].type_en_ != EmCrossStopLineType::kUnknown) &&
            (cross_stop_line_ar_[t_idx_obj_u8].dx_f_ > -20.0))
        {
          cross_stop_line_output_vt_.push_back(cross_stop_line_ar_[t_idx_obj_u8]);
        }
      }
      else
      {
        break;
      }
    }
  }
}

EmCrossStopLineType StaticObjProcess::CrossStopLineTypeMapping(const bc::int32_t type_horizon_s32)
{
  EmCrossStopLineType t_type_en = EmCrossStopLineType::kUnknown;

  if (type_horizon_s32 == 5)
  {
    t_type_en = EmCrossStopLineType::kStopLine;
  }
  else if (type_horizon_s32 == 8)
  {
    t_type_en = EmCrossStopLineType::kCrossLine;
  }
  else
  {
    t_type_en = EmCrossStopLineType::kUnknown;
  }
  return (t_type_en);
}

void StaticObjProcess::TrafficLightStablization(
    const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
    const StaticObjectData& per_static_obj_cs,
    const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar))
{
  if (is_in_map_b_ == bc::true_v)
  {
    /** map available */
    traffic_light_enable_b_ = bc::false_v;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLanePropertySegsInOneLaneElement; t_idx_u8++)
    {
      RefLineSegIsInIntersection& t_its_cs =
          em_collection_ptr_->lanes_[2].lane_elements_[0].reference_line_.is_in_intersection_segs_[t_idx_u8];
      if ((t_its_cs.valid_ == bc::true_v) && (t_its_cs.start_s_ < 100.0) && (t_its_cs.end_s_ > 10.0))
      {
        traffic_light_enable_b_ = bc::true_v;
        break;
      }
    }
  }
  else
  {
    /** only perception */
    if (traffic_light_enable_b_ == bc::false_v)
    {
      bc::bool_t t_left_bound_enable_b =
          (left_bound_cs_.existence_ == bc::true_v) &&
          ((GetBitU8(left_bound_cs_.source_type_, LaneBoundary::kSrcMaskPerception) == bc::true_v) ||
           (GetBitU8(left_bound_cs_.source_type_, LaneBoundary::kSrcMaskMap) == bc::true_v));
      bc::bool_t t_right_bound_enable_b =
          (right_bound_cs_.existence_ == bc::true_v) &&
          ((GetBitU8(right_bound_cs_.source_type_, LaneBoundary::kSrcMaskPerception) == bc::true_v) ||
           (GetBitU8(right_bound_cs_.source_type_, LaneBoundary::kSrcMaskMap) == bc::true_v));

      if ((t_left_bound_enable_b == bc::true_v) || (t_right_bound_enable_b == bc::true_v))
      {
        d_after_intersection_f_ = d_after_intersection_f_ + v_ego_f_ * time_cycle_f_;
      }
      else
      {
        d_after_intersection_f_ = 0.0;
      }

      if (((t_left_bound_enable_b == bc::true_v) || (t_right_bound_enable_b == bc::true_v)) &&
          (d_after_intersection_f_ > 10.0))
      {
        traffic_light_enable_b_ = bc::true_v;
      }
    }
    else
    {
      bc::bool_t t_left_bound_loss_b = (left_bound_cs_.existence_ == bc::false_v) || (left_bound_cs_.source_type_ == 4);
      bc::bool_t t_right_bound_loss_b =
          (right_bound_cs_.existence_ == bc::false_v) || (right_bound_cs_.source_type_ == 4);
      if ((t_left_bound_loss_b == bc::true_v) || (t_right_bound_loss_b == bc::true_v))
      {
        traffic_light_enable_b_ = bc::false_v;
        d_after_intersection_f_ = 0.0;
      }
    }
  }

  TrackingTrafficLightArrayAndCluster(ego_motion_cs);
  UpdateTrafficLightArray(static_obj_diag_cs, per_static_obj_cs);
  TrafficLightRoadAssignment(map_traffic_light_ar);
  TrafficLightArray2Vector();
  TrafficLightClustering();
  UpdateTrafficLightClusterList();
  TrafficLightDistillation();
  TrafficLightOutput();
}

void StaticObjProcess::TrackingTrafficLightArrayAndCluster(const EgoPoseCollection& ego_motion_cs)
{
  TransForm t_transform_st = ego_motion_cs.GetCurrentEgoPoseReadOnly().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::float32_t t_dx_trans_f = t_transform_st.translation_dx_f_;
  bc::float32_t t_dy_trans_f = t_transform_st.translation_dy_f_;
  bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f),
                                                                   -sinf(t_rotation_f), cosf(t_rotation_f)};

  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((traffic_light_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (!traffic_light_ar_[t_idx_obj_u8].type_status_pair_vt_.empty()))
    {
      /** TODO: update dx,dy by ego motion*/

      traffic_light_ar_[t_idx_obj_u8].dx_f_ = traffic_light_ar_[t_idx_obj_u8].dx_f_ - t_dx_trans_f;
      traffic_light_ar_[t_idx_obj_u8].dy_f_ = traffic_light_ar_[t_idx_obj_u8].dy_f_ - t_dy_trans_f;

      bc::float32_t t_dx_f = t_trans_matrix_ar[0] * traffic_light_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[1] * traffic_light_ar_[t_idx_obj_u8].dy_f_;
      bc::float32_t t_dy_f = t_trans_matrix_ar[2] * traffic_light_ar_[t_idx_obj_u8].dx_f_ +
                             t_trans_matrix_ar[3] * traffic_light_ar_[t_idx_obj_u8].dy_f_;

      traffic_light_ar_[t_idx_obj_u8].dx_f_ = t_dx_f;
      traffic_light_ar_[t_idx_obj_u8].dy_f_ = t_dy_f;
      traffic_light_ar_[t_idx_obj_u8].updated_b_ = bc::false_v;
      traffic_light_ar_[t_idx_obj_u8].in_lanes_b_ = bc::false_v;
    }
  }

  if (traffic_light_cluster_vt_.size() > 0)
  {
    bc::float32_t t_dx_tracking_f = traffic_light_cluster_vt_[0].dx_tracking_f_;
    if ((v_ego_f_ > 4.2) && (t_dx_tracking_f < 39.0))
    {
      t_dx_tracking_f = t_dx_tracking_f + 0.01;
    }
    for (bc::uint8_t t_idx_cluster_u8 = 0; t_idx_cluster_u8 < traffic_light_cluster_vt_.size(); t_idx_cluster_u8++)
    {
      traffic_light_cluster_vt_[t_idx_cluster_u8].traffic_light_pack_vt_.clear();
      // cross_stop_line_cluster_vt_[t_idx_cluster_u8].cluster_type_u8_ = 0;
      traffic_light_cluster_vt_[t_idx_cluster_u8].num_traffic_light_ = 0;
      traffic_light_cluster_vt_[t_idx_cluster_u8].new_create_b_ = bc::false_v;
      traffic_light_cluster_vt_[t_idx_cluster_u8].dx_mean_f_ = 0.0;

      /** TODO: update dx,dy by ego motion*/
      traffic_light_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ =
          traffic_light_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ - t_dx_trans_f;
      traffic_light_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ =
          traffic_light_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ - t_dy_trans_f;
      bc::float32_t t_dx_f = t_trans_matrix_ar[0] * traffic_light_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ +
                             t_trans_matrix_ar[1] * traffic_light_cluster_vt_[t_idx_cluster_u8].dy_mean_f_;
      bc::float32_t t_dy_f = t_trans_matrix_ar[2] * traffic_light_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ +
                             t_trans_matrix_ar[3] * traffic_light_cluster_vt_[t_idx_cluster_u8].dy_mean_f_;
      traffic_light_cluster_vt_[t_idx_cluster_u8].dx_tracking_f_ = t_dx_f;
      traffic_light_cluster_vt_[t_idx_cluster_u8].dy_mean_f_ = t_dy_f;
    }
  }
}

void StaticObjProcess::UpdateTrafficLightArray(const InputDiagInfo& static_obj_diag_cs,
                                               const StaticObjectData& per_static_obj_cs)
{
  if (static_obj_diag_cs.timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      static_obj_diag_cs.status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  { /** only process when status is not error confirmed. */

    /** 1. create a list, indicating input static objs are matched with last cycle. */
    bc::TCArray<bc::bool_t, kMaxStaticObjNum> t_input_match_list_ar;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      t_input_match_list_ar[t_idx_u8] = bc::false_v;
    }

    /** 2. loop traffic_light_ar_ in last cycle, the last cycle traffic light and the current input obj are matched
     * if
     * a) same type
     * b) same id
     */
    /** NOTICE: traffic_light_ar_ now stores info from last cycle, so temp variables below are named by "lst1" */
    for (bc::uint8_t t_idx_lst_u8 = 0; t_idx_lst_u8 < kMaxStaticObjNum; t_idx_lst_u8++)
    {
      /** check if last cycle traffic light is valid */
      bc::bool_t t_valid_b = !traffic_light_ar_[t_idx_lst_u8].type_status_pair_vt_.empty();
      bc::int32_t t_id_lst1_s32 = traffic_light_ar_[t_idx_lst_u8].id_s32_;
      if ((t_valid_b == bc::true_v) && (t_id_lst1_s32 != kInvalidStaticObjId))
      {
        /** if last cycle traffic light is valid, loop input obj list and try to match an input obj. */
        for (bc::uint8_t t_idx_new_u8 = 0; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** check if input obj is valid */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_type_new_s32 == 2) && (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** two objs are matched if their ids are the same.
             * the type in traffic_light_ar_ means light type, but the type in static obj means traffic light. They are
             * not the same meaning, so should not compare their types directly.
             */
            if ((t_id_lst1_s32 == t_id_new_s32))
            {
              /** set a flag in match list, and set input obj into the internal obj array with the old index. */
              t_input_match_list_ar[t_idx_new_u8] = bc::true_v;
              bc::bool_t t_new_cross_stop_b = bc::false_v;
              SetTrafficLightInput(per_static_obj_cs.objects_[t_idx_new_u8], t_new_cross_stop_b,
                                   traffic_light_ar_[t_idx_lst_u8]);
              traffic_light_ar_[t_idx_lst_u8].idx_u8_ = t_idx_lst_u8;
            }
          }
        }
      }
    }

    /** 3. delete traffic light in traffic_light_ar_ if its update is false and life time is exhausted.*/
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (((!traffic_light_ar_[t_idx_u8].type_status_pair_vt_.empty()) &&
           (traffic_light_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId) &&
           (traffic_light_ar_[t_idx_u8].updated_b_ == bc::false_v) &&
           (traffic_light_ar_[t_idx_u8].life_time_f_ < time_cycle_f_)) ||
          (traffic_light_ar_[t_idx_u8].dx_f_ < -20.0))
      {
        traffic_light_ar_[t_idx_u8] = TrafficLightInfo();
      }
    }

    /** 4. after above loop, all matched traffic lights are stored into the traffic_light_ar_ with the old index.
     * Below code is to insert remaining new traffic lights into absent position in traffic_light_ar_.
     */
    bc::uint8_t t_idx_start_u8 = 0;
    /** loop internal obj array */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      /** find absent position */
      bc::bool_t t_valid_b = !traffic_light_ar_[t_idx_u8].type_status_pair_vt_.empty();
      bc::int32_t t_id_s32 = traffic_light_ar_[t_idx_u8].id_s32_;
      if ((t_valid_b == bc::false_v) || (t_id_s32 == kInvalidStaticObjId))
      {
        /** loop input obj array */
        for (bc::uint8_t t_idx_new_u8 = t_idx_start_u8; t_idx_new_u8 < kMaxStaticObjNum; t_idx_new_u8++)
        {
          /** find valid static obj which has not been matched with any old static objs */
          bc::int32_t t_type_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].type_;
          bc::int32_t t_id_new_s32 = per_static_obj_cs.objects_[t_idx_new_u8].id_;
          if ((t_input_match_list_ar[t_idx_new_u8] == bc::false_v) && (t_type_new_s32 == 2) &&
              (t_id_new_s32 != kInvalidStaticObjId))
          {
            /** set input obj into the absent position and update t_idx_start_u8 */
            bc::bool_t t_new_cross_stop_b = bc::true_v;
            SetTrafficLightInput(per_static_obj_cs.objects_[t_idx_new_u8], t_new_cross_stop_b,
                                 traffic_light_ar_[t_idx_u8]);
            traffic_light_ar_[t_idx_u8].idx_u8_ = t_idx_u8;
            t_idx_start_u8 = t_idx_new_u8 + 1;
            break;
          }
        }
      }
      if (t_idx_start_u8 > kMaxStaticObjNum)
      {
        break;
      }
    }

    /** 5. update life time according to update_b flag. */
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if ((!traffic_light_ar_[t_idx_u8].type_status_pair_vt_.empty()) &&
          (traffic_light_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId))
      {
        if (traffic_light_ar_[t_idx_u8].updated_b_ == bc::true_v)
        {
          traffic_light_ar_[t_idx_u8].life_time_f_ = traffic_light_ar_[t_idx_u8].life_time_f_ + time_cycle_f_;
        }
        else
        {
          traffic_light_ar_[t_idx_u8].life_time_f_ = std::min(traffic_light_ar_[t_idx_u8].life_time_f_, 1.0f);
          traffic_light_ar_[t_idx_u8].life_time_f_ = traffic_light_ar_[t_idx_u8].life_time_f_ - time_cycle_f_;
        }
      }
    }
  }
  else
  { /** input invalid, reset timestamp. static obj array keep default value. */
  }
}

void StaticObjProcess::SetTrafficLightInput(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b,
                                            TrafficLightInfo& obj_out_cs)
{

  obj_out_cs.id_s32_ = obj_in_cs.id_;

  obj_out_cs.distilled_b_ = bc::false_v;
  /** currently life_time is still from last cycle. */
  if (new_obj_b == bc::true_v)
  {
    obj_out_cs.dx_f_ = obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = obj_in_cs.position_.y_;
    obj_out_cs.life_time_f_ = 0.3;
  }
  else
  {
    bc::float32_t t_k_f = LinearInterpolation(0.3, 0.9, 2.0, 0.95, obj_out_cs.life_time_f_, Lin_Interp_Method::kFlat);
    obj_out_cs.dx_f_ = t_k_f * obj_out_cs.dx_f_ + (1 - t_k_f) * obj_in_cs.position_.x_;
    obj_out_cs.dy_f_ = t_k_f * obj_out_cs.dy_f_ + (1 - t_k_f) * obj_in_cs.position_.y_;
  }

  bc::uint8_t t_num_child_types_u8 = 0;
  for (bc::uint8_t t_idx_child_types_u8 = 0; t_idx_child_types_u8 < 40; t_idx_child_types_u8++)
  {
    if (obj_in_cs.child_types_[t_idx_child_types_u8] > 0)
    {
      t_num_child_types_u8++;
    }
  }

  /** clear type_status_pair before push back. */

  bc::TFixedVector<bc::TPair<bc::uint16_t, EmTrafficLightColor>, 20> t_type_status_pair_vt;
  t_type_status_pair_vt.clear();

  if ((t_num_child_types_u8 > 0) && ((t_num_child_types_u8 % 2) == 0))
  {
    bc::uint8_t t_num_bulb_u8 = std::min(20, t_num_child_types_u8 / 2);

    for (bc::uint8_t t_idx_bulb_u8 = 0; t_idx_bulb_u8 < t_num_bulb_u8; t_idx_bulb_u8++)
    {
      bc::int32_t t_child_type_s32 = obj_in_cs.child_types_[t_idx_bulb_u8];

      if (t_child_type_s32 > 0)
      {

        bc::int32_t t_color_horizon_s32 = obj_in_cs.child_types_[t_num_bulb_u8 + t_idx_bulb_u8];
        bc::TPair<bc::uint16_t, EmTrafficLightColor> t_tuple =
            TrafficLightTypeMapping(t_child_type_s32, t_color_horizon_s32);
        if (t_tuple.first() != 0)
        {
          t_type_status_pair_vt.push_back(t_tuple);
        }
      }
    }
  }
  if (!t_type_status_pair_vt.empty())
  {
    obj_out_cs.type_status_pair_vt_.clear();
    obj_out_cs.updated_b_ = bc::true_v;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < t_type_status_pair_vt.size(); t_idx_u8++)
    {
      obj_out_cs.type_status_pair_vt_.push_back(t_type_status_pair_vt[t_idx_u8]);
    }
  }
  else
  {
    obj_out_cs.updated_b_ = bc::false_v;
  }
}
void StaticObjProcess::TrafficLightRoadAssignment(
    const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar))
{

  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((traffic_light_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (!traffic_light_ar_[t_idx_obj_u8].type_status_pair_vt_.empty()))
    {
      bc::bool_t t_match_with_map_b;
      bc::bool_t t_in_lanes_b = bc::false_v;
      bc::bool_t t_up_or_down_arrow_b = bc::false_v;
      for (auto iter_ptr = traffic_light_ar_[t_idx_obj_u8].type_status_pair_vt_.begin();
           iter_ptr != traffic_light_ar_[t_idx_obj_u8].type_status_pair_vt_.end(); iter_ptr++)
      {
        if ((GetBitU16(iter_ptr->first(), static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUpArrowMask)) ==
             bc::true_v) ||
            (GetBitU16(iter_ptr->first(), static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kDownArrowMask)) ==
             bc::true_v))
        {
          t_up_or_down_arrow_b = bc::true_v;
          break;
        }
      }

      if (traffic_light_enable_b_ == bc::true_v)
      {

        bc::float32_t t_dx_f = traffic_light_ar_[t_idx_obj_u8].dx_f_;
        bc::float32_t t_dy_f = traffic_light_ar_[t_idx_obj_u8].dy_f_;
        bc::float32_t t_left_bound_point_f = 0.0;
        bc::float32_t t_right_bound_point_f = 0.0;
        if ((t_dx_f > left_bound_cs_.pts_[0].x) && (t_dx_f < left_bound_cs_.pts_[kMaxBoundaryPoint - 1].x))
        {
          for (bc::uint16_t t_idx_u16 = 1; t_idx_u16 < kMaxBoundaryPoint; t_idx_u16++)
          {
            if ((t_dx_f > left_bound_cs_.pts_[t_idx_u16 - 1].x) && (t_dx_f < left_bound_cs_.pts_[t_idx_u16].x))
            {
              t_left_bound_point_f = LinearInterpolation(
                  left_bound_cs_.pts_[t_idx_u16 - 1].x, left_bound_cs_.pts_[t_idx_u16 - 1].y,
                  left_bound_cs_.pts_[t_idx_u16].x, left_bound_cs_.pts_[t_idx_u16].y, t_dx_f, Lin_Interp_Method::kFlat);
              t_right_bound_point_f =
                  LinearInterpolation(right_bound_cs_.pts_[t_idx_u16 - 1].x, right_bound_cs_.pts_[t_idx_u16 - 1].y,
                                      right_bound_cs_.pts_[t_idx_u16].x, right_bound_cs_.pts_[t_idx_u16].y, t_dx_f,
                                      Lin_Interp_Method::kFlat);
              break;
            }
          }
          bc::float32_t t_dy_offset_f = 5.0;
          if (t_up_or_down_arrow_b == bc::true_v)
          {
            t_dy_offset_f = 0.0;
          }

          t_left_bound_point_f = t_left_bound_point_f + t_dy_offset_f;
          t_right_bound_point_f = t_right_bound_point_f - t_dy_offset_f;

          if ((t_dy_f <= t_left_bound_point_f) && (t_dy_f >= t_right_bound_point_f))
          {
            t_in_lanes_b = bc::true_v;
          }
        }
        else
        {
          t_in_lanes_b = bc::false_v;
        }
      }
      else
      {
        t_in_lanes_b = bc::false_v;
      }

      if (is_in_map_b_ == bc::true_v)
      {
        t_match_with_map_b = MatchPerTrafficLightWithMap(traffic_light_ar_[t_idx_obj_u8], map_traffic_light_ar);
      }
      else
      {
        t_match_with_map_b = bc::false_v;
      }

      traffic_light_ar_[t_idx_obj_u8].in_lanes_b_ = (t_in_lanes_b || t_match_with_map_b);
    }
  }
}
bc::bool_t StaticObjProcess::MatchPerTrafficLightWithMap(
    const TrafficLightInfo per_traffic_light_cs,
    const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar))
{

  bc::float32_t t_dx_diff_abs_f = 0.0;
  bc::float32_t t_dy_diff_abs_f = 0.0;
  bc::bool_t t_is_matched_b = bc::false_v;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxTrafficLightNum; t_idx_u8++)
  {
    if (map_traffic_light_ar[t_idx_u8].valid_ == bc::true_v)
    {
      t_dx_diff_abs_f = std::abs(per_traffic_light_cs.dx_f_ - map_traffic_light_ar[t_idx_u8].position_.x);
      t_dy_diff_abs_f = std::abs(per_traffic_light_cs.dy_f_ - map_traffic_light_ar[t_idx_u8].position_.y);
      if ((t_dx_diff_abs_f < 20.0) && (t_dy_diff_abs_f < 3.5))
      {
        t_is_matched_b = bc::true_v;
        break;
      }
    }
  }
  return (t_is_matched_b);
}

void StaticObjProcess::TrafficLightArray2Vector()
{
  /** only store traffic light in lanes into vector. */
  for (bc::uint8_t t_idx_obj_u8 = 0; t_idx_obj_u8 < kMaxStaticObjNum; t_idx_obj_u8++)
  {
    if ((traffic_light_ar_[t_idx_obj_u8].id_s32_ != kInvalidStaticObjId) &&
        (!traffic_light_ar_[t_idx_obj_u8].type_status_pair_vt_.empty()) &&
        (traffic_light_ar_[t_idx_obj_u8].in_lanes_b_ == bc::true_v) &&
        (traffic_light_ar_[t_idx_obj_u8].dx_f_ > -10.0) && (traffic_light_ar_[t_idx_obj_u8].dx_f_ < 100.0))
    {
      traffic_light_vt_.push_back(traffic_light_ar_[t_idx_obj_u8]);
    }
  }
  /** sort by dx, in ascend order.*/
  if (!traffic_light_vt_.empty())
  {
    std::sort(traffic_light_vt_.begin(), traffic_light_vt_.end(), TrafficLightDxCompare);
  }
}
void StaticObjProcess::TrafficLightClustering()
{
  for (bc::uint8_t t_cnt_light_u8 = 0; t_cnt_light_u8 < traffic_light_vt_.size(); t_cnt_light_u8++)
  {
    /** if there is no any cluster, directly create a new cluster by current index. */
    if (traffic_light_cluster_vt_.empty() == bc::true_v)
    {
      CreateTrafficLightCluster(traffic_light_vt_[t_cnt_light_u8]);
    }
    else
    {
      /** there is at least one existing cluster. Let's loop all cluster.*/
      bc::bool_t t_cluster_found_b = bc::false_v;
      for (bc::uint8_t t_cnt_clst_u8 = 0; t_cnt_clst_u8 < traffic_light_cluster_vt_.size(); t_cnt_clst_u8++)
      {
        /** if the dx is close to the mean value of this cluster, add obj into this cluster. */

        // bc::float32_t t_dx_diff_f =
        //     traffic_light_vt_[t_cnt_light_u8].dx_f_ - traffic_light_cluster_vt_[t_cnt_clst_u8].dx_tracking_f_;
        // if (fabs(t_dx_diff_f) < 20.0)

        /** traffic light might be exist at both the entry and exit of intersection, so use dx instead of dx_diff to
         * select valid traffic lights*/
        if (traffic_light_vt_[t_cnt_light_u8].dx_f_ < 100.0)
        {
          InsertTrafficLightToCluster(traffic_light_vt_[t_cnt_light_u8], traffic_light_cluster_vt_[t_cnt_clst_u8]);
          t_cluster_found_b = bc::true_v;
          break;
        }
      }
      if ((t_cluster_found_b == bc::false_v) && (traffic_light_cluster_vt_.size() < kMaxClusterNumTrafficLight))
      {
        CreateTrafficLightCluster(traffic_light_vt_[t_cnt_light_u8]);
      }
    }
  }
}
void StaticObjProcess::CreateTrafficLightCluster(const TrafficLightInfo& traffic_light_cs)
{
  TrafficLightCluster t_cluster_cs = TrafficLightCluster();

  /** dx_tracking_f_ and life_time_f_ will be calculated later. */
  t_cluster_cs.traffic_light_pack_vt_.push_back(traffic_light_cs);
  t_cluster_cs.dx_mean_f_ = traffic_light_cs.dx_f_;
  t_cluster_cs.num_traffic_light_ = 1;
  t_cluster_cs.new_create_b_ = bc::true_v;

  traffic_light_cluster_vt_.push_back(t_cluster_cs);
}

void StaticObjProcess::InsertTrafficLightToCluster(const TrafficLightInfo& traffic_light_cs,
                                                   TrafficLightCluster& traffic_light_cluster_cs)
{
  bc::bool_t t_insert_b = bc::false_v;
  if (traffic_light_cluster_cs.traffic_light_pack_vt_.size() < kMaxNumTrafficLightPack)
  {
    traffic_light_cluster_cs.traffic_light_pack_vt_.push_back(traffic_light_cs);
    t_insert_b = bc::true_v;
  }
  else
  {
    std::sort(traffic_light_cluster_cs.traffic_light_pack_vt_.begin(),
              traffic_light_cluster_cs.traffic_light_pack_vt_.end(), TrafficLightLifeTimeCompare);
    if (traffic_light_cluster_cs.traffic_light_pack_vt_.back().updated_b_ == bc::false_v)
    {
      traffic_light_cluster_cs.traffic_light_pack_vt_.pop_back();
      traffic_light_cluster_cs.traffic_light_pack_vt_.push_back(traffic_light_cs);
      t_insert_b = bc::true_v;
    }
    else
    {
      t_insert_b = bc::false_v;
    }
  }

  if (t_insert_b == bc::true_v)
  {
    traffic_light_cluster_cs.dx_mean_f_ =
        traffic_light_cluster_cs.dx_mean_f_ * traffic_light_cluster_cs.num_traffic_light_ + traffic_light_cs.dx_f_;
    traffic_light_cluster_cs.dx_mean_f_ =
        traffic_light_cluster_cs.dx_mean_f_ / (traffic_light_cluster_cs.num_traffic_light_ + 1);
    traffic_light_cluster_cs.num_traffic_light_++;
  }
  else
  {
    /** this line has not been inserted into pack, do nothing. */
  }
}

void StaticObjProcess::UpdateTrafficLightClusterList()
{
  for (auto iter_ptr = traffic_light_cluster_vt_.begin(); iter_ptr != traffic_light_cluster_vt_.end();)
  {
    /** delete empty cluster if it contains no traffic light and its life time exhausted. */
    bc::bool_t t_del_old_cluster_b =
        (iter_ptr->new_create_b_ == bc::false_v) &&
        (((iter_ptr->num_traffic_light_ == 0) && (iter_ptr->life_time_f_ < time_cycle_f_)) ||
         (iter_ptr->dx_tracking_f_ < -10.0));
    bc::bool_t t_del_new_cluster_b = (iter_ptr->new_create_b_ == bc::true_v) && (iter_ptr->dx_mean_f_ < 0.0);

    if (t_del_old_cluster_b || t_del_new_cluster_b)
    {
      /** iter_ptr automatically increase by the return of erase method. */
      iter_ptr = traffic_light_cluster_vt_.erase(iter_ptr);
    }
    else
    {
      /** 1. update dx in cluster */
      if (iter_ptr->new_create_b_ == bc::true_v)
      {
        /** new cluster, dx_tracking_f_ directly equals to dx_mean_f_.
         * life time accumulated.
         */
        iter_ptr->dx_tracking_f_ = iter_ptr->dx_mean_f_;
        iter_ptr->life_time_f_ = 0.3;
      }
      else
      {
        /** if the cluster contains any traffic light in current cycle, update dx_tracking_f_ by filtering dx_mean_f_
         * and
         * accumulate life time. Otherwise keep dx_tracking_f_ and drease life time. dx_tracking_f_ will be updated at
         * the beginning*/
        if (iter_ptr->num_traffic_light_ > 0)
        {
          if (v_ego_f_ > 0.5)
          {
            bc::float32_t t_k_f =
                LinearInterpolation(0.3, 0.8, 3.0, 0.95, iter_ptr->life_time_f_, Lin_Interp_Method::kFlat);
            iter_ptr->dx_tracking_f_ = t_k_f * iter_ptr->dx_tracking_f_ + (1 - t_k_f) * iter_ptr->dx_mean_f_;
          }
          else
          {
            /** very low speed, do nothing, keep dx_tracking_f_ as pure tracking value*/
          }

          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ + time_cycle_f_;
        }
        else
        {
          iter_ptr->life_time_f_ = iter_ptr->life_time_f_ - time_cycle_f_;
        }
      }

      /** 2. update dy equals 0. */
      iter_ptr->dy_mean_f_ = 0.0;

      if (!iter_ptr->traffic_light_pack_vt_.empty())
      {
        std::sort(iter_ptr->traffic_light_pack_vt_.begin(), iter_ptr->traffic_light_pack_vt_.end(),
                  TrafficLightLifeTimeCompare);
      }
      /** iter_ptr accumulated. */
      iter_ptr++;
    }
  }

  if (!traffic_light_cluster_vt_.empty())
  {
    std::sort(traffic_light_cluster_vt_.begin(), traffic_light_cluster_vt_.end(), TrafficLightClusterDxCompare);
  }
}

void StaticObjProcess::TrafficLightDistillation()
{
  traffic_light_vt_.clear();

  /** if any traffic light cluster is available, only check the first one because the cluster vector is already sorted
   * by dx_tracking_f_. */
  if (!traffic_light_cluster_vt_.empty())
  {
    bc::bool_t t_cluster_found_b = bc::false_v;
    for (auto iter_cluster_ptr = traffic_light_cluster_vt_.begin(); iter_cluster_ptr != traffic_light_cluster_vt_.end();
         iter_cluster_ptr++)
    {
      bc::float32_t t_dx_cluster_f = iter_cluster_ptr->dx_tracking_f_;
      if (!iter_cluster_ptr->traffic_light_pack_vt_.empty())
      {
        for (auto iter_light_ptr = iter_cluster_ptr->traffic_light_pack_vt_.begin();
             iter_light_ptr != iter_cluster_ptr->traffic_light_pack_vt_.end(); iter_light_ptr++)
        {
          if (!iter_light_ptr->type_status_pair_vt_.empty())
          {
            bc::int32_t t_light_id_s32 = iter_light_ptr->id_s32_;
            for (auto iter_type_ptr = iter_light_ptr->type_status_pair_vt_.begin();
                 iter_type_ptr != iter_light_ptr->type_status_pair_vt_.end(); iter_type_ptr++)
            {
              /** bc::TPair<BulbTypeMask, TrafficLightStatus>, first() returns type and second() returns light
               * status(color). */

              bc::uint16_t t_light_type_u16 = iter_type_ptr->first();
              EmTrafficLightColor t_light_color_en = iter_type_ptr->second();

              /** if light type contains vehicle light, then this cluster is the final cluster we loop. other
               * clusters with larger dx will not be checked.*/

              bc::bool_t t_straight_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask));
              bc::bool_t t_turn_left_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask));
              bc::bool_t t_turn_right_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kRightArrowMask));
              bc::bool_t t_up_arrow_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUpArrowMask));
              bc::bool_t t_down_arrow_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kDownArrowMask));
              bc::bool_t t_uturn_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUturnMask));
              bc::bool_t t_ped_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kPedMask));
              bc::bool_t t_cyc_b =
                  GetBitU16(t_light_type_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCycMask));

              /** mapping */
              if (t_ped_b == bc::true_v)
              {
                traffic_light_semantic_type_ar_[kTrafficLightSemanticPed].first() = t_light_id_s32;
                traffic_light_semantic_type_ar_[kTrafficLightSemanticPed].second() = t_light_color_en;
              }
              else if (t_cyc_b == bc::true_v)
              {
                traffic_light_semantic_type_ar_[kTrafficLightSemanticCyc].first() = t_light_id_s32;
                traffic_light_semantic_type_ar_[kTrafficLightSemanticCyc].second() = t_light_color_en;
              }
              else
              {
                t_cluster_found_b = bc::true_v;
                if (t_straight_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticStraight, t_light_id_s32, t_light_color_en);
                }
                if (t_turn_left_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticLeft, t_light_id_s32, t_light_color_en);
                }
                if (t_turn_right_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticRight, t_light_id_s32, t_light_color_en);
                }
                if (t_up_arrow_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticStraight, t_light_id_s32, t_light_color_en);
                }
                if (t_down_arrow_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticStraight, t_light_id_s32, t_light_color_en);
                }
                if (t_uturn_b == bc::true_v)
                {
                  TrafficLightSemanticUpdate(kTrafficLightSemanticUturn, t_light_id_s32, t_light_color_en);
                }
              }
            }
          }
        }

        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kNumTrafficLightSemantic; t_idx_u8++)
        {
          if (traffic_light_semantic_type_ar_[t_idx_u8].second() != EmTrafficLightColor::kUnknown)
          {
            TrafficLightInfo t_traffic_light_cs = TrafficLightInfo();
            t_traffic_light_cs.id_s32_ = traffic_light_semantic_type_ar_[t_idx_u8].first();
            t_traffic_light_cs.dx_f_ = t_dx_cluster_f;
            t_traffic_light_cs.dy_f_ = traffic_light_dy_ar_[t_idx_u8];

            bc::uint16_t t_type_u16 = 0;
            switch (t_idx_u8)
            {
              case kTrafficLightSemanticStraight:
                SetBitU16(static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask), bc::true_v, t_type_u16);
                break;
              case kTrafficLightSemanticLeft:
                SetBitU16(static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask), bc::true_v, t_type_u16);
                break;
              case kTrafficLightSemanticRight:
                SetBitU16(static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kRightArrowMask), bc::true_v, t_type_u16);
                break;
              case kTrafficLightSemanticUturn:
                SetBitU16(static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUturnMask), bc::true_v, t_type_u16);
                break;
              default:
                t_type_u16 = 0;
                break;
            }
            if (t_type_u16 != 0)
            {
              bc::TPair<bc::uint16_t, EmTrafficLightColor> t_pair(t_type_u16,
                                                                  traffic_light_semantic_type_ar_[t_idx_u8].second());
              t_traffic_light_cs.type_status_pair_vt_.push_back(t_pair);
              t_traffic_light_cs.distilled_b_ = bc::true_v;
              traffic_light_vt_.push_back(t_traffic_light_cs);
            }
          }
        }
      }

      if (t_cluster_found_b == bc::true_v)
      {
        break;
      }
    }
  }
}

void StaticObjProcess::TrafficLightSemanticUpdate(const bc::uint8_t traffic_light_semantic,
                                                  const bc::int32_t id_input_s32,
                                                  const EmTrafficLightColor color_input_en)
{
  bc::int32_t& t_current_id_s32 = traffic_light_semantic_type_ar_[traffic_light_semantic].first();
  EmTrafficLightColor& t_current_color_en = traffic_light_semantic_type_ar_[traffic_light_semantic].second();
  if (static_cast<bc::uint8_t>(color_input_en) > static_cast<bc::uint8_t>(t_current_color_en))
  {
    t_current_id_s32 = id_input_s32;
    t_current_color_en = color_input_en;
  }
}

void StaticObjProcess::TrafficLightOutput()
{
  if (!traffic_light_vt_.empty())
  {
    bc::uint8_t t_cnt_traffic_light_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
    {
      if (t_cnt_traffic_light_u8 < traffic_light_vt_.size())
      {
        bc::int32_t t_type_s32 = static_obj_ar_[t_idx_u8].type_;
        bc::int32_t t_id_s32 = static_obj_ar_[t_idx_u8].id_;
        if ((t_type_s32 == 0) || (t_id_s32 == kInvalidStaticObjId))
        {
          EmStaticObject t_em_static_obj_cs = EmStaticObject();

          t_em_static_obj_cs.id_ = traffic_light_vt_[t_cnt_traffic_light_u8].id_s32_;
          t_em_static_obj_cs.type_ = 2;
          /** fix position x at 10m */
          t_em_static_obj_cs.position_.x = 10.0;
          t_em_static_obj_cs.position_.y = traffic_light_vt_[t_cnt_traffic_light_u8].dy_f_;
          t_em_static_obj_cs.position_.z = 5.5;

          bc::uint16_t t_type_em_u16 = traffic_light_vt_[t_cnt_traffic_light_u8].type_status_pair_vt_[0].first();
          bc::uint32_t t_type_horizon_u32 = 0;
          if (GetBitU16(t_type_em_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kCircleMask)))
          {
            t_type_horizon_u32 = 1;
          }
          else if (GetBitU16(t_type_em_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kLeftArrowMask)))
          {
            t_type_horizon_u32 = 2;
          }
          else if (GetBitU16(t_type_em_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kRightArrowMask)))
          {
            t_type_horizon_u32 = 4;
          }
          else if (GetBitU16(t_type_em_u16, static_cast<bc::uint8_t>(EmTrafficLightTypeMsk::kUturnMask)))
          {
            t_type_horizon_u32 = 32;
          }
          else
          {
            t_type_horizon_u32 = 0;
          }
          t_em_static_obj_cs.child_types_[0] = (bc::int32_t)(t_type_horizon_u32);

          bc::int32_t t_status_s32 = 0;
          switch (traffic_light_vt_[t_cnt_traffic_light_u8].type_status_pair_vt_[0].second())
          {
            case EmTrafficLightColor::kGreen:
              t_status_s32 = 10;
              break;
            case EmTrafficLightColor::kYellow:
              t_status_s32 = 11;
              break;
            case EmTrafficLightColor::kRed:
              t_status_s32 = 12;
              break;
            default:
              t_status_s32 = 0;
              break;
          }
          t_em_static_obj_cs.child_types_[1] = t_status_s32;

          static_obj_ar_[t_idx_u8] = t_em_static_obj_cs;
          t_cnt_traffic_light_u8++;
        }
      }
      else
      {
        break;
      }
    }

    bc::uint8_t t_count_traffic_light_u8 = 0;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxTrafficLightNum; t_idx_u8++)
    {
      if (t_count_traffic_light_u8 < traffic_light_vt_.size())
      {
        EmTrafficLight& t_em_traffic_light_cs = em_collection_ptr_->static_object_.traffic_light_ar_[t_idx_u8];
        t_em_traffic_light_cs.id_ = traffic_light_vt_[t_count_traffic_light_u8].id_s32_;
        t_em_traffic_light_cs.type_ = traffic_light_vt_[t_count_traffic_light_u8].type_status_pair_vt_[0].first();
        t_em_traffic_light_cs.color_ = traffic_light_vt_[t_count_traffic_light_u8].type_status_pair_vt_[0].second();
        t_em_traffic_light_cs.countdown_ = 0;
        t_em_traffic_light_cs.countdown_valid_ = bc::false_v;

        // TrafficLightTypeMapping(t_count_traffic_light_u8, t_idx_u8);
        t_em_traffic_light_cs.position_.x = 10.0;
        t_em_traffic_light_cs.position_.y = traffic_light_vt_[t_count_traffic_light_u8].dy_f_;
        t_em_traffic_light_cs.position_.z = 5.5;
        t_em_traffic_light_cs.mode_ = EmTrafficLightMode::kContinuous;  // reserved
        t_em_traffic_light_cs.heading_ = 0.0;                           // reserved
        t_em_traffic_light_cs.valid_ = bc::true_v;

        traffic_light_output_vt_.push_back(traffic_light_vt_[t_count_traffic_light_u8]);
        t_count_traffic_light_u8++;
      }
      else
      {
        break;
      }
    }
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxStaticObjNum; t_idx_u8++)
  {
    if (traffic_light_output_vt_.size() < kMaxStaticObjNum)
    {
      if ((!traffic_light_ar_[t_idx_u8].type_status_pair_vt_.empty()) &&
          (traffic_light_ar_[t_idx_u8].id_s32_ != kInvalidStaticObjId) && (traffic_light_ar_[t_idx_u8].dx_f_ > -10.0))
      {
        traffic_light_output_vt_.push_back(traffic_light_ar_[t_idx_u8]);
      }
    }
    else
    {
      break;
    }
  }
}

bc::TPair<bc::uint16_t, EmTrafficLightColor> StaticObjProcess::TrafficLightTypeMapping(
    const bc::int32_t type_horizon_s32, const bc::int32_t color_horizon_s32)
{
  bc::int32_t t_type_horizon_s32 = type_horizon_s32;
  bc::uint16_t t_type_internal_u16 = 0;
  EmTrafficLightColor t_color_internal_en = EmTrafficLightColor::kUnknown;
  bc::TPair<bc::uint16_t, EmTrafficLightColor> t_tuple(t_type_internal_u16, t_color_internal_en);

  for (bc::uint8_t t_bit_u8 = 0; t_bit_u8 < static_cast<bc::uint8_t>(BulbTypeMask::kNumBulbTypeMask); t_bit_u8++)
  {
    if (t_type_horizon_s32 == 1)
    {
      BulbTypeMask t_type_horizon_en = bulb_type_mask_ar_[t_bit_u8];

      for (auto iter_map_ptr = traffic_light_type_map_.lower_bound(t_type_horizon_en);
           iter_map_ptr != traffic_light_type_map_.upper_bound(t_type_horizon_en); iter_map_ptr++)
      {
        SetBitU16(iter_map_ptr->second, bc::true_v, t_type_internal_u16);
      }

      switch (color_horizon_s32)
      {
        case 10:
          t_color_internal_en = EmTrafficLightColor::kGreen;
          break;
        case 11:
          t_color_internal_en = EmTrafficLightColor::kYellow;
          break;
        case 12:
          t_color_internal_en = EmTrafficLightColor::kRed;
          break;
        default:
          t_color_internal_en = EmTrafficLightColor::kUnknown;
          break;
      }

      t_tuple.first() = t_type_internal_u16;
      t_tuple.second() = t_color_internal_en;

      break;
    }
    else
    {
      t_type_horizon_s32 = t_type_horizon_s32 >> 1;
    }
  }

  return (t_tuple);
}

void StaticObjProcess::SetOutput()
{
  /* output to EM output class    */
  em_collection_ptr_->static_object_.timestamp_s64_ = timestamp_s64_;
  em_collection_ptr_->static_object_.objects_ = static_obj_ar_;
}

bc::bool_t StaticObjProcess::FindNearestRefIdx(const ReferenceLine& ref_line_cs, const bc::float32_t x_f,
                                               bc::uint16_t& nearest_idx_u16)
{
  bc::bool_t t_find_b = bc::false_v;
  if (ref_line_cs.available_ == bc::true_v)
  {
    if (x_f < ref_line_cs.ref_line_pts_[0].pos_.x)
    {
      t_find_b = bc::true_v;
      nearest_idx_u16 = 0;
    }
    else if (x_f > ref_line_cs.ref_line_pts_[kMaxRefLinePtsNum - 1].pos_.x)
    {
      t_find_b = bc::true_v;
      nearest_idx_u16 = kMaxRefLinePtsNum - 1;
    }
    else
    {
      for (bc::uint16_t t_idx_u16 = 1; t_idx_u16 < kMaxRefLinePtsNum; t_idx_u16++)
      {
        if ((x_f > ref_line_cs.ref_line_pts_[t_idx_u16 - 1].pos_.x) &&
            (x_f < ref_line_cs.ref_line_pts_[t_idx_u16].pos_.x))
        {
          bc::float32_t t_dis_lower_f = fabs(x_f - ref_line_cs.ref_line_pts_[t_idx_u16 - 1].pos_.x);
          bc::float32_t t_dis_upper_f = fabs(x_f - ref_line_cs.ref_line_pts_[t_idx_u16].pos_.x);

          t_find_b = bc::true_v;
          if (t_dis_lower_f <= t_dis_upper_f)
          {
            nearest_idx_u16 = t_idx_u16 - 1;
          }
          else
          {
            nearest_idx_u16 = t_idx_u16;
          }
          break;
        }
      }
    }
  }
  return (t_find_b);
}

bc::float64_t StaticObjProcess::Distance(const StPoint p1, const StPoint p2)
{
  return sqrt((p1.x_ - p2.x_) * (p1.x_ - p2.x_) + (p1.y_ - p2.y_) * (p1.y_ - p2.y_));
}

void StaticObjProcess::CalcLengthWidthHeading(const StaticObject& obj_cs, bc::float32_t& length_f,
                                              bc::float32_t& width_f, bc::float32_t& heading_f)
{
  bc::float64_t t_x1_f64 = 0.0;
  bc::float64_t t_y1_f64 = 0.0;
  bc::float64_t t_x2_f64 = 0.0;
  bc::float64_t t_y2_f64 = 0.0;
  bc::float64_t t_heading1_f64 = 0.0;
  bc::float64_t t_heading2_f64 = 0.0;
  // 当point[0]和point[1]的连线是长边时
  if (Distance(obj_cs.border_.points_[0], obj_cs.border_.points_[1]) >
      Distance(obj_cs.border_.points_[1], obj_cs.border_.points_[2]))
  {
    length_f = (Distance(obj_cs.border_.points_[0], obj_cs.border_.points_[1]) +
                Distance(obj_cs.border_.points_[2], obj_cs.border_.points_[3])) *
               0.50;
    width_f = (Distance(obj_cs.border_.points_[1], obj_cs.border_.points_[2]) +
               Distance(obj_cs.border_.points_[0], obj_cs.border_.points_[3])) *
              0.50;
    // 求取值范围在0到pi之间的最长边的法向量
    // point[0]和point[1]连接而成的长边的法向量
    if (obj_cs.border_.points_[0].x_ > obj_cs.border_.points_[1].x_)
    {
      t_x1_f64 = obj_cs.border_.points_[1].y_ - obj_cs.border_.points_[0].y_;
      t_y1_f64 = obj_cs.border_.points_[0].x_ - obj_cs.border_.points_[1].x_;
    }
    else
    {
      t_x1_f64 = obj_cs.border_.points_[0].y_ - obj_cs.border_.points_[1].y_;
      t_y1_f64 = obj_cs.border_.points_[1].x_ - obj_cs.border_.points_[0].x_;
    }
    // point[2]和point[3]连接而成的长边的法向量
    if (obj_cs.border_.points_[2].x_ > obj_cs.border_.points_[3].x_)
    {
      t_x2_f64 = obj_cs.border_.points_[3].y_ - obj_cs.border_.points_[2].y_;
      t_y2_f64 = obj_cs.border_.points_[2].x_ - obj_cs.border_.points_[3].x_;
    }
    else
    {
      t_x2_f64 = obj_cs.border_.points_[2].y_ - obj_cs.border_.points_[3].y_;
      t_y2_f64 = obj_cs.border_.points_[3].x_ - obj_cs.border_.points_[2].x_;
    }
    t_heading1_f64 = acos(t_x1_f64 / sqrt(t_x1_f64 * t_x1_f64 + t_y1_f64 * t_y1_f64));
    t_heading2_f64 = acos(t_x2_f64 / sqrt(t_x2_f64 * t_x2_f64 + t_y2_f64 * t_y2_f64));
    heading_f = (t_heading1_f64 + t_heading2_f64) / 2;
  }
  // 当point[0]和point[3]的连线是长边时
  else
  {
    length_f = (Distance(obj_cs.border_.points_[1], obj_cs.border_.points_[2]) +
                Distance(obj_cs.border_.points_[0], obj_cs.border_.points_[3])) *
               0.50;
    width_f = (Distance(obj_cs.border_.points_[0], obj_cs.border_.points_[1]) +
               Distance(obj_cs.border_.points_[2], obj_cs.border_.points_[3])) *
              0.50;
    // 求取值范围在0到pi之间的最长边的法向量
    // point[0]和point[3]连接而成的长边的法向量
    if (obj_cs.border_.points_[0].x_ > obj_cs.border_.points_[3].x_)
    {
      t_x1_f64 = obj_cs.border_.points_[3].y_ - obj_cs.border_.points_[0].y_;
      t_y1_f64 = obj_cs.border_.points_[0].x_ - obj_cs.border_.points_[3].x_;
    }
    else
    {
      t_x1_f64 = obj_cs.border_.points_[0].y_ - obj_cs.border_.points_[3].y_;
      t_y1_f64 = obj_cs.border_.points_[3].x_ - obj_cs.border_.points_[0].x_;
    }
    // point[1]和point[2]连接而成的长边的法向量
    if (obj_cs.border_.points_[2].x_ > obj_cs.border_.points_[1].x_)
    {
      t_x2_f64 = obj_cs.border_.points_[1].y_ - obj_cs.border_.points_[2].y_;
      t_y2_f64 = obj_cs.border_.points_[2].x_ - obj_cs.border_.points_[1].x_;
    }
    else
    {
      t_x2_f64 = obj_cs.border_.points_[2].y_ - obj_cs.border_.points_[1].y_;
      t_y2_f64 = obj_cs.border_.points_[1].x_ - obj_cs.border_.points_[2].x_;
    }
    t_heading1_f64 = acos(t_x1_f64 / sqrt(t_x1_f64 * t_x1_f64 + t_y1_f64 * t_y1_f64));
    t_heading2_f64 = acos(t_x2_f64 / sqrt(t_x2_f64 * t_x2_f64 + t_y2_f64 * t_y2_f64));
    heading_f = (t_heading1_f64 + t_heading2_f64) / 2;
  }

  HeadingRangeNormalize(heading_f);
}

void StaticObjProcess::HeadingRangeNormalize(bc::float32_t& heading_f)
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
