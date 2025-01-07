
#include "hlmf.h"

namespace zone {
namespace environment_model {

Hlmf::Hlmf(EmData& em_collection, EmData& em_collection_lst_cycle)
    : em_collection_cs_(&em_collection), em_collection_lst_cycle_(&em_collection_lst_cycle)
{
}
void Hlmf::Run(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_, const BoundaryPack& boundary_pack,
               const WeightPack& weight_pack, const SemanticPack& semantic_pack_cs, StEndIdxPack& last_em_st_end_idx_,
               EgoPoseCollection& ego_motion_cs, const EmData& em_output_lst_cycle_cs_, const EmParam& param_st_,
               const bc::float32_t time_cycle_f)
{
  /** 1 lane boundary geometry*/
  in_map_b = semantic_pack_cs.map_geofence_.is_in_map_;
  map_exist_b = boundary_pack.lane_pack_ar_[kHostLane].element_pack_ar_[0].left_pack_cs_.map_cs_.valid_b_;
  per_id_counter_ = 0;
  LaneGeometryFusion(x_ref_ar_, boundary_pack, weight_pack, last_em_st_end_idx_, ego_motion_cs);
  ReflineFusion(last_em_st_end_idx_, em_output_lst_cycle_cs_, ego_motion_cs, param_st_, time_cycle_f);
  /** ２. semantic info set */
  SemanticInfoSet(semantic_pack_cs);
}

void Hlmf::LaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                              const BoundaryPack& boundary_pack, const WeightPack& weight_pack,
                              StEndIdxPack& last_em_st_end_idx_, EgoPoseCollection& ego_motion_cs)
{
  /** according to source validity to set em output's lane validity*/
  SetValidity(boundary_pack);
  /** host lane's boundary must be valid*/
  if (em_collection_cs_->lanes_[kHostLane].lane_valid_)
  {
    HostLaneGeometryFusion(x_ref_ar_, boundary_pack.lane_pack_ar_[kHostLane], weight_pack.lane_weight_ar_[kHostLane],
                           em_collection_cs_->lanes_[kHostLane], last_em_st_end_idx_.lane_st_end_idx_[kHostLane]);
  }
  else
  {
    HostLaneProcess(x_ref_ar_, ego_motion_cs, last_em_st_end_idx_.lane_st_end_idx_[kHostLane]);
    em_collection_cs_->lanes_[kHostLane].lane_valid_ = bc::false_v;
    em_collection_cs_->lanes_[kRightLane].lane_valid_ = bc::false_v;
    em_collection_cs_->lanes_[kLeftLane].lane_valid_ = bc::false_v;
    em_collection_cs_->lanes_[kRRLane].lane_valid_ = bc::false_v;
    em_collection_cs_->lanes_[kLLLane].lane_valid_ = bc::false_v;
  }
  if (em_collection_cs_->lanes_[kRightLane].lane_valid_)
  {
    NeighborLaneGeometryFusion(x_ref_ar_, boundary_pack.lane_pack_ar_[kRightLane],
                               boundary_pack.lane_pack_ar_[kHostLane], weight_pack.lane_weight_ar_[kRightLane],
                               kRightBoundary, em_collection_cs_->lanes_[kHostLane],
                               last_em_st_end_idx_.lane_st_end_idx_[kHostLane], host_right_heading_ar,
                               em_collection_cs_->lanes_[kRightLane], last_em_st_end_idx_.lane_st_end_idx_[kRightLane]);
  }
  if (em_collection_cs_->lanes_[kLeftLane].lane_valid_)
  {
    NeighborLaneGeometryFusion(x_ref_ar_, boundary_pack.lane_pack_ar_[kLeftLane],
                               boundary_pack.lane_pack_ar_[kHostLane], weight_pack.lane_weight_ar_[kLeftLane],
                               kLeftBoundary, em_collection_cs_->lanes_[kHostLane],
                               last_em_st_end_idx_.lane_st_end_idx_[kHostLane], host_left_heading_ar,
                               em_collection_cs_->lanes_[kLeftLane], last_em_st_end_idx_.lane_st_end_idx_[kLeftLane]);
  }
  if (em_collection_cs_->lanes_[kRightLane].lane_valid_ && em_collection_cs_->lanes_[kRRLane].lane_valid_)
  {
    NeighborLaneGeometryFusion(x_ref_ar_, boundary_pack.lane_pack_ar_[kRRLane], boundary_pack.lane_pack_ar_[kRightLane],
                               weight_pack.lane_weight_ar_[kRRLane], kRightBoundary,
                               em_collection_cs_->lanes_[kRightLane], last_em_st_end_idx_.lane_st_end_idx_[kRightLane],
                               host_right_heading_ar, em_collection_cs_->lanes_[kRRLane],
                               last_em_st_end_idx_.lane_st_end_idx_[kRRLane]);
  }
  else
  {
    em_collection_cs_->lanes_[kRRLane] = LaneData();
  }
  if (em_collection_cs_->lanes_[kLeftLane].lane_valid_ && em_collection_cs_->lanes_[kLLLane].lane_valid_)
  {
    NeighborLaneGeometryFusion(x_ref_ar_, boundary_pack.lane_pack_ar_[kLLLane], boundary_pack.lane_pack_ar_[kLeftLane],
                               weight_pack.lane_weight_ar_[kLLLane], kLeftBoundary,
                               em_collection_cs_->lanes_[kLeftLane], last_em_st_end_idx_.lane_st_end_idx_[kLeftLane],
                               host_left_heading_ar, em_collection_cs_->lanes_[kLLLane],
                               last_em_st_end_idx_.lane_st_end_idx_[kLLLane]);
  }
  else
  {
    em_collection_cs_->lanes_[kLLLane] = LaneData();
  }
  ///@todo CheckLaneCross need?
  CheckLaneCross(em_collection_cs_->lanes_, last_em_st_end_idx_);
}
void Hlmf::CheckLaneCross(bc::TCArray<LaneData, kMaxLaneNum>& lanes, StEndIdxPack& last_em_st_end_idx_)
{
  bc::uint8_t t_start_idx = 0;
  bc::uint8_t t_end_idx = 0;
  bc::uint8_t t_check_start_idx = 0;
  bc::uint8_t t_check_end_idx = 0;
  bc::float32_t t_check_start_x_f = 0.f;
  bc::float32_t t_check_end_x_f = 0.f;

  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (lanes[t_lane_idx_u8].lane_valid_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (lanes[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_valid_)
        {
          LaneBoundary& left = lanes[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].left_boundary_;
          LaneBoundary& right = lanes[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].right_boundary_;
          ElementStEndIdxPack& ele_st_end_idx_pack =
              last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8].ele_st_end_idx_[t_ele_idx_u8];
          t_start_idx = left.start_idx_;
          t_end_idx = left.start_idx_ + left.total_valid_number_ - 1;
          t_check_start_x_f = ele_st_end_idx_pack.boundary_st_x_f_;
          t_check_end_x_f = ele_st_end_idx_pack.boundary_end_x_f_;
          t_check_start_idx = t_start_idx;
          t_check_end_idx = t_end_idx;
          bc::bool_t t_st_change_b = bc::false_v;
          bc::bool_t t_end_change_b = bc::false_v;
          for (bc::uint8_t t_pt_idx_u8 = t_start_idx; t_pt_idx_u8 <= t_end_idx; t_pt_idx_u8++)
          {
            if (left.pts_[t_pt_idx_u8].y >= right.pts_[t_pt_idx_u8].y)
            {
              t_check_start_idx = t_pt_idx_u8;
              t_check_start_x_f = left.pts_[t_pt_idx_u8].x;
              t_st_change_b = (t_start_idx == t_pt_idx_u8) ? static_cast<bc::bool_t>(bc::false_v)
                                                           : static_cast<bc::bool_t>(bc::true_v);
              break;
            }
          }
          for (bc::uint8_t t_pt_idx_u8 = t_check_start_idx; t_pt_idx_u8 <= t_end_idx; t_pt_idx_u8++)
          {
            if (left.pts_[t_pt_idx_u8].y < right.pts_[t_pt_idx_u8].y)
            {
              t_check_end_idx = t_pt_idx_u8;
              t_check_end_x_f = left.pts_[t_pt_idx_u8].x;
              t_end_change_b = bc::true_v;
              break;
            }
          }

          if (t_check_end_idx <= t_check_start_idx || t_check_end_x_f <= t_check_start_x_f)
          {
            lanes[t_lane_idx_u8].lane_elements_[t_ele_idx_u8] = LaneElement();
            lanes[t_lane_idx_u8].valid_elements_cnts_--;
            if (lanes[t_lane_idx_u8].valid_elements_cnts_ == 0)
            {
              lanes[t_lane_idx_u8] = LaneData();
            }
          }
          else
          {
            if (t_st_change_b)
            {
              ele_st_end_idx_pack.boundary_st_idx_ = t_check_start_idx;
              ele_st_end_idx_pack.boundary_st_x_f_ = t_check_start_x_f;
              right.start_idx_ = left.start_idx_ = right.reserve_[0] = left.reserve_[0] = t_check_start_idx;
            }
            if (t_end_change_b)
            {
              ele_st_end_idx_pack.boundary_end_idx_ = t_check_end_idx;
              ele_st_end_idx_pack.boundary_end_x_f_ = t_check_end_x_f;
            }
            right.total_valid_number_ = left.total_valid_number_ = t_check_end_idx - t_check_start_idx + 1;
          }
        }
      }
    }
  }
}

void Hlmf::ReflineFusion(const StEndIdxPack& last_em_st_end_idx_, const EmData& em_last_cycle_output,
                         EgoPoseCollection& ego_motion_cs, const EmParam& param_st_, const bc::float32_t time_cycle_f)
{
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (em_collection_cs_->lanes_[t_lane_idx_u8].lane_valid_)
    {
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (em_collection_cs_->lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8].element_valid_)
        {
          LaneElement& t_em_elem_cs = em_collection_cs_->lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8];
          bc::bool_t t_coeff_reliable_b = CenterLineGenerator(t_em_elem_cs.left_boundary_, t_em_elem_cs.right_boundary_,
                                                              t_em_elem_cs.reference_line_);
          if (t_em_elem_cs.reference_line_.ref_line_valid_pts_ >= 2)
          {
            ReflineFillIn(t_em_elem_cs.left_boundary_, t_em_elem_cs.right_boundary_,
                          last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8].ele_st_end_idx_[t_elem_idx_u8],
                          t_em_elem_cs.reference_line_, em_last_cycle_output, param_st_, time_cycle_f,
                          ego_motion_cs.GetCurrentEgoPose().GetLongVelocity(), t_lane_idx_u8, t_elem_idx_u8);
            bc::uint8_t t_start_idx_u8 = t_em_elem_cs.reference_line_.ref_start_idx_;
            bc::uint8_t t_end_idx_u8 =
                t_em_elem_cs.reference_line_.ref_start_idx_ + t_em_elem_cs.reference_line_.ref_line_valid_pts_ - 1;
            if (t_em_elem_cs.reference_line_.ref_line_valid_pts_ != 0 && t_end_idx_u8 < kMaxRefLinePtsNum &&
                t_em_elem_cs.reference_line_.ref_line_pts_[kFrontStartIdx].lane_width_ < 1.f &&
                t_em_elem_cs.reference_line_.ref_line_pts_[t_start_idx_u8].lane_width_ < 1.f &&
                t_em_elem_cs.reference_line_.ref_line_pts_[t_end_idx_u8].lane_width_ < 1.f &&
                t_em_elem_cs.reference_line_.ref_line_pts_[(t_start_idx_u8 + t_end_idx_u8) / 2].lane_width_ < 1.f)
            {
              if (t_lane_idx_u8 == kHostLane &&
                  t_em_elem_cs.reference_line_.ref_line_pts_[t_end_idx_u8].lane_width_ > FLOAT32_ZERO)
              {
                continue;
              }
              t_em_elem_cs = LaneElement();
              em_collection_cs_->lanes_[t_lane_idx_u8].valid_elements_cnts_--;
              if (em_collection_cs_->lanes_[t_lane_idx_u8].valid_elements_cnts_ == 0)
              {
                em_collection_cs_->lanes_[t_lane_idx_u8] = LaneData();
              }
            }
          }
        }
      }
    }
  }
}

void Hlmf::SetValidity(const BoundaryPack& boundary_pack)
{
  /**set em_out_data's validity*/
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    LaneData& cur_lane = em_collection_cs_->lanes_[t_lane_idx_u8];
    LanePack cur_packed_lane = boundary_pack.lane_pack_ar_[t_lane_idx_u8];
    cur_lane = LaneData();
    cur_lane.lane_valid_ = cur_packed_lane.valid_b_;
    cur_lane.valid_elements_cnts_ = 0;
    for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
    {
      LaneElement& cur_lane_elem = cur_lane.lane_elements_[t_elem_idx_u8];
      ElementPack cur_elem_pack = cur_packed_lane.element_pack_ar_[t_elem_idx_u8];
      cur_lane_elem.element_valid_ = cur_elem_pack.valid_b_;
      cur_lane_elem.left_boundary_.existence_ = cur_elem_pack.left_pack_cs_.valid_b_;
      cur_lane_elem.right_boundary_.existence_ = cur_elem_pack.right_pack_cs_.valid_b_;
      /**only if left and right boundary both valid, element valid */
      if (cur_lane_elem.element_valid_ &&
          !(cur_lane_elem.right_boundary_.existence_ && cur_lane_elem.left_boundary_.existence_))
      {
        cur_lane_elem.element_valid_ = bc::false_v;
      }
      if (cur_lane_elem.element_valid_)
      {
        cur_lane.valid_elements_cnts_ += 1;
        if (cur_lane.valid_elements_cnts_ >= 2)
        {
#ifdef STD_COUT_ENABLE
          std::cout << "Error in Hlmf::SetValidity: Lane elements num error!!!" << std::endl;
#endif
        }
      }
    }
    if (cur_lane.valid_elements_cnts_ == 0)
    {
      cur_lane.lane_valid_ = bc::false_v;
    }
  }
}

void Hlmf::HostLaneProcess(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar,
                           EgoPoseCollection& ego_motion_cs, LaneStEndIdxPack& lane_st_end_idx)
{
  // Generate host lane in case no other source valid
  if (em_collection_cs_->lanes_[kHostLane].lane_valid_ == bc::false_v)
  {
    bc::float32_t t_default_lane_width_f = 2.6f;
    bc::TCArray<bc::float32_t, 4> t_coeff_ar{0.f, 0.f, 0.f, 0.f};
    if (ego_motion_cs.GetCurrentEgoPose().GetValidation() == bc::true_v)
    {
      t_coeff_ar[2] = 0.5 * ego_motion_cs.GetCurrentEgoPose().GetCurvature();
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_ = 0;
      SetBitU8(LaneBoundary::kSrcMaskEgo, bc::true_v,
               em_collection_cs_->lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_);
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kVirtual;
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_ = 0;
      SetBitU8(LaneBoundary::kSrcMaskEgo, bc::true_v,
               em_collection_cs_->lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_);
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].right_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kVirtual;
    }
    else
    {
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_ = 0;
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kUnknown;
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_ = 0;
      em_collection_cs_->lanes_[kHostLane].lane_elements_[0].right_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kUnknown;
    }
    GenerateHostLane(x_ref_ar, t_default_lane_width_f, t_coeff_ar);
    lane_st_end_idx.ele_st_end_idx_[0].boundary_end_idx_ = kMaxBoundaryPoint - 1;
    lane_st_end_idx.ele_st_end_idx_[0].boundary_st_idx_ = 0;
    lane_st_end_idx.ele_st_end_idx_[0].boundary_end_x_f_ = x_ref_ar[kMaxBoundaryPoint - 1];
    lane_st_end_idx.ele_st_end_idx_[0].boundary_st_x_f_ = x_ref_ar[0];
  }
}

void Hlmf::GenerateHostLane(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar, bc::float32_t lane_width_f,
                            bc::TCArray<bc::float32_t, 4> coeff_ar)
{
  em_collection_cs_->lanes_[kHostLane].valid_elements_cnts_ = 1;
  LaneElement& t_host_element_cs = em_collection_cs_->lanes_[kHostLane].lane_elements_[0];
  t_host_element_cs.relative_idx_ = 0;
  t_host_element_cs.element_valid_ = bc::true_v;
  // Boundary
  for (bc::uint8_t i = 0; i < kMaxNumBoundary; ++i)
  {
    bc::float32_t current_x = x_ref_ar[i];
    bc::float32_t current_heading =
        std::atan(coeff_ar[1] + 2.f * coeff_ar[2] * current_x + 3.f * coeff_ar[3] * current_x * current_x);
    bc::float32_t current_curvature =
        2.f * coeff_ar[2] +
        6.f * coeff_ar[3] * current_x / std::pow(1 + std::tan(current_heading) * std::tan(current_heading), 1.5f);
    t_host_element_cs.left_boundary_.pts_[i].x = current_x - 0.5 * lane_width_f * sinf(current_heading);
    t_host_element_cs.left_boundary_.pts_[i].y = coeff_ar[0] + 0.5 * lane_width_f * cosf(current_heading) +
                                                 coeff_ar[1] * current_x + coeff_ar[2] * current_x * current_x +
                                                 coeff_ar[3] * current_x * current_x * current_x;
    t_host_element_cs.right_boundary_.pts_[i].x = current_x + 0.5 * lane_width_f * sinf(current_heading);
    t_host_element_cs.right_boundary_.pts_[i].y = coeff_ar[0] - 0.5 * lane_width_f * cosf(current_heading) +
                                                  coeff_ar[1] * current_x + coeff_ar[2] * current_x * current_x +
                                                  coeff_ar[3] * current_x * current_x * current_x;
  }
  t_host_element_cs.left_boundary_.existence_ = bc::true_v;
  t_host_element_cs.left_boundary_.total_valid_number_ = kMaxNumBoundary;
  // t_host_element_cs.left_boundary_.source_type_ = left.source_type_;
  t_host_element_cs.left_boundary_.ego_point_idx_ = 20;

  t_host_element_cs.left_boundary_.segs_[0].valid_ = bc::true_v;
  t_host_element_cs.left_boundary_.segs_[0].start_s_ = 0.f;
  t_host_element_cs.left_boundary_.segs_[0].end_s_ = 100.f;
  // t_host_element_cs.left_boundary_.segs_[0].boundary_type_ = left.type_;

  t_host_element_cs.right_boundary_.existence_ = bc::true_v;
  t_host_element_cs.right_boundary_.total_valid_number_ = kMaxNumBoundary;
  // t_host_element_cs.right_boundary_.source_type_ = right.source_type_;
  t_host_element_cs.right_boundary_.ego_point_idx_ = 20;

  t_host_element_cs.right_boundary_.segs_[0].valid_ = bc::true_v;
  t_host_element_cs.right_boundary_.segs_[0].start_s_ = 0.f;
  t_host_element_cs.right_boundary_.segs_[0].end_s_ = 100.f;
  // t_host_element_cs.right_boundary_.segs_[0].boundary_type_ = right.type_;
}

void Hlmf::HostLaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                                  const LanePack& lane_pack, const LaneWeight& lane_weight, LaneData& lane_data,
                                  LaneStEndIdxPack& lane_st_end_idx)
{
  for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
  {
    if (lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
    {
      BoundaryPointPack left_pack = lane_pack.element_pack_ar_[t_elem_idx_u8].left_pack_cs_;
      BoundaryPointPack right_pack = lane_pack.element_pack_ar_[t_elem_idx_u8].right_pack_cs_;
      ///@todo map start and end x need to be set in preprocess by real index rather than using data in segs
      bc::uint8_t t_l_p_end_u8 = left_pack.per_cs_.valid_b_ ? left_pack.per_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_l_r_end_u8 = left_pack.ref_cs_.valid_b_ ? left_pack.ref_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_l_m_end_u8 = left_pack.map_cs_.valid_b_ ? left_pack.map_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_l_l_end_u8 = left_pack.ldveh_cs_.valid_b_ ? left_pack.ldveh_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_r_p_end_u8 = right_pack.per_cs_.valid_b_ ? right_pack.per_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_r_r_end_u8 = right_pack.ref_cs_.valid_b_ ? right_pack.ref_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_r_m_end_u8 = right_pack.map_cs_.valid_b_ ? right_pack.map_cs_.dy_end_idx_u8_ : 0;
      bc::uint8_t t_r_l_end_u8 = right_pack.ldveh_cs_.valid_b_ ? right_pack.ldveh_cs_.dy_end_idx_u8_ : 0;

      bc::float32_t t_l_p_end_x_f = left_pack.per_cs_.valid_b_ ? left_pack.per_cs_.dx_end_f_ + 5.f : 0.f;
      bc::float32_t t_l_r_end_x_f = left_pack.ref_cs_.valid_b_ ? left_pack.ref_cs_.dx_end_f_ : 0.f;
      bc::float32_t t_l_m_end_x_f = left_pack.map_cs_.valid_b_ ? left_pack.map_cs_.dx_end_f_ + 5.f : 0.f;
      bc::float32_t t_l_l_end_x_f = left_pack.ldveh_cs_.valid_b_ ? left_pack.ldveh_cs_.dx_end_f_ : 0.f;
      bc::float32_t t_r_p_end_x_f = right_pack.per_cs_.valid_b_ ? right_pack.per_cs_.dx_end_f_ + 5.f : 0.f;
      bc::float32_t t_r_r_end_x_f = right_pack.ref_cs_.valid_b_ ? right_pack.ref_cs_.dx_end_f_ : 0.f;
      bc::float32_t t_r_m_end_x_f = right_pack.map_cs_.valid_b_ ? right_pack.map_cs_.dx_end_f_ + 5.f : 0.f;
      bc::float32_t t_r_l_end_x_f = right_pack.ldveh_cs_.valid_b_ ? right_pack.ldveh_cs_.dx_end_f_ : 0.f;

      /// Real end idx, pre for next cycle and rviz
      /// host end idx use max to promise the validity
      lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].boundary_end_idx_ =
          std::max(std::max({t_l_p_end_u8, t_l_r_end_u8, t_l_m_end_u8, t_l_l_end_u8}),
                   std::max({t_r_p_end_u8, t_r_r_end_u8, t_r_m_end_u8, t_r_l_end_u8}));
      lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].boundary_end_x_f_ =
          std::max(std::max({t_l_p_end_x_f, t_l_r_end_x_f, t_l_m_end_x_f, t_l_l_end_x_f}),
                   std::max({t_r_p_end_x_f, t_r_r_end_x_f, t_r_m_end_x_f, t_r_l_end_x_f}));
      bc::uint8_t t_l_p_start_u8 = left_pack.per_cs_.valid_b_ ? left_pack.per_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_l_r_start_u8 = left_pack.ref_cs_.valid_b_ ? left_pack.ref_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_l_m_start_u8 = left_pack.map_cs_.valid_b_ ? left_pack.map_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_l_l_start_u8 =
          left_pack.ldveh_cs_.valid_b_ ? left_pack.ldveh_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_r_p_start_u8 =
          right_pack.per_cs_.valid_b_ ? right_pack.per_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_r_r_start_u8 =
          right_pack.ref_cs_.valid_b_ ? right_pack.ref_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_r_m_start_u8 =
          right_pack.map_cs_.valid_b_ ? right_pack.map_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::uint8_t t_r_l_start_u8 =
          right_pack.ldveh_cs_.valid_b_ ? right_pack.ldveh_cs_.dy_start_idx_u8_ : kMaxBoundaryPoint;
      bc::float32_t t_l_p_start_x_f = left_pack.per_cs_.valid_b_ ? left_pack.per_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_l_r_start_x_f = left_pack.ref_cs_.valid_b_ ? left_pack.ref_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_l_m_start_x_f = left_pack.map_cs_.valid_b_ ? left_pack.map_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_l_l_start_x_f = left_pack.ldveh_cs_.valid_b_ ? left_pack.ldveh_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_r_p_start_x_f = right_pack.per_cs_.valid_b_ ? right_pack.per_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_r_r_start_x_f = right_pack.ref_cs_.valid_b_ ? right_pack.ref_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_r_m_start_x_f = right_pack.map_cs_.valid_b_ ? right_pack.map_cs_.dx_start_f_ : 100.f;
      bc::float32_t t_r_l_start_x_f = right_pack.ldveh_cs_.valid_b_ ? right_pack.ldveh_cs_.dx_start_f_ : 100.f;
      lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].boundary_st_idx_ =
          std::min(std::min({t_l_p_start_u8, t_l_r_start_u8, t_l_m_start_u8, t_l_l_start_u8}),
                   std::min({t_r_p_start_u8, t_r_r_start_u8, t_r_m_start_u8, t_r_l_start_u8}));
      lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].boundary_st_x_f_ =
          std::min(std::min({t_l_p_start_x_f, t_l_r_start_x_f, t_l_m_start_x_f, t_l_l_start_x_f}),
                   std::min({t_r_p_start_x_f, t_r_r_start_x_f, t_r_m_start_x_f, t_r_l_start_x_f}));
      bc::uint8_t t_min_start_idx = lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].boundary_st_idx_;
      if (t_min_start_idx < kMaxBoundaryPoint)
      {
        lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].datum_idx_ =
            t_min_start_idx > kFrontStartIdx ? t_min_start_idx : kFrontStartIdx;
      }
      else
      {
        lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8].datum_idx_ = kFrontStartIdx;
      }
      HostBoundaryFusion(x_ref_ar_, left_pack, lane_weight.element_weight_ar_[t_elem_idx_u8].left_weight_ar_,
                         lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                         lane_data.lane_elements_[t_elem_idx_u8].left_boundary_);
      HostBoundaryFusion(x_ref_ar_, right_pack, lane_weight.element_weight_ar_[t_elem_idx_u8].right_weight_ar_,
                         lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                         lane_data.lane_elements_[t_elem_idx_u8].right_boundary_);
    }
  }
}

void Hlmf::NeighborLaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                                      const LanePack& lane_pack_tar, const LanePack& lane_pack_ref,
                                      const LaneWeight& lane_weight, const bc::uint8_t dir_u8,
                                      const LaneData& ref_lane_data, const LaneStEndIdxPack& ref_lane_st_end_idx,
                                      const bc::TCArray<bc::float32_t, kMaxNumBoundary>& host_bd_heading_ar,
                                      LaneData& tar_lane_data, LaneStEndIdxPack& lane_st_end_idx)
{
  bc::bool_t t_has_split_with_neighbor_b = bc::false_v;
  /**  target lane is on the right of ref lane
   *eg: target lane is right lane, ref lane is host lane
   *find the nearest element of ref_lane */
  bc::uint8_t t_end_idx_u8 = kMaxBoundaryPoint - 1;
  bc::uint8_t t_start_idx_u8 = 0;
  if (dir_u8 == kRightBoundary)
  {
    bc::uint8_t t_nearest_elem_u8 = 0;
    bc::float32_t t_min_delta_dy_f = 1000;
    /** find the nearest element of ref_lane */
    /** end idx must be equal*/
    bc::uint8_t t_end_idx_0 = ref_lane_data.lane_elements_[0].right_boundary_.existence_
                                  ? ref_lane_data.lane_elements_[0].right_boundary_.total_valid_number_ - 1
                                  : kMaxBoundaryPoint;
    bc::uint8_t t_end_idx_1 = ref_lane_data.lane_elements_[1].right_boundary_.existence_
                                  ? ref_lane_data.lane_elements_[1].right_boundary_.total_valid_number_ - 1
                                  : kMaxBoundaryPoint;
    t_end_idx_u8 = std::min(t_end_idx_0, t_end_idx_1);
    if (t_end_idx_u8 >= kMaxBoundaryPoint)
    {
      tar_lane_data = LaneData();
#ifdef STD_COUT_ENABLE
      std::cout << "Error in Hlmf::NeighborLaneGeometryFusion: Left neighbour lane's ref lane invalid!" << std::endl;
#endif
    }
    else
    {
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (ref_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
        {
          bc::float32_t cur_delta_dy = ref_lane_data.lane_elements_[t_elem_idx_u8].right_boundary_.pts_[t_end_idx_u8].y;
          if (cur_delta_dy < t_min_delta_dy_f)
          {
            t_min_delta_dy_f = cur_delta_dy;
            t_nearest_elem_u8 = t_elem_idx_u8;
          }
        }
      }
      bc::bool_t t_has_split_with_neighbor_b =
          CheckSplit(lane_pack_tar.element_pack_ar_[0].left_pack_cs_,
                     lane_pack_ref.element_pack_ar_[t_nearest_elem_u8].right_pack_cs_);
      if (t_has_split_with_neighbor_b == bc::false_v)
      {
        tar_lane_data.lane_elements_[0].left_boundary_ =
            ref_lane_data.lane_elements_[t_nearest_elem_u8].right_boundary_;
        StoreParStartEndIdx(ref_lane_st_end_idx.ele_st_end_idx_[t_nearest_elem_u8],
                            lane_pack_tar.element_pack_ar_[0].left_pack_cs_,
                            lane_pack_tar.element_pack_ar_[0].right_pack_cs_, lane_st_end_idx.ele_st_end_idx_[0]);
        BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[0].right_pack_cs_,
                       lane_weight.element_weight_ar_[0].right_weight_ar_, lane_st_end_idx.ele_st_end_idx_[0],
                       tar_lane_data.lane_elements_[0].right_boundary_);
        tar_lane_data.lane_elements_[0].left_boundary_.start_idx_ =
            tar_lane_data.lane_elements_[0].right_boundary_.start_idx_;
        tar_lane_data.lane_elements_[0].left_boundary_.reserve_[0] =
            tar_lane_data.lane_elements_[0].right_boundary_.reserve_[0];
        tar_lane_data.lane_elements_[0].left_boundary_.total_valid_number_ =
            tar_lane_data.lane_elements_[0].right_boundary_.total_valid_number_;
        for (bc::uint8_t t_elem_idx_u8 = 1; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
        {
          if (tar_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
          {
            StoreStartEndIdx(lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                             lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                             lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8]);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].left_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].left_boundary_);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].right_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].right_boundary_);
          }
        }
      }
      else
      {
        for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
        {
          if (tar_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
          {
            StoreStartEndIdx(lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                             lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                             lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8]);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].left_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].left_boundary_);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].right_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].right_boundary_);
          }
        }
      }
    }
  }
  else
  {
    bc::uint8_t t_nearest_elem_u8 = 0;
    bc::float32_t t_min_delta_dy_f = 1000;
    /** find the nearest element of ref_lane */
    /** end idx must be equal*/
    bc::uint8_t t_end_idx_0 = ref_lane_data.lane_elements_[0].left_boundary_.existence_
                                  ? ref_lane_data.lane_elements_[0].left_boundary_.total_valid_number_ - 1
                                  : kMaxBoundaryPoint;
    bc::uint8_t t_end_idx_1 = ref_lane_data.lane_elements_[1].left_boundary_.existence_
                                  ? ref_lane_data.lane_elements_[1].left_boundary_.total_valid_number_ - 1
                                  : kMaxBoundaryPoint;
    t_end_idx_u8 = std::min(t_end_idx_0, t_end_idx_1);
    if (t_end_idx_u8 >= kMaxBoundaryPoint)
    {
      tar_lane_data = LaneData();
#ifdef STD_COUT_ENABLE
      std::cout << "Error in Hlmf::NeighborLaneGeometryFusion: Right neighbour lane's ref lane invalid!" << std::endl;
#endif
    }

    else
    {
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (ref_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
        {
          bc::float32_t cur_delta_dy = ref_lane_data.lane_elements_[t_elem_idx_u8].left_boundary_.pts_[t_end_idx_u8].y;
          if (cur_delta_dy < t_min_delta_dy_f)
          {
            t_min_delta_dy_f = cur_delta_dy;
            t_nearest_elem_u8 = t_elem_idx_u8;
          }
        }
      }
      bc::bool_t t_has_split_with_neighbor_b =
          CheckSplit(lane_pack_tar.element_pack_ar_[0].right_pack_cs_,
                     lane_pack_ref.element_pack_ar_[t_nearest_elem_u8].left_pack_cs_);
      if (t_has_split_with_neighbor_b == bc::false_v)
      {
        tar_lane_data.lane_elements_[0].right_boundary_ =
            ref_lane_data.lane_elements_[t_nearest_elem_u8].left_boundary_;
        StoreParStartEndIdx(ref_lane_st_end_idx.ele_st_end_idx_[t_nearest_elem_u8],
                            lane_pack_tar.element_pack_ar_[0].left_pack_cs_,
                            lane_pack_tar.element_pack_ar_[0].right_pack_cs_, lane_st_end_idx.ele_st_end_idx_[0]);
        BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[0].left_pack_cs_,
                       lane_weight.element_weight_ar_[0].left_weight_ar_, lane_st_end_idx.ele_st_end_idx_[0],
                       tar_lane_data.lane_elements_[0].left_boundary_);
        tar_lane_data.lane_elements_[0].right_boundary_.start_idx_ =
            tar_lane_data.lane_elements_[0].left_boundary_.start_idx_;
        tar_lane_data.lane_elements_[0].right_boundary_.reserve_[0] =
            tar_lane_data.lane_elements_[0].left_boundary_.reserve_[0];
        tar_lane_data.lane_elements_[0].right_boundary_.total_valid_number_ =
            tar_lane_data.lane_elements_[0].left_boundary_.total_valid_number_;
        for (bc::uint8_t t_elem_idx_u8 = 1; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
        {
          if (tar_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
          {
            StoreStartEndIdx(lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                             lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                             lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8]);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].left_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].left_boundary_);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].right_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].right_boundary_);
          }
        }
      }
      else
      {
        for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
        {
          if (tar_lane_data.lane_elements_[t_elem_idx_u8].element_valid_)
          {
            StoreStartEndIdx(lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                             lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                             lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8]);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].left_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].left_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].left_boundary_);
            BoundaryFusion(x_ref_ar_, lane_pack_tar.element_pack_ar_[t_elem_idx_u8].right_pack_cs_,
                           lane_weight.element_weight_ar_[t_elem_idx_u8].right_weight_ar_,
                           lane_st_end_idx.ele_st_end_idx_[t_elem_idx_u8],
                           tar_lane_data.lane_elements_[t_elem_idx_u8].right_boundary_);
          }
        }
      }
    }
  }
}

void Hlmf::HostBoundaryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                              const BoundaryPointPack& boundary_point_pack, const BoundaryWeight& boundary_weight,
                              const ElementStEndIdxPack& ele_st_ed_idx, LaneBoundary& lane_boundary)
{
  lane_boundary.total_valid_number_ = ele_st_ed_idx.boundary_end_idx_ - ele_st_ed_idx.boundary_st_idx_ + 1;
  lane_boundary.reserve_[0] = ele_st_ed_idx.boundary_st_idx_;
  lane_boundary.start_idx_ = ele_st_ed_idx.boundary_st_idx_;
  lane_boundary.ego_point_idx_ = kFrontStartIdx;
  per_id_counter_++;
  lane_boundary.per_line_id_ = boundary_point_pack.per_cs_.per_line_id_s32_;
  lane_boundary.matching_id_ = per_id_counter_;
  for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < kMaxBoundaryPoint; t_point_idx_u8++)
  {
    lane_boundary.pts_[t_point_idx_u8].x = x_ref_ar_[t_point_idx_u8];
  }
  HostDeltaHeadingFusion(boundary_point_pack, x_ref_ar_, boundary_weight, ele_st_ed_idx.datum_idx_, lane_boundary);
  lane_boundary.ego_point_idx_ = kFrontStartIdx;
  bc::float32_t t_end_s_f = 0;
  bc::float32_t t_start_s_f = 0;
  for (bc::uint8_t t_point_idx_u8 = kFrontStartIdx + 1; t_point_idx_u8 <= ele_st_ed_idx.boundary_end_idx_;
       t_point_idx_u8++)
  {
    t_end_s_f += CalcSquRoot(lane_boundary.pts_[t_point_idx_u8].x, lane_boundary.pts_[t_point_idx_u8 - 1].x,
                             lane_boundary.pts_[t_point_idx_u8].y, lane_boundary.pts_[t_point_idx_u8 - 1].y);
    lane_boundary.pt_property_[t_point_idx_u8].dis_f_ = t_end_s_f;
  }
  for (bc::int8_t t_point_idx_u8 = kFrontStartIdx - 1; t_point_idx_u8 >= 0; t_point_idx_u8--)
  {
    t_start_s_f -= CalcSquRoot(lane_boundary.pts_[t_point_idx_u8].x, lane_boundary.pts_[t_point_idx_u8 + 1].x,
                               lane_boundary.pts_[t_point_idx_u8].y, lane_boundary.pts_[t_point_idx_u8 + 1].y);
    lane_boundary.pt_property_[t_point_idx_u8].dis_f_ = t_start_s_f;
  }
  /// point_porperty boundary_type
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.map_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
             t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_ &&
             boundary_point_pack.per_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.per_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
             t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.ref_cs_.boundary_type_en_;
    }
    else if ((boundary_point_pack.ldveh_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ldveh_cs_.dy_start_idx_u8_ &&
              t_pt_idx_u8 <= boundary_point_pack.ldveh_cs_.dy_end_idx_u8_) ||
             (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
              t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_ &&
              boundary_point_pack.per_cs_.boundary_type_en_ == LaneBoundaryType::kVirtual))
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = LaneBoundaryType::kVirtual;
    }
    else
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = LaneBoundaryType::kUnknown;
    }
  }
  /// point_porperty source_type
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    lane_boundary.pt_property_[t_pt_idx_u8].source_type_ = 0;
    if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskPerception, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskMap, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
    if (boundary_point_pack.ldveh_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ldveh_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.ldveh_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskVehFlow, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
    if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskTrack, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
  }
  /// point_porperty color_type
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = LaneBoundaryColor::kUnknown;
    if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.ref_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.per_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.map_cs_.boundary_color_en_;
    }
  }
  /**lane_boundary.segs_ */

    lane_boundary.segs_[0].valid_ = bc::true_v;
    lane_boundary.segs_[0].start_s_ = t_start_s_f;
    lane_boundary.segs_[0].end_s_ = t_end_s_f;
     /**lane_boundary.segs_[0].boundary_type_ */
    if (boundary_point_pack.per_cs_.valid_b_ &&
        boundary_point_pack.per_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual)
    {
      lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.per_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.map_cs_.valid_b_)
    {
      lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.map_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.ref_cs_.valid_b_)
    {
      lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.ref_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.ldveh_cs_.valid_b_ ||
             (boundary_point_pack.per_cs_.valid_b_ &&
              boundary_point_pack.per_cs_.boundary_type_en_ == LaneBoundaryType::kVirtual))
    {
      lane_boundary.segs_[0].boundary_type_ = LaneBoundaryType::kVirtual;
    }
    else
    {
      lane_boundary.segs_[0].boundary_type_ = LaneBoundaryType::kUnknown;
    }
    /** lane_boundary.segs_[0].color_type_ */
    lane_boundary.segs_[0].color_type_ = LaneBoundaryColor::kUnknown;
    if (boundary_point_pack.ref_cs_.valid_b_)
    {
      lane_boundary.segs_[0].color_type_ = boundary_point_pack.ref_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.per_cs_.valid_b_)
    {
      lane_boundary.segs_[0].color_type_ = boundary_point_pack.per_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.map_cs_.valid_b_)
    {
      lane_boundary.segs_[0].color_type_ = boundary_point_pack.map_cs_.boundary_color_en_;
    }

  ///track 
  // bc::uint8_t t_seg_idx_adder = 0;
  // LaneBoundaryType t_boundary_type = LaneBoundaryType::kUnknown;
  // LaneBoundaryColor t_color_type = LaneBoundaryColor::kUnknown;
  // for (bc::uint8_t t_pt_idx_u8 = lane_boundary.start_idx_; t_pt_idx_u8 <= ele_st_ed_idx.boundary_end_idx_;
  //      t_pt_idx_u8++)
  // {
  //   if (t_pt_idx_u8 == lane_boundary.start_idx_)
  //   {
  //     t_boundary_type = lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_;
  //     t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //     lane_boundary.segs_[t_seg_idx_adder].start_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //     lane_boundary.segs_[t_seg_idx_adder].start_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //     lane_boundary.segs_[t_seg_idx_adder].boundary_type_ = t_boundary_type;
  //     lane_boundary.segs_[t_seg_idx_adder].color_type_ = t_color_type;
  //     lane_boundary.segs_[t_seg_idx_adder].valid_ = bc::true_v;
  //   }
  //   else
  //   {
  //     if (lane_boundary.pt_property_[t_pt_idx_u8].color_type_ != t_color_type &&
  //         lane_boundary.pt_property_[t_pt_idx_u8].color_type_ != LaneBoundaryColor::kUnknown)
  //     {
  //       t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //     }
  //     if (lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ == t_boundary_type)
  //     {
  //       lane_boundary.segs_[t_seg_idx_adder].end_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //       lane_boundary.segs_[t_seg_idx_adder].end_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //       continue;
  //     }
  //     else
  //     {
  //       lane_boundary.segs_[t_seg_idx_adder].end_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //       lane_boundary.segs_[t_seg_idx_adder].end_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //       t_seg_idx_adder++;
  //       if (t_pt_idx_u8 != ele_st_ed_idx.boundary_end_idx_ && t_seg_idx_adder < kMaxBoundarySegment)
  //       {
  //         t_boundary_type = lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_;
  //         t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //         lane_boundary.segs_[t_seg_idx_adder].start_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //         lane_boundary.segs_[t_seg_idx_adder].start_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //         lane_boundary.segs_[t_seg_idx_adder].boundary_type_ = t_boundary_type;
  //         lane_boundary.segs_[t_seg_idx_adder].color_type_ = t_color_type;
  //         lane_boundary.segs_[t_seg_idx_adder].valid_ = bc::true_v;
  //       }
  //     }
  //   }
  // }
  
  /**lane_boundary.source_type_ */
  lane_boundary.source_type_ = 0;
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskPerception, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.map_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskMap, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.ldveh_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskVehFlow, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.ref_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskTrack, bc::true_v, lane_boundary.source_type_);
  }

  lane_boundary.existence_ = bc::true_v;
}
void Hlmf::NeighDeltaHeadingFusion(const BoundaryPointPack& boundary_point_pack,
                                   const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                   const BoundaryWeight& boundary_weight, const bc::uint8_t datum_idx,
                                   LaneBoundary& lane_boundary)
{
  bc::float32_t t_step_f;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_per_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ref_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_map_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_per_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ref_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_map_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ar;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    t_heading_ar[t_idx_u8] = 0;
    t_heading_per_ar[t_idx_u8] = 0;
    t_heading_ref_ar[t_idx_u8] = 0;
    t_heading_map_ar[t_idx_u8] = 0;
    t_delta_heading_per_ar[t_idx_u8] = 0;
    t_delta_heading_ref_ar[t_idx_u8] = 0;
    t_delta_heading_map_ar[t_idx_u8] = 0;
    t_delta_heading_ar[t_idx_u8] = 0;
  }
  /// 计算headng，按照定义，只考虑前一个点与当前点
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_heading_per_ar[t_idx_u8] =
          (boundary_point_pack.per_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.per_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      t_heading_ref_ar[t_idx_u8] =
          (boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      t_heading_map_ar[t_idx_u8] =
          (boundary_point_pack.map_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.map_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
    }
    else
    {
      t_step_f = x_ref[t_idx_u8 + 1] - x_ref[t_idx_u8];

      /// t_heading_per_ar在kFrontStartIdx处是否要改为c1?
      t_heading_per_ar[t_idx_u8] =
          (boundary_point_pack.per_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.per_cs_.dy_ar_[t_idx_u8]) / t_step_f;
      t_heading_ref_ar[t_idx_u8] =
          (boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8]) / t_step_f;
      t_heading_map_ar[t_idx_u8] =
          (boundary_point_pack.map_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.map_cs_.dy_ar_[t_idx_u8]) / t_step_f;
    }
  }
  ///计算delta heading，按照定义，也只考虑前一个点与当前点
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_delta_heading_per_ar[t_idx_u8] = (t_heading_per_ar[t_idx_u8] - t_heading_per_ar[t_idx_u8 - 1]) / t_step_f;
      t_delta_heading_ref_ar[t_idx_u8] = (t_heading_ref_ar[t_idx_u8] - t_heading_ref_ar[t_idx_u8 - 1]) / t_step_f;
      t_delta_heading_map_ar[t_idx_u8] = (t_heading_map_ar[t_idx_u8] - t_heading_map_ar[t_idx_u8 - 1]) / t_step_f;
    }
    else
    {
      t_step_f = x_ref[t_idx_u8 + 1] - x_ref[t_idx_u8];
      t_delta_heading_per_ar[t_idx_u8] = (t_heading_per_ar[t_idx_u8 + 1] - t_heading_per_ar[t_idx_u8]) / t_step_f;
      t_delta_heading_ref_ar[t_idx_u8] = (t_heading_ref_ar[t_idx_u8 + 1] - t_heading_ref_ar[t_idx_u8]) / t_step_f;
      t_delta_heading_map_ar[t_idx_u8] = (t_heading_map_ar[t_idx_u8 + 1] - t_heading_map_ar[t_idx_u8]) / t_step_f;
    }

    /// fusion

    t_delta_heading_ar[t_idx_u8] =
        boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ * t_delta_heading_per_ar[t_idx_u8] +
        boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ * t_delta_heading_ref_ar[t_idx_u8] +
        boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ * t_delta_heading_map_ar[t_idx_u8];
  }
  /// delta_heading -> heading -> dy,需要拆开，从kFrontStartIdx开始向前向后递推。因为要将kFrontStartIdx作为锚点
  bc::uint8_t t_datum_idx = std::max(kFrontStartIdx, datum_idx);
  for (bc::uint8_t t_idx_u8 = t_datum_idx; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == t_datum_idx)
    {
      lane_boundary.pts_[t_idx_u8].y =
          boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ *
              boundary_point_pack.per_cs_.dy_ar_[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ *
              boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ * boundary_point_pack.map_cs_.dy_ar_[t_idx_u8];
      t_heading_ar[t_idx_u8] =
          boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ * t_heading_per_ar[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ * t_heading_ref_ar[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ * t_heading_map_ar[t_idx_u8];
    }
    else
    {
      t_step_f = (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      lane_boundary.pts_[t_idx_u8].y = lane_boundary.pts_[t_idx_u8 - 1].y + t_heading_ar[t_idx_u8 - 1] * t_step_f;
      t_heading_ar[t_idx_u8] = t_heading_ar[t_idx_u8 - 1] + t_delta_heading_ar[t_idx_u8 - 1] * t_step_f;
    }
  }
  for (bc::int8_t t_idx_s8 = t_datum_idx - 1; t_idx_s8 >= 0; t_idx_s8--)
  {
    t_step_f = (x_ref[t_idx_s8 + 1] - x_ref[t_idx_s8]);
    // if (boundary_point_pack.map_cs_.valid_b_ && boundary_point_pack.ref_cs_.valid_b_)
    // {
    /// 从前往后，需要先计算heading，再计算dy, t_step_f前的符号要反向
    t_heading_ar[t_idx_s8] = t_heading_ar[t_idx_s8 + 1] - t_delta_heading_ar[t_idx_s8] * t_step_f;
    lane_boundary.pts_[t_idx_s8].y = lane_boundary.pts_[t_idx_s8 + 1].y - t_heading_ar[t_idx_s8] * t_step_f;
    // }
    // else
    // {
    //   lane_boundary.pts_[t_idx_s8].x = x_ref[t_idx_s8];
    //   lane_boundary.pts_[t_idx_s8].y = LinearInterpolation(
    //       lane_boundary.pts_[t_idx_s8 + 1].x, lane_boundary.pts_[t_idx_s8 + 1].y, lane_boundary.pts_[t_idx_s8 + 2].x,
    //       lane_boundary.pts_[t_idx_s8 + 2].y, lane_boundary.pts_[t_idx_s8].x, Lin_Interp_Method::kExtrapolate);
    // }
  }
}

void Hlmf::HostDeltaHeadingFusion(const BoundaryPointPack& boundary_point_pack,
                                  const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                  const BoundaryWeight& boundary_weight, const bc::uint8_t datum_idx,
                                  LaneBoundary& lane_boundary)
{
  bc::float32_t t_step_f = x_ref[1] - x_ref[0];
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_per_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dbg_t_heading_per_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ref_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dbg_t_heading_ref_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ldveh_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dbg_t_heading_ldveh_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_map_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dbg_t_heading_map_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_per_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ref_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_map_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ldveh_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ar;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    t_heading_ar[t_idx_u8] = 0;
    t_heading_per_ar[t_idx_u8] = 0;
    dbg_t_heading_per_ar[t_idx_u8] = 0;
    t_heading_ref_ar[t_idx_u8] = 0;
    dbg_t_heading_ref_ar[t_idx_u8] = 0;
    t_heading_ldveh_ar[t_idx_u8] = 0;
    dbg_t_heading_ldveh_ar[t_idx_u8] = 0;
    t_heading_map_ar[t_idx_u8] = 0;
    dbg_t_heading_map_ar[t_idx_u8] = 0;
    t_delta_heading_per_ar[t_idx_u8] = 0;
    t_delta_heading_ref_ar[t_idx_u8] = 0;
    t_delta_heading_map_ar[t_idx_u8] = 0;
    t_delta_heading_ldveh_ar[t_idx_u8] = 0;
    t_delta_heading_ar[t_idx_u8] = 0;
  }

  /// 计算headng，按照定义，只考虑前一个点与当前点
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_heading_per_ar[t_idx_u8] =
          (boundary_point_pack.per_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.per_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      t_heading_ref_ar[t_idx_u8] =
          (boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      t_heading_map_ar[t_idx_u8] =
          (boundary_point_pack.map_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.map_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      t_heading_ldveh_ar[t_idx_u8] =
          (boundary_point_pack.ldveh_cs_.dy_ar_[t_idx_u8] - boundary_point_pack.ldveh_cs_.dy_ar_[t_idx_u8 - 1]) /
          (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
    }
    else
    {
      t_step_f = x_ref[t_idx_u8 + 1] - x_ref[t_idx_u8];
      /// t_heading_per_ar在kFrontStartIdx处是否要改为c1?
      t_heading_per_ar[t_idx_u8] =
          (boundary_point_pack.per_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.per_cs_.dy_ar_[t_idx_u8]) / t_step_f;
      t_heading_ref_ar[t_idx_u8] =
          (boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8]) / t_step_f;
      t_heading_map_ar[t_idx_u8] =
          (boundary_point_pack.map_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.map_cs_.dy_ar_[t_idx_u8]) / t_step_f;
      t_heading_ldveh_ar[t_idx_u8] =
          (boundary_point_pack.ldveh_cs_.dy_ar_[t_idx_u8 + 1] - boundary_point_pack.ldveh_cs_.dy_ar_[t_idx_u8]) /
          t_step_f;
    }
  }

  ///计算delta heading，按照定义，也只考虑前一个点与当前点
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_delta_heading_per_ar[t_idx_u8] = (t_heading_per_ar[t_idx_u8] - t_heading_per_ar[t_idx_u8 - 1]) / t_step_f;
      t_delta_heading_ref_ar[t_idx_u8] = (t_heading_ref_ar[t_idx_u8] - t_heading_ref_ar[t_idx_u8 - 1]) / t_step_f;
      t_delta_heading_map_ar[t_idx_u8] = (t_heading_map_ar[t_idx_u8] - t_heading_map_ar[t_idx_u8 - 1]) / t_step_f;
      t_delta_heading_ldveh_ar[t_idx_u8] = (t_heading_ldveh_ar[t_idx_u8] - t_heading_ldveh_ar[t_idx_u8 - 1]) / t_step_f;
    }
    else
    {
      t_step_f = x_ref[t_idx_u8 + 1] - x_ref[t_idx_u8];
      t_delta_heading_per_ar[t_idx_u8] = (t_heading_per_ar[t_idx_u8 + 1] - t_heading_per_ar[t_idx_u8]) / t_step_f;
      t_delta_heading_ref_ar[t_idx_u8] = (t_heading_ref_ar[t_idx_u8 + 1] - t_heading_ref_ar[t_idx_u8]) / t_step_f;
      t_delta_heading_map_ar[t_idx_u8] = (t_heading_map_ar[t_idx_u8 + 1] - t_heading_map_ar[t_idx_u8]) / t_step_f;
      t_delta_heading_ldveh_ar[t_idx_u8] = (t_heading_ldveh_ar[t_idx_u8 + 1] - t_heading_ldveh_ar[t_idx_u8]) / t_step_f;
    }

    /// fusion

    t_delta_heading_ar[t_idx_u8] =
        boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ * t_delta_heading_per_ar[t_idx_u8] +
        boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ * t_delta_heading_ref_ar[t_idx_u8] +
        boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ * t_delta_heading_map_ar[t_idx_u8] +
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_norm_f_ * t_delta_heading_ldveh_ar[t_idx_u8];
  }

  /// delta_heading -> heading -> dy,需要拆开，从kFrontStartIdx开始向前向后递推。因为要将kFrontStartIdx作为锚点
  bc::uint8_t t_datum_idx = std::max(kFrontStartIdx, datum_idx);
  for (bc::uint8_t t_idx_u8 = t_datum_idx; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == t_datum_idx)
    {
      lane_boundary.pts_[t_idx_u8].y = boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ *
                                           boundary_point_pack.per_cs_.dy_ar_[t_idx_u8] +
                                       boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ *
                                           boundary_point_pack.ref_cs_.dy_ar_[t_idx_u8] +
                                       boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ *
                                           boundary_point_pack.map_cs_.dy_ar_[t_idx_u8] +
                                       boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_norm_f_ *
                                           boundary_point_pack.ldveh_cs_.dy_ar_[t_idx_u8];
      t_heading_ar[t_idx_u8] =
          boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_norm_f_ * t_heading_per_ar[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_norm_f_ * t_heading_ref_ar[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_norm_f_ * t_heading_map_ar[t_idx_u8] +
          boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_norm_f_ * t_heading_ldveh_ar[t_idx_u8];
    }
    else
    {
      t_step_f = (x_ref[t_idx_u8] - x_ref[t_idx_u8 - 1]);
      lane_boundary.pts_[t_idx_u8].y = lane_boundary.pts_[t_idx_u8 - 1].y + t_heading_ar[t_idx_u8 - 1] * t_step_f;
      t_heading_ar[t_idx_u8] = t_heading_ar[t_idx_u8 - 1] + t_delta_heading_ar[t_idx_u8 - 1] * t_step_f;
    }
  }

  for (bc::int8_t t_idx_s8 = t_datum_idx - 1; t_idx_s8 >= 0; t_idx_s8--)
  {
    // if (boundary_point_pack.map_cs_.valid_b_ && boundary_point_pack.ref_cs_.valid_b_)
    // {
    /// 从前往后，需要先计算heading，再计算dy, t_step_f前的符号要反向
    t_heading_ar[t_idx_s8] = t_heading_ar[t_idx_s8 + 1] - t_delta_heading_ar[t_idx_s8] * t_step_f;
    lane_boundary.pts_[t_idx_s8].y = lane_boundary.pts_[t_idx_s8 + 1].y - t_heading_ar[t_idx_s8] * t_step_f;
    // }
    // else
    // {
    //   lane_boundary.pts_[t_idx_s8].x = x_ref[t_idx_s8];
    //   lane_boundary.pts_[t_idx_s8].y = LinearInterpolation(
    //       lane_boundary.pts_[t_idx_s8 + 1].x, lane_boundary.pts_[t_idx_s8 + 1].y, lane_boundary.pts_[t_idx_s8 + 2].x,
    //       lane_boundary.pts_[t_idx_s8 + 2].y, lane_boundary.pts_[t_idx_s8].x, Lin_Interp_Method::kExtrapolate);
    // }
  }
}

void Hlmf::BoundaryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                          const BoundaryPointPack& boundary_point_pack, const BoundaryWeight& boundary_weight,
                          const ElementStEndIdxPack& ele_st_ed_idx, LaneBoundary& lane_boundary)
{
  lane_boundary.total_valid_number_ = ele_st_ed_idx.boundary_end_idx_ - ele_st_ed_idx.boundary_st_idx_ + 1;
  lane_boundary.reserve_[0] = ele_st_ed_idx.boundary_st_idx_;
  lane_boundary.start_idx_ = ele_st_ed_idx.boundary_st_idx_;
  lane_boundary.ego_point_idx_ = kFrontStartIdx;
  per_id_counter_++;
  lane_boundary.per_line_id_ = boundary_point_pack.per_cs_.per_line_id_s32_;
  lane_boundary.matching_id_ = per_id_counter_;
  for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < kMaxBoundaryPoint; t_point_idx_u8++)
  {
    lane_boundary.pts_[t_point_idx_u8].x = x_ref_ar_[t_point_idx_u8];
  }
  NeighDeltaHeadingFusion(boundary_point_pack, x_ref_ar_, boundary_weight, ele_st_ed_idx.datum_idx_, lane_boundary);
  lane_boundary.ego_point_idx_ = kFrontStartIdx;
  bc::float32_t t_end_s_f = 0;
  bc::float32_t t_start_s_f = 0;
  for (bc::uint8_t t_point_idx_u8 = kFrontStartIdx + 1; t_point_idx_u8 <= ele_st_ed_idx.boundary_end_idx_;
       t_point_idx_u8++)
  {
    t_end_s_f += CalcSquRoot(lane_boundary.pts_[t_point_idx_u8].x, lane_boundary.pts_[t_point_idx_u8 - 1].x,
                             lane_boundary.pts_[t_point_idx_u8].y, lane_boundary.pts_[t_point_idx_u8 - 1].y);
    lane_boundary.pt_property_[t_point_idx_u8].dis_f_ = t_end_s_f;
  }
  for (bc::int8_t t_point_idx_u8 = kFrontStartIdx - 1; t_point_idx_u8 >= 0; t_point_idx_u8--)
  {
    t_start_s_f -= CalcSquRoot(lane_boundary.pts_[t_point_idx_u8].x, lane_boundary.pts_[t_point_idx_u8 + 1].x,
                               lane_boundary.pts_[t_point_idx_u8].y, lane_boundary.pts_[t_point_idx_u8 + 1].y);
    lane_boundary.pt_property_[t_point_idx_u8].dis_f_ = t_start_s_f;
  }
  /// point_porperty boundary_type
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.map_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
             t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_ &&
             boundary_point_pack.per_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.per_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
             t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = boundary_point_pack.ref_cs_.boundary_type_en_;
    }
    else if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
             t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_ &&
             boundary_point_pack.per_cs_.boundary_type_en_ == LaneBoundaryType::kVirtual)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = LaneBoundaryType::kVirtual;
    }
    else
    {
      lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ = LaneBoundaryType::kUnknown;
    }
  }
  /// point_porperty source_type

  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    lane_boundary.pt_property_[t_pt_idx_u8].source_type_ = 0;
    if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskPerception, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskMap, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
    if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      SetBitU8(LaneBoundary::kSrcMaskTrack, bc::true_v, lane_boundary.pt_property_[t_pt_idx_u8].source_type_);
    }
  }
  /// point_porperty color_type
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
  {
    lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = LaneBoundaryColor::kUnknown;
    if (boundary_point_pack.ref_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.ref_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.ref_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.ref_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.per_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.per_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.per_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.per_cs_.boundary_color_en_;
    }
    if (boundary_point_pack.map_cs_.valid_b_ && t_pt_idx_u8 >= boundary_point_pack.map_cs_.dy_start_idx_u8_ &&
        t_pt_idx_u8 <= boundary_point_pack.map_cs_.dy_end_idx_u8_)
    {
      lane_boundary.pt_property_[t_pt_idx_u8].color_type_ = boundary_point_pack.map_cs_.boundary_color_en_;
    }
  }

  /**lane_boundary.segs_ */
  lane_boundary.segs_[0].valid_ = bc::true_v;
  lane_boundary.segs_[0].start_s_ = t_start_s_f;
  lane_boundary.segs_[0].end_s_ = t_end_s_f;
  /**lane_boundary.segs_[0].boundary_type_ */
  if (boundary_point_pack.per_cs_.valid_b_ && boundary_point_pack.per_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual)
  {
    lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.per_cs_.boundary_type_en_;
  }
  else if (boundary_point_pack.map_cs_.valid_b_)
  {
    lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.map_cs_.boundary_type_en_;
  }
  else if (boundary_point_pack.ref_cs_.valid_b_)
  {
    lane_boundary.segs_[0].boundary_type_ = boundary_point_pack.ref_cs_.boundary_type_en_;
  }
  else if (boundary_point_pack.per_cs_.valid_b_ &&
           boundary_point_pack.per_cs_.boundary_type_en_ == LaneBoundaryType::kVirtual)
  {
    lane_boundary.segs_[0].boundary_type_ = LaneBoundaryType::kVirtual;
  }
  else
  {
    lane_boundary.segs_[0].boundary_type_ = LaneBoundaryType::kUnknown;
  }

/** lane_boundary.segs_[0].color_type_ */
  lane_boundary.segs_[0].color_type_ = LaneBoundaryColor::kUnknown;
  if (boundary_point_pack.ref_cs_.valid_b_)
  {
    lane_boundary.segs_[0].color_type_ = boundary_point_pack.ref_cs_.boundary_color_en_;
  }
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    lane_boundary.segs_[0].color_type_ = boundary_point_pack.per_cs_.boundary_color_en_;
  }
  if (boundary_point_pack.map_cs_.valid_b_)
  {
    lane_boundary.segs_[0].color_type_ = boundary_point_pack.map_cs_.boundary_color_en_;
  }
  // bc::uint8_t t_seg_idx_adder = 0;
  // LaneBoundaryType t_boundary_type = LaneBoundaryType::kUnknown;
  // LaneBoundaryColor t_color_type = LaneBoundaryColor::kUnknown;
  // for (bc::uint8_t t_pt_idx_u8 = lane_boundary.start_idx_; t_pt_idx_u8 <= ele_st_ed_idx.boundary_end_idx_;
  //      t_pt_idx_u8++)
  // {
  //   if (t_pt_idx_u8 == lane_boundary.start_idx_)
  //   {
  //     t_boundary_type = lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_;
  //     t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //     lane_boundary.segs_[t_seg_idx_adder].start_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //     lane_boundary.segs_[t_seg_idx_adder].start_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //     lane_boundary.segs_[t_seg_idx_adder].boundary_type_ = t_boundary_type;
  //     lane_boundary.segs_[t_seg_idx_adder].color_type_ = t_color_type;
  //     lane_boundary.segs_[t_seg_idx_adder].valid_ = bc::true_v;
  //   }
  //   else
  //   {
  //     if (lane_boundary.pt_property_[t_pt_idx_u8].color_type_ != t_color_type &&
  //         lane_boundary.pt_property_[t_pt_idx_u8].color_type_ != LaneBoundaryColor::kUnknown)
  //     {
  //       t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //     }
  //     if (lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_ == t_boundary_type)
  //     {
  //       lane_boundary.segs_[t_seg_idx_adder].end_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //       lane_boundary.segs_[t_seg_idx_adder].end_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //       continue;
  //     }
  //     else
  //     {
  //       lane_boundary.segs_[t_seg_idx_adder].end_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //       lane_boundary.segs_[t_seg_idx_adder].end_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //       t_seg_idx_adder++;
  //       if (t_pt_idx_u8 != ele_st_ed_idx.boundary_end_idx_ && t_seg_idx_adder < kMaxBoundarySegment)
  //       {
  //         t_boundary_type = lane_boundary.pt_property_[t_pt_idx_u8].boundary_type_;
  //         t_color_type = lane_boundary.pt_property_[t_pt_idx_u8].color_type_;
  //         lane_boundary.segs_[t_seg_idx_adder].start_s_ = lane_boundary.pt_property_[t_pt_idx_u8].dis_f_;
  //         lane_boundary.segs_[t_seg_idx_adder].start_point_ = Point2D(lane_boundary.pts_[t_pt_idx_u8].x,lane_boundary.pts_[t_pt_idx_u8].y);
  //         lane_boundary.segs_[t_seg_idx_adder].boundary_type_ = t_boundary_type;
  //         lane_boundary.segs_[t_seg_idx_adder].color_type_ = t_color_type;
  //         lane_boundary.segs_[t_seg_idx_adder].valid_ = bc::true_v;
  //       }
  //     }
  //   }
  // }
  

  /**lane_boundary.source_type_ */
  lane_boundary.source_type_ = 0;
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskPerception, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.map_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskMap, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.ldveh_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskVehFlow, bc::true_v, lane_boundary.source_type_);
  }
  if (boundary_point_pack.ref_cs_.valid_b_)
  {
    SetBitU8(LaneBoundary::kSrcMaskTrack, bc::true_v, lane_boundary.source_type_);
  }

  lane_boundary.existence_ = bc::true_v;
}

bc::bool_t Hlmf::CheckSplit(const BoundaryPointPack& tar_boundary_pack, const BoundaryPointPack& ref_boundary_pack)
{
  BoundaryPoint t_map_pack_tar = tar_boundary_pack.map_cs_;
  BoundaryPoint t_map_pack_ref = ref_boundary_pack.map_cs_;
  BoundaryPoint t_ref_pack_tar = tar_boundary_pack.ref_cs_;
  BoundaryPoint t_ref_pack_ref = ref_boundary_pack.ref_cs_;
  bc::uint8_t t_map_st_idx_u8 = std::min(t_map_pack_tar.dy_start_idx_u8_, t_map_pack_ref.dy_start_idx_u8_);
  bc::uint8_t t_map_end_idx_u8 = std::min(t_map_pack_tar.dy_end_idx_u8_, t_map_pack_ref.dy_end_idx_u8_);
  bc::uint8_t t_ref_st_idx_u8 = std::min(t_ref_pack_tar.dy_start_idx_u8_, t_ref_pack_ref.dy_start_idx_u8_);
  bc::uint8_t t_ref_end_idx_u8 = std::min(t_ref_pack_tar.dy_end_idx_u8_, t_ref_pack_ref.dy_end_idx_u8_);

  if (t_map_pack_tar.valid_b_ && t_map_pack_ref.valid_b_ &&
      (fabsf(t_map_pack_ref.dy_ar_[t_map_st_idx_u8] - t_map_pack_tar.dy_ar_[t_map_st_idx_u8]) > 1.f ||
       fabsf(t_map_pack_ref.dy_ar_[t_map_end_idx_u8] - t_map_pack_tar.dy_ar_[t_map_end_idx_u8]) > 1.f))
  {
    return bc::true_v;
  }
  else if (t_ref_pack_tar.valid_b_ && t_ref_pack_ref.valid_b_ &&
           (fabsf(t_ref_pack_ref.dy_ar_[t_ref_st_idx_u8] - t_ref_pack_tar.dy_ar_[t_ref_st_idx_u8]) > 1.f ||
            fabsf(t_ref_pack_ref.dy_ar_[t_ref_end_idx_u8] - t_ref_pack_tar.dy_ar_[t_ref_end_idx_u8]) > 1.f))
  {
    return bc::true_v;
  }
  else
  {
    return bc::false_v;
  }
}

void Hlmf::StoreParStartEndIdx(const ElementStEndIdxPack& ref_idx, const BoundaryPointPack& left_pack,
                               const BoundaryPointPack& right_pack, ElementStEndIdxPack& st_end_idx)
{
  bc::uint8_t lf_per_start_idx_u8 = 100, lf_ref_start_idx_u8 = 100, lf_map_start_idx_u8 = 100,
              lf_ldveh_start_idx_u8 = 100;
  bc::uint8_t rt_per_start_idx_u8 = 100, rt_ref_start_idx_u8 = 100, rt_map_start_idx_u8 = 100,
              rt_ldveh_start_idx_u8 = 100;
  bc::uint8_t lf_per_end_idx_u8 = 0, lf_ref_end_idx_u8 = 0, lf_map_end_idx_u8 = 0, lf_ldveh_end_idx_u8 = 0;
  bc::uint8_t rt_per_end_idx_u8 = 0, rt_ref_end_idx_u8 = 0, rt_map_end_idx_u8 = 0, rt_ldveh_end_idx_u8 = 0;
  bc::float32_t lf_per_start_x_f = 100.f, lf_ref_start_x_f = 100.f, lf_map_start_x_f = 100.f,
                lf_ldveh_start_x_f = 100.f;
  bc::float32_t rt_per_start_x_f = 100.f, rt_ref_start_x_f = 100.f, rt_map_start_x_f = 100.f,
                rt_ldveh_start_x_f = 100.f;
  bc::float32_t lf_per_end_x_f = 0.f, lf_ref_end_x_f = 0.f, lf_map_end_x_f = 0.f, lf_ldveh_end_x_f = 0.f;
  bc::float32_t rt_per_end_x_f = 0.f, rt_ref_end_x_f = 0.f, rt_map_end_x_f = 0.f, rt_ldveh_end_x_f = 0.f;
  if (left_pack.per_cs_.valid_b_)
  {
    lf_per_start_idx_u8 = left_pack.per_cs_.dy_start_idx_u8_;
    lf_per_end_idx_u8 = left_pack.per_cs_.dy_end_idx_u8_;
    lf_per_start_x_f = left_pack.per_cs_.dx_start_f_;
    lf_per_end_x_f = left_pack.per_cs_.dx_end_f_ + 5.f;
  }
  if (left_pack.ref_cs_.valid_b_)
  {
    lf_ref_start_idx_u8 = left_pack.ref_cs_.dy_start_idx_u8_;
    lf_ref_end_idx_u8 = left_pack.ref_cs_.dy_end_idx_u8_;
    lf_ref_start_x_f = left_pack.ref_cs_.dx_start_f_;
    lf_ref_end_x_f = left_pack.ref_cs_.dx_end_f_;
  }
  if (left_pack.map_cs_.valid_b_)
  {
    lf_map_start_idx_u8 = left_pack.map_cs_.dy_start_idx_u8_;
    lf_map_end_idx_u8 = left_pack.map_cs_.dy_end_idx_u8_;
    lf_map_start_x_f = left_pack.map_cs_.dx_start_f_;
    lf_map_end_x_f = left_pack.map_cs_.dx_end_f_ + 5.f;
  }
  if (left_pack.ldveh_cs_.valid_b_)
  {
    lf_ldveh_start_idx_u8 = left_pack.ldveh_cs_.dy_start_idx_u8_;
    lf_ldveh_end_idx_u8 = left_pack.ldveh_cs_.dy_end_idx_u8_;
    lf_ldveh_start_x_f = left_pack.ldveh_cs_.dx_start_f_;
    lf_ldveh_end_x_f = left_pack.ldveh_cs_.dx_end_f_;
  }
  if (right_pack.per_cs_.valid_b_)
  {
    rt_per_start_idx_u8 = right_pack.per_cs_.dy_start_idx_u8_;
    rt_per_end_idx_u8 = right_pack.per_cs_.dy_end_idx_u8_;
    rt_per_start_x_f = right_pack.per_cs_.dx_start_f_;
    rt_per_end_x_f = right_pack.per_cs_.dx_end_f_ + 5.f;
  }
  if (right_pack.ref_cs_.valid_b_)
  {
    rt_ref_start_idx_u8 = right_pack.ref_cs_.dy_start_idx_u8_;
    rt_ref_end_idx_u8 = right_pack.ref_cs_.dy_end_idx_u8_;
    rt_ref_start_x_f = right_pack.ref_cs_.dx_start_f_;
    rt_ref_end_x_f = right_pack.ref_cs_.dx_end_f_;
  }
  if (right_pack.map_cs_.valid_b_)
  {
    rt_map_start_idx_u8 = right_pack.map_cs_.dy_start_idx_u8_;
    rt_map_end_idx_u8 = right_pack.map_cs_.dy_end_idx_u8_;
    rt_map_start_x_f = right_pack.map_cs_.dx_start_f_;
    rt_map_end_x_f = right_pack.map_cs_.dx_end_f_ + 5.f;
  }
  if (right_pack.ldveh_cs_.valid_b_)
  {
    rt_ldveh_start_idx_u8 = right_pack.ldveh_cs_.dy_start_idx_u8_;
    rt_ldveh_end_idx_u8 = right_pack.ldveh_cs_.dy_end_idx_u8_;
    rt_ldveh_start_x_f = right_pack.ldveh_cs_.dx_start_f_;
    rt_ldveh_end_x_f = right_pack.ldveh_cs_.dx_end_f_;
  }
  bc::uint8_t start_idx_u8, end_idx_u8;
  bc::float32_t start_x_f, end_x_f;
  start_idx_u8 =
      std::min({std::min({lf_per_start_idx_u8, lf_ref_start_idx_u8, lf_map_start_idx_u8, lf_ldveh_start_idx_u8}),
                std::min({rt_per_start_idx_u8, rt_ref_start_idx_u8, rt_map_start_idx_u8, rt_ldveh_start_idx_u8}),
                ref_idx.boundary_st_idx_});
  start_x_f = std::min({std::min({lf_per_start_x_f, lf_ref_start_x_f, lf_map_start_x_f, lf_ldveh_start_x_f}),
                        std::min({rt_per_start_x_f, rt_ref_start_x_f, rt_map_start_x_f, rt_ldveh_start_x_f}),
                        ref_idx.boundary_st_x_f_});
  end_idx_u8 = std::min({std::max({lf_per_end_idx_u8, lf_ref_end_idx_u8, lf_map_end_idx_u8, lf_ldveh_end_idx_u8}),
                         std::max({rt_per_end_idx_u8, rt_ref_end_idx_u8, rt_map_end_idx_u8, rt_ldveh_end_idx_u8}),
                         ref_idx.boundary_end_idx_});
  end_x_f = std::min({std::max({lf_per_end_x_f, lf_ref_end_x_f, lf_map_end_x_f, lf_ldveh_end_x_f}),
                      std::max({rt_per_end_x_f, rt_ref_end_x_f, rt_map_end_x_f, rt_ldveh_end_x_f}),
                      ref_idx.boundary_end_x_f_});
  st_end_idx.boundary_st_idx_ = start_idx_u8;
  st_end_idx.boundary_st_x_f_ = start_x_f;
  st_end_idx.boundary_end_idx_ = end_idx_u8;
  st_end_idx.boundary_end_x_f_ = end_x_f;
  bc::uint8_t t_min_start_idx = st_end_idx.boundary_st_idx_;
  if (t_min_start_idx < kMaxBoundaryPoint)
  {
    st_end_idx.datum_idx_ = t_min_start_idx > kFrontStartIdx ? t_min_start_idx : kFrontStartIdx;
  }
  else
  {
    st_end_idx.datum_idx_ = kFrontStartIdx;
  }
}

void Hlmf::StoreStartEndIdx(const BoundaryPointPack& left_pack, const BoundaryPointPack& right_pack,
                            ElementStEndIdxPack& st_end_idx)
{
  bc::uint8_t lf_per_start_idx_u8 = 100, lf_ref_start_idx_u8 = 100, lf_map_start_idx_u8 = 100,
              lf_ldveh_start_idx_u8 = 100;
  bc::uint8_t rt_per_start_idx_u8 = 100, rt_ref_start_idx_u8 = 100, rt_map_start_idx_u8 = 100,
              rt_ldveh_start_idx_u8 = 100;
  bc::uint8_t lf_per_end_idx_u8 = 0, lf_ref_end_idx_u8 = 0, lf_map_end_idx_u8 = 0, lf_ldveh_end_idx_u8 = 0;
  bc::uint8_t rt_per_end_idx_u8 = 0, rt_ref_end_idx_u8 = 0, rt_map_end_idx_u8 = 0, rt_ldveh_end_idx_u8 = 0;
  bc::float32_t lf_per_start_x_f = 100.f, lf_ref_start_x_f = 100.f, lf_map_start_x_f = 100.f,
                lf_ldveh_start_x_f = 100.f;
  bc::float32_t rt_per_start_x_f = 100.f, rt_ref_start_x_f = 100.f, rt_map_start_x_f = 100.f,
                rt_ldveh_start_x_f = 100.f;
  bc::float32_t lf_per_end_x_f = 0.f, lf_ref_end_x_f = 0.f, lf_map_end_x_f = 0.f, lf_ldveh_end_x_f = 0.f;
  bc::float32_t rt_per_end_x_f = 0.f, rt_ref_end_x_f = 0.f, rt_map_end_x_f = 0.f, rt_ldveh_end_x_f = 0.f;
  if (left_pack.per_cs_.valid_b_)
  {
    lf_per_start_idx_u8 = left_pack.per_cs_.dy_start_idx_u8_;
    lf_per_end_idx_u8 = left_pack.per_cs_.dy_end_idx_u8_;
    lf_per_start_x_f = left_pack.per_cs_.dx_start_f_;
    lf_per_end_x_f = left_pack.per_cs_.dx_end_f_ + 5.f;
  }
  if (left_pack.ref_cs_.valid_b_)
  {
    lf_ref_start_idx_u8 = left_pack.ref_cs_.dy_start_idx_u8_;
    lf_ref_end_idx_u8 = left_pack.ref_cs_.dy_end_idx_u8_;
    lf_ref_start_x_f = left_pack.ref_cs_.dx_start_f_;
    lf_ref_end_x_f = left_pack.ref_cs_.dx_end_f_;
  }
  if (left_pack.map_cs_.valid_b_)
  {
    lf_map_start_idx_u8 = left_pack.map_cs_.dy_start_idx_u8_;
    lf_map_end_idx_u8 = left_pack.map_cs_.dy_end_idx_u8_;
    lf_map_start_x_f = left_pack.map_cs_.dx_start_f_;
    lf_map_end_x_f = left_pack.map_cs_.dx_end_f_ + 5.f;
  }
  if (left_pack.ldveh_cs_.valid_b_)
  {
    lf_ldveh_start_idx_u8 = left_pack.ldveh_cs_.dy_start_idx_u8_;
    lf_ldveh_end_idx_u8 = left_pack.ldveh_cs_.dy_end_idx_u8_;
    lf_ldveh_start_x_f = left_pack.ldveh_cs_.dx_start_f_;
    lf_ldveh_end_x_f = left_pack.ldveh_cs_.dx_end_f_;
  }
  if (right_pack.per_cs_.valid_b_)
  {
    rt_per_start_idx_u8 = right_pack.per_cs_.dy_start_idx_u8_;
    rt_per_end_idx_u8 = right_pack.per_cs_.dy_end_idx_u8_;
    rt_per_start_x_f = right_pack.per_cs_.dx_start_f_;
    rt_per_end_x_f = right_pack.per_cs_.dx_end_f_ + 5.f;
  }
  if (right_pack.ref_cs_.valid_b_)
  {
    rt_ref_start_idx_u8 = right_pack.ref_cs_.dy_start_idx_u8_;
    rt_ref_end_idx_u8 = right_pack.ref_cs_.dy_end_idx_u8_;
    rt_ref_start_x_f = right_pack.ref_cs_.dx_start_f_;
    rt_ref_end_x_f = right_pack.ref_cs_.dx_end_f_;
  }
  if (right_pack.map_cs_.valid_b_)
  {
    rt_map_start_idx_u8 = right_pack.map_cs_.dy_start_idx_u8_;
    rt_map_end_idx_u8 = right_pack.map_cs_.dy_end_idx_u8_;
    rt_map_start_x_f = right_pack.map_cs_.dx_start_f_;
    rt_map_end_x_f = right_pack.map_cs_.dx_end_f_ + 5.f;
  }
  if (right_pack.ldveh_cs_.valid_b_)
  {
    rt_ldveh_start_idx_u8 = right_pack.ldveh_cs_.dy_start_idx_u8_;
    rt_ldveh_end_idx_u8 = right_pack.ldveh_cs_.dy_end_idx_u8_;
    rt_ldveh_start_x_f = right_pack.ldveh_cs_.dx_start_f_;
    rt_ldveh_end_x_f = right_pack.ldveh_cs_.dx_end_f_;
  }
  bc::uint8_t start_idx_u8, end_idx_u8;
  bc::float32_t start_x_f, end_x_f;
  start_idx_u8 =
      std::min(std::min({lf_per_start_idx_u8, lf_ref_start_idx_u8, lf_map_start_idx_u8, lf_ldveh_start_idx_u8}),
               std::min({rt_per_start_idx_u8, rt_ref_start_idx_u8, rt_map_start_idx_u8, rt_ldveh_start_idx_u8}));
  start_x_f = std::min(std::min({lf_per_start_x_f, lf_ref_start_x_f, lf_map_start_x_f, lf_ldveh_start_x_f}),
                       std::min({rt_per_start_x_f, rt_ref_start_x_f, rt_map_start_x_f, rt_ldveh_start_x_f}));
  end_idx_u8 = std::min(std::max({lf_per_end_idx_u8, lf_ref_end_idx_u8, lf_map_end_idx_u8, lf_ldveh_end_idx_u8}),
                        std::max({rt_per_end_idx_u8, rt_ref_end_idx_u8, rt_map_end_idx_u8, rt_ldveh_end_idx_u8}));
  end_x_f = std::min(std::max({lf_per_end_x_f, lf_ref_end_x_f, lf_map_end_x_f, lf_ldveh_end_x_f}),
                     std::max({rt_per_end_x_f, rt_ref_end_x_f, rt_map_end_x_f, rt_ldveh_end_x_f}));
  st_end_idx.boundary_st_idx_ = start_idx_u8;
  st_end_idx.boundary_st_x_f_ = start_x_f;
  st_end_idx.boundary_end_idx_ = end_idx_u8;
  st_end_idx.boundary_end_x_f_ = end_x_f;
  bc::uint8_t t_min_start_idx = st_end_idx.boundary_st_idx_;
  if (t_min_start_idx < kMaxBoundaryPoint)
  {
    st_end_idx.datum_idx_ = t_min_start_idx > kFrontStartIdx ? t_min_start_idx : kFrontStartIdx;
  }
  else
  {
    st_end_idx.datum_idx_ = kFrontStartIdx;
  }
}

bc::bool_t Hlmf::CenterLineGenerator(const LaneBoundary& left, const LaneBoundary& right, ReferenceLine& refline)
{
  bc::uint8_t t_pts_num_u8 = std::min(left.total_valid_number_, right.total_valid_number_);
  refline = ReferenceLine();
  refline.ref_line_valid_pts_ = t_pts_num_u8;
  bc::TCArray<Point2D, 100> pts;
  /** per(no map): poly from ego
   *  map: poly from start
   */
  bc::uint8_t t_start_idx = left.start_idx_;
  for (bc::uint8_t t_point_idx_u8 = t_start_idx; t_point_idx_u8 < t_start_idx + t_pts_num_u8; t_point_idx_u8++)
  {
    Point2D t_current_point =
        Point2D(left.pts_[t_point_idx_u8].x, (left.pts_[t_point_idx_u8].y + right.pts_[t_point_idx_u8].y) / 2.f);
    pts[t_point_idx_u8 - t_start_idx] = t_current_point;
    if (t_point_idx_u8 > t_start_idx + 1 && in_map_b)
    {
      bc::float32_t temp_delta_y_f = fabsf(t_current_point.y - pts[t_point_idx_u8 - t_start_idx - 1].y);
      bc::float32_t temp_delta_x_f = fabsf(t_current_point.x - pts[t_point_idx_u8 - t_start_idx - 1].x);
      bc::float32_t temp_rate_f = temp_delta_y_f / temp_delta_x_f;
      if (temp_rate_f > 1.5f)
      {
        t_pts_num_u8 = t_point_idx_u8 - t_start_idx - 1;
        break;
      }
    }
  }
  if (t_pts_num_u8 <= 3)
  {
    return bc::false_v;
  }
  PolynomialRegression poly_fit;
  Eigen::Vector4d local_coeff{0.f, 0.f, 0.f, 0.f};
  poly_fit.SetPara(pts, t_pts_num_u8, true);
  local_coeff = poly_fit.PolyGen();
  RefLineCoeff& coeff = refline.coeff_;
  coeff.clothoid_.c0_position = local_coeff[0];
  coeff.clothoid_.c1_heading_angle = local_coeff[1];
  coeff.clothoid_.c2_curvature = local_coeff[2];
  coeff.clothoid_.c3_curvature_derivative = local_coeff[3];
  coeff.start_x_ = left.pts_[t_start_idx].x;
  coeff.end_x_ = left.pts_[t_start_idx + refline.ref_line_valid_pts_ - 1].x;
  coeff.valid_ = bc::true_v;
  bc::float32_t t_pts_rmse_f = 0.f;
  t_pts_rmse_f = poly_fit.GetRootMeanSquareErrorForSomePoints(t_pts_num_u8);
  if (t_pts_rmse_f > kMinRMSE)
  {
    return bc::false_v;
  }
  return bc::true_v;
}

bc::float32_t Hlmf::CalculateMean(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& raw_kappa, bc::uint8_t start_idx,
                                  bc::uint8_t end_idx)
{
  bc::float32_t t_sum_f = 0.f;
  if (start_idx <= end_idx)
  {
    for (bc::uint8_t t_idx_u8 = start_idx; t_idx_u8 <= end_idx; t_idx_u8++)
    {
      t_sum_f += raw_kappa[t_idx_u8];
    }
    return t_sum_f / (end_idx - start_idx + 1);
  }
  else
  {
    return 0.f;
  }
}

void Hlmf::ReflineFillIn(const LaneBoundary& left, const LaneBoundary& right, const ElementStEndIdxPack& st_end_idx,
                         ReferenceLine& refline, const EmData& em_last_cycle_output, const EmParam& param_st_,
                         const bc::float32_t time_cycle_f, const bc::float32_t long_velocity_f,
                         const bc::uint8_t lane_idx, const bc::uint8_t element_idx)
{
  bc::uint8_t t_start_idx = left.start_idx_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_heading_ar = {0.f};
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_delta_heading_ar = {0.f};
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_kappa_ar = {0.f};
  bc::float32_t t_step_f = 0.f;
  bc::uint8_t t_num_mean_u8 = param_st_.hlmf_param_st_.p_num_mean_filter_point_kappa_u8_;
  bc::float32_t t_kappa_f = 0.f;
  CurvParam<bc::float32_t, kMaxBoundaryPoint> t_data_interp_cur;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_mean_filter_kappa_ar = {0.f};

  refline.reserve_[0] = t_start_idx;
  refline.ref_start_idx_ = t_start_idx;
  for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < kMaxBoundaryPoint; t_point_idx_u8++)
  {
    refline.ref_line_pts_[t_point_idx_u8].pos_.x = left.pts_[t_point_idx_u8].x;
    refline.ref_line_pts_[t_point_idx_u8].pos_.y = (right.pts_[t_point_idx_u8].y + left.pts_[t_point_idx_u8].y) / 2.f;
    if (t_point_idx_u8 >= st_end_idx.boundary_st_idx_ && t_point_idx_u8 <= st_end_idx.boundary_end_idx_)
    {
      refline.ref_line_pts_[t_point_idx_u8].valid_ = bc::true_v;
    }
    else
    {
      refline.ref_line_pts_[t_point_idx_u8].valid_ = bc::false_v;
    }
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_step_f = refline.ref_line_pts_[t_idx_u8].pos_.x - refline.ref_line_pts_[t_idx_u8 - 1].pos_.x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_heading_ar[t_idx_u8] =
          (refline.ref_line_pts_[t_idx_u8].pos_.y - refline.ref_line_pts_[t_idx_u8 - 1].pos_.y) / t_step_f;
    }
    else
    {
      t_step_f = refline.ref_line_pts_[t_idx_u8 + 1].pos_.x - refline.ref_line_pts_[t_idx_u8].pos_.x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_heading_ar[t_idx_u8] =
          (refline.ref_line_pts_[t_idx_u8 + 1].pos_.y - refline.ref_line_pts_[t_idx_u8].pos_.y) / t_step_f;
    }
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    if (t_idx_u8 == kMaxNumBoundary - 1)
    {
      t_step_f = refline.ref_line_pts_[t_idx_u8].pos_.x - refline.ref_line_pts_[t_idx_u8 - 1].pos_.x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_delta_heading_ar[t_idx_u8] = (t_heading_ar[t_idx_u8] - t_heading_ar[t_idx_u8 - 1]) / t_step_f;
    }
    else
    {
      t_step_f = refline.ref_line_pts_[t_idx_u8 + 1].pos_.x - refline.ref_line_pts_[t_idx_u8].pos_.x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_delta_heading_ar[t_idx_u8] = (t_heading_ar[t_idx_u8 + 1] - t_heading_ar[t_idx_u8]) / t_step_f;
    }
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    t_kappa_ar[t_idx_u8] =
        t_delta_heading_ar[t_idx_u8] / (pow(1 + t_heading_ar[t_idx_u8] * t_heading_ar[t_idx_u8], 1.5));
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  {
    if (t_idx_u8 < t_num_mean_u8)
    {
      t_mean_filter_kappa_ar[t_idx_u8] = CalculateMean(t_kappa_ar, 0, 2 * t_num_mean_u8 - 1);
    }
    else if (t_idx_u8 + t_num_mean_u8 > kMaxBoundaryPoint - 1)
    {
      t_mean_filter_kappa_ar[t_idx_u8] =
          CalculateMean(t_kappa_ar, kMaxBoundaryPoint - 2 * t_num_mean_u8, kMaxBoundaryPoint - 1);
    }
    else
    {
      t_mean_filter_kappa_ar[t_idx_u8] = CalculateMean(t_kappa_ar, t_idx_u8 - t_num_mean_u8, t_idx_u8 + t_num_mean_u8);
    }
  }

  if (em_last_cycle_output.lanes_[lane_idx].lane_elements_[element_idx].element_valid_ &&
      em_last_cycle_output.lanes_[lane_idx].lane_elements_[element_idx].left_boundary_.source_type_ != 4)
  {
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
    {
      t_data_interp_cur.x_arr[t_idx_u8] = em_last_cycle_output.lanes_[lane_idx]
                                              .lane_elements_[element_idx]
                                              .reference_line_.ref_line_pts_[t_idx_u8]
                                              .pos_.x;
      t_data_interp_cur.y_arr[t_idx_u8] = em_last_cycle_output.lanes_[lane_idx]
                                              .lane_elements_[element_idx]
                                              .reference_line_.ref_line_pts_[t_idx_u8]
                                              .curvature_;
    }
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
    {
      t_kappa_f =
          InterpCurv(t_data_interp_cur, refline.ref_line_pts_[t_idx_u8].pos_.x + time_cycle_f * long_velocity_f);
      refline.ref_line_pts_[t_idx_u8].curvature_ =
          LowPass(t_mean_filter_kappa_ar[t_idx_u8], t_kappa_f, param_st_.hlmf_param_st_.p_lowpass_time_filter_kappa_f_,
                  time_cycle_f);
      refline.ref_line_pts_[t_idx_u8].heading_ = std::atan(t_heading_ar[t_idx_u8]);
    }
  }
  else
  {
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
    {
      refline.ref_line_pts_[t_idx_u8].curvature_ = t_mean_filter_kappa_ar[t_idx_u8];
      refline.ref_line_pts_[t_idx_u8].heading_ = std::atan(t_heading_ar[t_idx_u8]);
    }
  }

  /**each point's lane_width_*/
  for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < kMaxBoundaryPoint; t_point_idx_u8++)
  {
    if (fabsf(cos(refline.ref_line_pts_[t_point_idx_u8].heading_)) <= FLOAT32_EPSILON)
    {
      refline.ref_line_pts_[t_point_idx_u8].lane_width_ =
          fabsf((left.pts_[t_point_idx_u8].y - right.pts_[t_point_idx_u8].y) *
                cos(refline.ref_line_pts_[t_point_idx_u8].heading_));
    }
    else
    {
      refline.ref_line_pts_[t_point_idx_u8].lane_width_ =
          fabsf((left.pts_[t_point_idx_u8].y - right.pts_[t_point_idx_u8].y) *
                cos(refline.ref_line_pts_[t_point_idx_u8].heading_));
    }
  }
  refline.current_point_idx_ = left.ego_point_idx_;
  /**calc s (t_point_idx_u8 > 0(current_point_idx_)  s > 0)*/
  for (bc::uint8_t t_point_idx_u8 = refline.current_point_idx_; t_point_idx_u8 < kMaxBoundaryPoint; t_point_idx_u8++)
  {
    if (t_point_idx_u8 == refline.current_point_idx_)
    {
      refline.ref_line_pts_[t_point_idx_u8].s_ = 0.f;
    }
    else
    {
      refline.ref_line_pts_[t_point_idx_u8].s_ =
          CalcSquRoot(refline.ref_line_pts_[t_point_idx_u8].pos_.x, refline.ref_line_pts_[t_point_idx_u8 - 1].pos_.x,
                      refline.ref_line_pts_[t_point_idx_u8].pos_.y, refline.ref_line_pts_[t_point_idx_u8 - 1].pos_.y) +
          refline.ref_line_pts_[t_point_idx_u8 - 1].s_;
    }
  }
  /**calc s (t_point_idx_u8 < kFrontStartIdx(current_point_idx_)  s < kFrontStartIdx)*/
  for (bc::int8_t t_point_idx_u8 = refline.current_point_idx_ - 1; t_point_idx_u8 >= 0; t_point_idx_u8--)
  {
    refline.ref_line_pts_[t_point_idx_u8].s_ =
        -CalcSquRoot(refline.ref_line_pts_[t_point_idx_u8].pos_.x, refline.ref_line_pts_[t_point_idx_u8 + 1].pos_.x,
                     refline.ref_line_pts_[t_point_idx_u8].pos_.y, refline.ref_line_pts_[t_point_idx_u8 + 1].pos_.y) +
        refline.ref_line_pts_[t_point_idx_u8 + 1].s_;
  }
  /**coeff s range*/
  refline.coeff_.start_s_ = refline.ref_line_pts_[refline.ref_start_idx_].s_;
  refline.coeff_.end_s_ = refline.ref_line_pts_[refline.ref_start_idx_ + refline.ref_line_valid_pts_ - 1].s_;
  /** heading segs*/
  refline.heading_segs_[0].start_s_ = refline.coeff_.start_s_;
  refline.heading_segs_[0].end_s_ = refline.coeff_.end_s_;
  refline.heading_segs_[0].start_heading_ = refline.ref_line_pts_[refline.ref_start_idx_].heading_;
  refline.heading_segs_[0].end_heading_ =
      refline.ref_line_pts_[refline.ref_start_idx_ + refline.ref_line_valid_pts_ - 1].heading_;
  refline.heading_segs_[0].valid_ = bc::true_v;
  /** lane_width segs*/
  refline.width_segs_[0].start_s_ = refline.coeff_.start_s_;
  refline.width_segs_[0].end_s_ = refline.coeff_.end_s_;
  refline.width_segs_[0].start_width_ = refline.ref_line_pts_[refline.ref_start_idx_].lane_width_;
  refline.width_segs_[0].end_width_ =
      refline.ref_line_pts_[refline.ref_start_idx_ + refline.ref_line_valid_pts_ - 1].lane_width_;
  refline.width_segs_[0].valid_ = bc::true_v;
  /** curvature segs*/
  refline.curvature_segs_[0].start_s_ = refline.coeff_.start_s_;
  refline.curvature_segs_[0].end_s_ = refline.coeff_.end_s_;
  refline.curvature_segs_[0].start_curvature_ = refline.ref_line_pts_[refline.ref_start_idx_].curvature_;
  refline.curvature_segs_[0].end_curvature_ =
      refline.ref_line_pts_[refline.ref_start_idx_ + refline.ref_line_valid_pts_ - 1].curvature_;
  refline.curvature_segs_[0].valid_ = bc::true_v;
  refline.available_ = bc::true_v;
}

bc::float32_t Hlmf::CalcSquRoot(const bc::float32_t x1, const bc::float32_t x2, const bc::float32_t y1,
                                const bc::float32_t y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
bc::bool_t Hlmf::FindEgoIdx(LaneBoundary& boundary)
{
  bc::bool_t t_find_ego_idx_b = bc::false_v;
  if (boundary.start_idx_ < kFrontStartIdx && boundary.start_idx_ + boundary.total_valid_number_ - 1 > kFrontStartIdx)
  {
    t_find_ego_idx_b = bc::true_v;
  }
  // for (bc::uint8_t t_pt_idx_u8 = boundary.start_idx_; t_pt_idx_u8 < boundary.start_idx_ +
  // boundary.total_valid_number_;
  //      t_pt_idx_u8++)
  // {
  //   if (fabsf(boundary.pts_[t_pt_idx_u8].x) <= 0.5)
  //   {
  //     if ((fabsf(boundary.pts_[t_pt_idx_u8].x) <= fabsf(boundary.pts_[t_pt_idx_u8 + 1].x) ||
  //          t_pt_idx_u8 == boundary.total_valid_number_ - 1))
  //     {
  //       boundary.ego_point_idx_ = t_pt_idx_u8;
  //       t_find_ego_idx_b = bc::true_v;
  //     }
  //     else
  //     {
  //       boundary.ego_point_idx_ = 0;
  //     }
  //   }
  // }
  return t_find_ego_idx_b;
}

void Hlmf::CalcDyBoundary(const LaneElement& element_cs, const bc::float32_t dx_tsr_f,
                          bc::float32_t& dy_boundary_left_f, bc::float32_t& dy_boundary_right_f)
{
  /** Find the closest point of the two boundary between ego and tsr */
  bc::uint8_t t_closestpt_left_u8 = 0;
  bc::uint8_t t_closestpt_right_u8 = 0;
  for (bc::uint8_t t_idx_u8 = element_cs.left_boundary_.total_valid_number_ - 1; t_idx_u8 > 0; t_idx_u8--)
  {
    if (element_cs.left_boundary_.pts_[t_idx_u8].x <= dx_tsr_f)
    {
      t_closestpt_left_u8 = t_idx_u8;
      break;
    }
  }
  for (bc::uint8_t t_idx_u8 = element_cs.right_boundary_.total_valid_number_ - 1; t_idx_u8 > 0; t_idx_u8--)
  {
    if (element_cs.right_boundary_.pts_[t_idx_u8].x <= dx_tsr_f)
    {
      t_closestpt_right_u8 = t_idx_u8;
      break;
    }
  }
  dy_boundary_left_f = element_cs.left_boundary_.pts_[t_closestpt_left_u8].y;
  dy_boundary_right_f = element_cs.right_boundary_.pts_[t_closestpt_right_u8].y;
}

void Hlmf::SemanticInfoSet(const SemanticPack& semantic_pack_cs)
{
  em_collection_cs_->map_geofence_ = semantic_pack_cs.map_geofence_;
  em_collection_cs_->map_guide_pts_ = semantic_pack_cs.map_guide_pts_;
  em_collection_cs_->map_car_position_ = semantic_pack_cs.map_car_position_;
  em_collection_cs_->relat_dest_lane_ = semantic_pack_cs.relat_dest_lane_;
  em_collection_cs_->global_lane_num_ = semantic_pack_cs.global_lane_num_;
  em_collection_cs_->main_lane_num_ = semantic_pack_cs.main_lane_num_;
  // em_collection_cs_->timestamp_ = semantic_pack_cs.timestamp_;
  em_collection_cs_->tja_target_id_ = semantic_pack_cs.tja_target_id_;
  em_collection_cs_->host_lane_idx_ = semantic_pack_cs.host_lane_idx_;
  em_collection_cs_->intersection_data_ = semantic_pack_cs.map_intersection_data_;
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (em_collection_cs_->lanes_[t_lane_idx_u8].lane_valid_ &&
        semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].valid_b_)
    {
      em_collection_cs_->lanes_[t_lane_idx_u8].main_lane_idx_ =
          semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].main_lane_idx_u8_;
      em_collection_cs_->lanes_[t_lane_idx_u8].global_lane_idx_ =
          semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].global_lane_idx_u32_;
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (em_collection_cs_->lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8].element_valid_ &&
            semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].lane_semantic_ar_[t_elem_idx_u8].valid_b_)
        {
          LaneElement& t_em_elem_cs = em_collection_cs_->lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8];
          ElementSemanticPack elem_semantic =
              semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].lane_semantic_ar_[t_elem_idx_u8];
          t_em_elem_cs.route_left_dis_ = elem_semantic.route_left_dis_;
          t_em_elem_cs.is_dest_lane_ele_ = elem_semantic.is_dest_lane_ele_;
          t_em_elem_cs.lane_type_ = elem_semantic.lane_type_;
          t_em_elem_cs.element_id_ = elem_semantic.element_id_;
          t_em_elem_cs.recommend_level_ = elem_semantic.recommend_level_;
          t_em_elem_cs.relative_idx_ = elem_semantic.relative_idx_;
          t_em_elem_cs.merge_flag_ = elem_semantic.merge_flag_;
          t_em_elem_cs.direction_to_dest_ = elem_semantic.direction_to_dest_;
          t_em_elem_cs.reference_line_.is_on_route_segs_ = elem_semantic.is_on_route_segs_;
          t_em_elem_cs.reference_line_.is_in_intersection_segs_ = elem_semantic.is_in_intersection_segs_;
          // t_em_elem_cs.reference_line_.speed_limit_segs_ = elem_semantic.speed_limit_segs_;
          t_em_elem_cs.reference_line_.lane_type_segs_ = elem_semantic.lane_type_segs_;
          t_em_elem_cs.reference_line_.slope_segs_ = elem_semantic.slope_segs_;
          t_em_elem_cs.reference_line_.lane_transition_dir_segs_ = elem_semantic.lane_transition_dir_segs_;
          t_em_elem_cs.reference_line_.super_elevation_segs_ = elem_semantic.super_elevation_segs_;
          t_em_elem_cs.reference_line_.arrow_segs_ = elem_semantic.arrow_segs_;
          t_em_elem_cs.reference_line_.special_segs_ = elem_semantic.special_segs_;
          // t_em_elem_cs.reference_line_.agents_projected_traj_ = elem_semantic.agents_projected_traj_;
          if (!t_em_elem_cs.reference_line_.curvature_segs_[0].valid_)
          {
            t_em_elem_cs.reference_line_.curvature_segs_ = elem_semantic.curvature_segs_;
          }
          t_em_elem_cs.left_boundary_.segs_ = elem_semantic.left_segs_;
          t_em_elem_cs.right_boundary_.segs_ = elem_semantic.right_segs_;
        }
      }
    }
  }
}

void Hlmf::GetDbgData(EmPreDbgData& dbg_data_cs)
{
  // dbg_data_cs.ledveh_cs_.valid_b_ = bc::true_v;
  // dbg_data_cs.ledveh_cs_.start_idx_ = 0;
  // dbg_data_cs.ledveh_cs_.end_idx_ = 99;
  // dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_ = map_ar;
  // dbg_data_cs.ledveh_cs_.right_cs_.dy_ar_ = ref_ar;
  // dbg_data_cs.ledveh_cs_.center_cs_.dy_ar_ = map_ar;
}
}  // namespace environment_model
}  // namespace zone
