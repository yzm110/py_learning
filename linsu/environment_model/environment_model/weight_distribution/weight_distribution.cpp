#include "weight_distribution.h"

namespace zone {
namespace environment_model {

void WeightDistribution::Run(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                             const BoundaryPack& boundary_pack, const EmParam& param_st,
                             const SemanticPack& semantic_pack, EgoPoseCollection& ego_motion_cs,
                             WeightPack& weight_pack)
{
  /** 0. Initialize weight_pack. The validity flag of each lane/element in weight_pack is calculated here */
  Initialize(boundary_pack, param_st, weight_pack);

  /** 0.5 Preprocess: Calculate Length Pack for scenario and quality_length*/
  CalcLengthPack(x_ref, boundary_pack);

  /** 1. Calc weight according to validity, road structure and pre-defined priority */
  CalcRawWeight(boundary_pack, param_st, weight_pack);

  /** 2. Calc weight according to current scenario */
  CalcScenarioWeight(x_ref, boundary_pack, param_st, semantic_pack, weight_pack);

  /** 3. Calc weight according to source quality */
  CalcQualityWeight(x_ref, boundary_pack, param_st, ego_motion_cs, weight_pack);

  /** 4. Normalize weight over each source for each point */
  Normalize(weight_pack);
}
/**Initialize*/
void WeightDistribution::Initialize(const BoundaryPack& boundary_pack, const EmParam& param_st, WeightPack& weight_pack)
{
  /** Check lane/element validity from boundary_pack, and set corresponding valid flag in weight_pack.
   *  If any lane/element is invalid, set valid flag as False and weights as default value.
   *  If true, also give default valid value.
   */
  /** Set weight_pack's validity from boundary_pack
   *  According to the order of lane, element,left/right boundary, per/map/ldveh/ref source
   */
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    weight_pack.lane_weight_ar_[t_lane_idx_u8].valid_b_ = boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_;
    for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
    {
      ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
      ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
      t_ele_wei_ar.valid_b_ = t_ele_pack_ar.valid_b_;
      t_ele_wei_ar.left_weight_ar_.valid_b_ = t_ele_pack_ar.left_pack_cs_.valid_b_;
      t_ele_wei_ar.right_weight_ar_.valid_b_ = t_ele_pack_ar.right_pack_cs_.valid_b_;
    }
    /**
     * From lane to element to boundary, if invalid, set weight of data of corresponding source to 0
     */
    bc::float32_t t_w_inv_def_f = param_st.wei_param.p_w_raw_init_invalid_f;
    bc::float32_t t_w_val_def_f = param_st.wei_param.p_w_raw_init_valid_f;
    if (!weight_pack.lane_weight_ar_[t_lane_idx_u8].valid_b_)
    {
      LaneWeightInit(t_w_inv_def_f, weight_pack.lane_weight_ar_[t_lane_idx_u8]);
    }
    else
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
        if (!t_ele_wei_ar.valid_b_)
        {
          ElementWeightInit(t_w_inv_def_f, t_ele_wei_ar);
        }
        else
        {
          ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
          if (!t_ele_pack_ar.left_pack_cs_.valid_b_)
          {
            BoundaryWeightInit(t_w_inv_def_f, t_ele_wei_ar.left_weight_ar_);
          }
          else
          {
            /**need to check each source's validity*/
            BoundaryInit(t_ele_pack_ar.left_pack_cs_, t_w_inv_def_f, t_w_val_def_f, t_ele_wei_ar.left_weight_ar_);
          }
          if (!t_ele_pack_ar.right_pack_cs_.valid_b_)
          {
            BoundaryWeightInit(t_w_inv_def_f, t_ele_wei_ar.right_weight_ar_);
          }
          else
          {
            BoundaryInit(t_ele_pack_ar.right_pack_cs_, t_w_inv_def_f, t_w_val_def_f, t_ele_wei_ar.right_weight_ar_);
          }
        }
      }
    }
  }
}

void WeightDistribution::LaneWeightInit(const bc::float32_t w_def_f, LaneWeight& lane_weight)
{
  for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
  {
    ElementWeightInit(w_def_f, lane_weight.element_weight_ar_[t_ele_idx_u8]);
  }
}
void WeightDistribution::ElementWeightInit(const bc::float32_t w_def_f, ElementWeight& ele_weight)
{
  BoundaryWeightInit(w_def_f, ele_weight.left_weight_ar_);
  BoundaryWeightInit(w_def_f, ele_weight.right_weight_ar_);
}
void WeightDistribution::BoundaryWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  BoundaryRawWeightInit(w_def_f, boundary_weight);
  BoundaryScenWeightInit(w_def_f, boundary_weight);
  BoundaryQualityWeightInit(w_def_f, boundary_weight);
  BoundaryTotalWeightInit(w_def_f, boundary_weight);
  BoundaryNormWeightInit(w_def_f, boundary_weight);
}
void WeightDistribution::BoundaryRawWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_raw_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_raw_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_raw_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_raw_f_ = w_def_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c0_weight_.w_map_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c1_weight_.w_per_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c1_weight_.w_map_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_raw_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_raw_f_ = w_def_f;
}
void WeightDistribution::BoundaryScenWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_scenario_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_scenario_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_scenario_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_scenario_f_ = w_def_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c0_weight_.w_map_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c1_weight_.w_per_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c1_weight_.w_map_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_scenario_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_scenario_f_ = w_def_f;
}
void WeightDistribution::BoundaryQualityWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_quality_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_quality_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ = w_def_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_quality_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_quality_f_ = w_def_f;
}
void WeightDistribution::BoundaryTotalWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_total_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_total_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_total_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_total_f_ = w_def_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_total_f_ = w_def_f;
  boundary_weight.c0_weight_.w_map_cs_.w_total_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_total_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_total_f_ = w_def_f;
  boundary_weight.c1_weight_.w_per_cs_.w_total_f_ = w_def_f;
  boundary_weight.c1_weight_.w_map_cs_.w_total_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_total_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_total_f_ = w_def_f;
}
void WeightDistribution::BoundaryNormWeightInit(const bc::float32_t w_def_f, BoundaryWeight& boundary_weight)
{
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_norm_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_norm_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_norm_f_ = w_def_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_norm_f_ = w_def_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c0_weight_.w_map_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c1_weight_.w_per_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c1_weight_.w_map_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_norm_f_ = w_def_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_norm_f_ = w_def_f;
}
void WeightDistribution::BoundaryInit(const BoundaryPointPack& boundary_point_pack, const bc::float32_t inv_weight,
                                      const bc::float32_t val_weight, BoundaryWeight& boundary_weight)
{

  if (!boundary_point_pack.per_cs_.valid_b_)
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(inv_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_);
    }
    SourceBoundaryInit(inv_weight, boundary_weight.c0_weight_.w_per_cs_);
    SourceBoundaryInit(inv_weight, boundary_weight.c1_weight_.w_per_cs_);
  }
  else
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(val_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_);
    }
    SourceBoundaryInit(val_weight, boundary_weight.c0_weight_.w_per_cs_);
    SourceBoundaryInit(val_weight, boundary_weight.c1_weight_.w_per_cs_);
  }
  if (!boundary_point_pack.map_cs_.valid_b_)
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(inv_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_);
    }
    SourceBoundaryInit(inv_weight, boundary_weight.c0_weight_.w_map_cs_);
    SourceBoundaryInit(inv_weight, boundary_weight.c1_weight_.w_map_cs_);
  }
  else
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(val_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_);
    }
    SourceBoundaryInit(val_weight, boundary_weight.c0_weight_.w_map_cs_);
    SourceBoundaryInit(val_weight, boundary_weight.c1_weight_.w_map_cs_);
  }
  if (!boundary_point_pack.ldveh_cs_.valid_b_)
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(inv_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_);
    }
    SourceBoundaryInit(inv_weight, boundary_weight.c0_weight_.w_ldveh_cs_);
    SourceBoundaryInit(inv_weight, boundary_weight.c1_weight_.w_ldveh_cs_);
  }
  else
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(val_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_);
    }
    SourceBoundaryInit(val_weight, boundary_weight.c0_weight_.w_ldveh_cs_);
    SourceBoundaryInit(val_weight, boundary_weight.c1_weight_.w_ldveh_cs_);
  }
  if (!boundary_point_pack.ref_cs_.valid_b_)
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(inv_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_);
    }
    SourceBoundaryInit(inv_weight, boundary_weight.c0_weight_.w_ref_cs_);
    SourceBoundaryInit(inv_weight, boundary_weight.c1_weight_.w_ref_cs_);
  }
  else
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      SourceBoundaryInit(val_weight, boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_);
    }
    SourceBoundaryInit(val_weight, boundary_weight.c0_weight_.w_ref_cs_);
    SourceBoundaryInit(val_weight, boundary_weight.c1_weight_.w_ref_cs_);
  }
}
void WeightDistribution::SourceBoundaryInit(const bc::float32_t w_def_f, WeightDist& weight_dist)
{
  weight_dist.w_raw_f_ = w_def_f;
  weight_dist.w_scenario_f_ = w_def_f;
  weight_dist.w_quality_f_ = w_def_f;
  weight_dist.w_total_f_ = w_def_f;
  weight_dist.w_norm_f_ = w_def_f;
}

/**CalcLengthPack*/
void WeightDistribution::CalcLengthPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                        const BoundaryPack& boundary_pack)
{
  BoundaryPoint left_ldveh = boundary_pack.lane_pack_ar_[2].element_pack_ar_[0].left_pack_cs_.ldveh_cs_;
  BoundaryPoint right_ldveh = boundary_pack.lane_pack_ar_[2].element_pack_ar_[0].right_pack_cs_.ldveh_cs_;
  if (left_ldveh.valid_b_)
  {
    ldveh_cen_pt_pack_cs.valid_b_ = left_ldveh.valid_b_;
    ldveh_cen_pt_pack_cs.dy_start_idx_u8_ = left_ldveh.dy_start_idx_u8_;
    ldveh_cen_pt_pack_cs.dy_end_idx_u8_ = left_ldveh.dy_end_idx_u8_;
    ldveh_cen_pt_pack_cs.total_valid_num_u8_ = left_ldveh.total_valid_num_u8_;
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      ldveh_cen_pt_pack_cs.dy_ar_[t_pt_idx_u8] =
          (left_ldveh.dy_ar_[t_pt_idx_u8] + right_ldveh.dy_ar_[t_pt_idx_u8]) * 0.5f;
    }
    CalcSrcBoundaryLenPack(
        x_ref, ldveh_cen_pt_pack_cs.dy_ar_,
        length_pack.lane_length_pack_[2].element_length_pack_[0].left_boundary_length_pack_.ldveh_boundary_length_);
    length_pack.lane_length_pack_[2].element_length_pack_[0].right_boundary_length_pack_.ldveh_boundary_length_ =
        length_pack.lane_length_pack_[2].element_length_pack_[0].left_boundary_length_pack_.ldveh_boundary_length_;
  }
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
        ElementLengthPack& t_ele_len_pack_ar =
            length_pack.lane_length_pack_[t_lane_idx_u8].element_length_pack_[t_ele_idx_u8];
        if (t_ele_pack_ar.valid_b_)
        {
          if (t_ele_pack_ar.left_pack_cs_.valid_b_)
          {
            CalcBoundaryLengthPack(x_ref, t_ele_pack_ar.left_pack_cs_, t_ele_len_pack_ar.left_boundary_length_pack_);
          }
          if (t_ele_pack_ar.right_pack_cs_.valid_b_)
          {
            CalcBoundaryLengthPack(x_ref, t_ele_pack_ar.right_pack_cs_, t_ele_len_pack_ar.right_boundary_length_pack_);
          }
        }
      }
    }
  }
}
void WeightDistribution::CalcBoundaryLengthPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                                const BoundaryPointPack& boundary_point_pack,
                                                BoundaryLengthPack& boundary_length_pack)
{
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    CalcSrcBoundaryLenPack(x_ref, boundary_point_pack.per_cs_.dy_ar_, boundary_length_pack.per_boundary_length_);
  }
  if (boundary_point_pack.map_cs_.valid_b_)
  {
    CalcSrcBoundaryLenPack(x_ref, boundary_point_pack.map_cs_.dy_ar_, boundary_length_pack.map_boundary_length_);
  }
  if (boundary_point_pack.ref_cs_.valid_b_)
  {
    CalcSrcBoundaryLenPack(x_ref, boundary_point_pack.ref_cs_.dy_ar_, boundary_length_pack.ref_boundary_length_);
  }
}
void WeightDistribution::CalcSrcBoundaryLenPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                                const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_dy,
                                                bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length)
{

  /**Calculate every point's length*/
  for (bc::uint8_t t_pt_idx_u8 = kFrontStartIdx; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    if (t_pt_idx_u8 == kFrontStartIdx)
    {
      boundary_length[t_pt_idx_u8] = 0.f;
    }
    else
    {
      boundary_length[t_pt_idx_u8] =
          boundary_length[t_pt_idx_u8 - 1] + CalcSquRoot(x_ref[t_pt_idx_u8], x_ref[t_pt_idx_u8 - 1],
                                                         boundary_dy[t_pt_idx_u8], boundary_dy[t_pt_idx_u8 - 1]);
    }
  }
  for (bc::int8_t t_pt_idx_s8 = kFrontStartIdx - 1; t_pt_idx_s8 >= 0; t_pt_idx_s8--)
  {
    boundary_length[t_pt_idx_s8] =
        boundary_length[t_pt_idx_s8 + 1] -
        CalcSquRoot(x_ref[t_pt_idx_s8], x_ref[t_pt_idx_s8 + 1], boundary_dy[t_pt_idx_s8], boundary_dy[t_pt_idx_s8 + 1]);
  }
}

/**CalcRawWeight*/
void WeightDistribution::CalcRawWeight(const BoundaryPack& boundary_pack, const EmParam& param_st,
                                       WeightPack& weight_pack)
{
  /** Calc RawWeight and put result into w_raw. */
  /** 1. Ego lane raw weight distribution. */
  RawWeightEgo(param_st, weight_pack.lane_weight_ar_[kHostLane]);

  /** 2. Left lane raw weight distribution. */
  if (boundary_pack.lane_pack_ar_[kLeftLane].valid_b_ == bc::true_v)
  {
    RawWeightNeighbor(boundary_pack.lane_pack_ar_[kLeftLane], boundary_pack.lane_pack_ar_[kHostLane], param_st,
                      kLeftBoundary, kLeftLane, weight_pack.lane_weight_ar_[kLeftLane]);
  }
  /** Nothing in Else{}, because default values have been set in Initialize(). */

  /** 3. Right lane raw weight distribution. */
  if (boundary_pack.lane_pack_ar_[kRightLane].valid_b_ == bc::true_v)
  {
    RawWeightNeighbor(boundary_pack.lane_pack_ar_[kRightLane], boundary_pack.lane_pack_ar_[kHostLane], param_st,
                      kRightBoundary, kRightLane, weight_pack.lane_weight_ar_[kRightLane]);
  }

  /** 4. Left Left lane raw weight distribution. */
  if (boundary_pack.lane_pack_ar_[kLLLane].valid_b_ == bc::true_v)
  {
    RawWeightNeighbor(boundary_pack.lane_pack_ar_[kLLLane], boundary_pack.lane_pack_ar_[kLeftLane], param_st,
                      kLeftBoundary, kLLLane, weight_pack.lane_weight_ar_[kLLLane]);
  }

  /** 5. Right right lane raw weight distribution. */
  if (boundary_pack.lane_pack_ar_[kRRLane].valid_b_ == bc::true_v)
  {
    RawWeightNeighbor(boundary_pack.lane_pack_ar_[kRRLane], boundary_pack.lane_pack_ar_[kRightLane], param_st,
                      kRightBoundary, kRRLane, weight_pack.lane_weight_ar_[kRRLane]);
  }
}
void WeightDistribution::RawWeightEgo(const EmParam& param_st, LaneWeight& lane_weight)
{
  /** 1. Loop for all valid elements.*/

  /** 2. Fill in weight according to parameters. */
  if (lane_weight.valid_b_)
  {
    /** host lane itself has split*/
    bc::bool_t t_lane_has_split_b = CheckLaneSelfSplit(lane_weight);
    if (t_lane_has_split_b)
    {
      RawLaneWeightSet(param_st.wei_param.p_w_raw_host_per_split_f, param_st.wei_param.p_w_raw_map_split_f,
                       param_st.wei_param.p_w_raw_host_ldveh_split_f, param_st.wei_param.p_w_raw_host_ref_split_f,
                       lane_weight);
    }
    /** host lane itself dose not have split*/
    else
    {
      RawElementWeightSet(param_st.wei_param.p_w_raw_host_per_non_split_f, param_st.wei_param.p_w_raw_map_non_split_f,
                          param_st.wei_param.p_w_raw_host_ldveh_non_split_f,
                          param_st.wei_param.p_w_raw_host_ref_non_split_f, lane_weight.element_weight_ar_[0]);
    }
  }
}
void WeightDistribution::RawWeightNeighbor(const LanePack& lane_pack_tar, const LanePack& lane_pack_ref,
                                           const EmParam& param_st, const bc::uint8_t dir_u8,
                                           const bc::uint8_t tar_lane_idx, LaneWeight& lane_weight)
{
  /** 1. Check if ref lane and target lane splits according to map info. */
  /** 1.1 Choose ref element and target element by dir_u8. If target lane is on the left side, the ref element should
   * be
   * the left element on ref lane, and the target element should be the right element on target lane(always element[0]
   * according to the latest definition of element).*/

  /** 1.2.Check map info on both element. If both valid, check if the difference between them is in a certain
   * tolerence.
   * The two lanes are splited when the difference is too large.*/

  /** 2. Fill in weights. If spliting, we give belief to map info, otherwise we trust ref(As to neighbor lane,
   * ref means the fusion result of ref lane boundary).
   * TODO: This can be handled as a interpolation regarding the value of difference in the future. */
  bc::float32_t t_w_raw_per_split_f = 1.f;
  bc::float32_t t_w_raw_ref_split_f = 1.f;
  bc::float32_t t_w_raw_per_non_split_f = 1.f;
  bc::float32_t t_w_raw_ref_non_split_f = 1.f;
  bc::float32_t t_w_raw_ldveh_f = 1.f;
  if (tar_lane_idx == kLeftLane || tar_lane_idx == kRightLane)
  {
    t_w_raw_per_split_f = param_st.wei_param.p_w_raw_rl_per_split_f;
    t_w_raw_ref_split_f = param_st.wei_param.p_w_raw_rl_ref_split_f;
    t_w_raw_per_non_split_f = param_st.wei_param.p_w_raw_rl_per_non_split_f;
    t_w_raw_ref_non_split_f = param_st.wei_param.p_w_raw_rl_ref_non_split_f;
    t_w_raw_ldveh_f = param_st.wei_param.p_w_raw_neigh_ldveh_non_split_f;
  }
  else if (tar_lane_idx == kLLLane || tar_lane_idx == kRRLane)
  {
    t_w_raw_per_split_f = param_st.wei_param.p_w_raw_rrll_per_split_f;
    t_w_raw_ref_split_f = param_st.wei_param.p_w_raw_rrll_ref_split_f;
    t_w_raw_per_non_split_f = param_st.wei_param.p_w_raw_rrll_per_non_split_f;
    t_w_raw_ref_non_split_f = param_st.wei_param.p_w_raw_rrll_ref_non_split_f;
    t_w_raw_ldveh_f = param_st.wei_param.p_w_raw_neigh_ldveh_non_split_f;
  }
  /** check validity */
  if (lane_pack_tar.valid_b_ && lane_pack_ref.valid_b_)
  {
    /**judge whether lane itself has split and set weight assignment*/
    bc::bool_t t_lane_has_split_b = CheckLaneSelfSplit(lane_weight);
    if (t_lane_has_split_b)
    {
      RawLaneWeightSet(t_w_raw_per_split_f, param_st.wei_param.p_w_raw_map_split_f, t_w_raw_ldveh_f,
                       t_w_raw_ref_split_f, lane_weight);
    }
    else
    {
      RawElementWeightSet(t_w_raw_per_non_split_f, param_st.wei_param.p_w_raw_map_non_split_f, t_w_raw_ldveh_f,
                          t_w_raw_ref_non_split_f, lane_weight.element_weight_ar_[0]);
    }
    BoundaryPoint t_boundary_point_tar;
    BoundaryPoint t_boundary_point_ref;
    /**  target lane is on the right of ref lane
     * eg: target lane is right lane, ref lane is host lane
     * find the nearest element of ref_lane */
    if (dir_u8 == kRightBoundary)
    {
      t_boundary_point_tar = lane_pack_tar.element_pack_ar_[0].left_pack_cs_.map_cs_;
      bc::uint8_t t_nearest_ele_u8 = 0;
      bc::float32_t t_min_delta_dy_f = 1000;
      bc::uint8_t t_min_end_idx_u8 = kMaxElemNumInOneLane - 1;
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].valid_b_)
        {
          if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].right_pack_cs_.map_cs_.dy_end_idx_u8_ < t_min_end_idx_u8)
          {
            t_min_end_idx_u8 = lane_pack_ref.element_pack_ar_[t_ele_idx_u8].right_pack_cs_.map_cs_.dy_end_idx_u8_;
          }
        }
      }
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].valid_b_)
        {
          BoundaryPoint t_cur_boundary_point_ref = lane_pack_ref.element_pack_ar_[t_ele_idx_u8].right_pack_cs_.map_cs_;
          bc::float32_t t_cur_delta_dy_f = t_cur_boundary_point_ref.dy_ar_[t_min_end_idx_u8];
          if (t_cur_delta_dy_f < t_min_delta_dy_f)
          {
            t_min_delta_dy_f = t_cur_delta_dy_f;
            t_nearest_ele_u8 = t_ele_idx_u8;
          }
        }
      }
      t_boundary_point_ref = lane_pack_ref.element_pack_ar_[t_nearest_ele_u8].right_pack_cs_.map_cs_;
    }
    /** target lane is on the left of ref lane
     * eg: target lane is left lane, ref lane is host lane**/
    else
    {
      t_boundary_point_tar = lane_pack_tar.element_pack_ar_[0].right_pack_cs_.map_cs_;
      bc::uint8_t t_nearest_ele_u8 = 0;
      bc::float32_t t_max_delta_dy_f = -1000;
      bc::uint8_t t_min_end_idx_u8 = kMaxElemNumInOneLane - 1;
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].valid_b_)
        {
          if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].left_pack_cs_.map_cs_.dy_end_idx_u8_ < t_min_end_idx_u8)
          {
            t_min_end_idx_u8 = lane_pack_ref.element_pack_ar_[t_ele_idx_u8].left_pack_cs_.map_cs_.dy_end_idx_u8_;
          }
        }
      }
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (lane_pack_ref.element_pack_ar_[t_ele_idx_u8].valid_b_)
        {
          BoundaryPoint t_cur_boundary_point_ref = lane_pack_ref.element_pack_ar_[t_ele_idx_u8].left_pack_cs_.map_cs_;
          bc::float32_t t_cur_delta_dy_f = t_cur_boundary_point_ref.dy_ar_[t_min_end_idx_u8];
          if (t_cur_delta_dy_f > t_max_delta_dy_f)
          {
            t_max_delta_dy_f = t_cur_delta_dy_f;
            t_nearest_ele_u8 = t_ele_idx_u8;
          }
        }
      }
      t_boundary_point_ref = lane_pack_ref.element_pack_ar_[t_nearest_ele_u8].left_pack_cs_.map_cs_;
    }
    bc::uint8_t t_start_idx_u8 = t_boundary_point_tar.dy_start_idx_u8_;
    bc::uint8_t t_end_idx_u8 = t_boundary_point_tar.dy_end_idx_u8_;
    /** if the difference of start or end idx dy is larger than the tolarance we set,
     * we suppose the two boundary is not the same boundary
     * that's target lane has split with ref lane*/
    if (fabsf(t_boundary_point_ref.dy_ar_[t_start_idx_u8] - t_boundary_point_tar.dy_ar_[t_start_idx_u8]) >
            param_st.wei_param.p_raw_dy_split_tolarance_f ||
        fabsf(t_boundary_point_ref.dy_ar_[t_end_idx_u8] - t_boundary_point_tar.dy_ar_[t_end_idx_u8]) >
            param_st.wei_param.p_raw_dy_split_tolarance_f)
    {
      /** TODO: per_lane_split --> interpolate from 1 to 0.1 according to above delta_dy(small to large)*/
      RawLaneWeightSet(param_st.wei_param.p_w_raw_per_lane_split_f, param_st.wei_param.p_w_raw_map_lane_split_f,
                       param_st.wei_param.p_w_raw_ldveh_lane_split_f, param_st.wei_param.p_w_raw_ref_lane_split_f,
                       lane_weight);
    }
  }
  else
  {
  }
}

void WeightDistribution::RawLaneWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f,
                                          const bc::float32_t w_ldveh_f, const bc::float32_t w_ref_f,
                                          LaneWeight& lane_weight)
{
  for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
  {
    RawElementWeightSet(w_per_f, w_map_f, w_ldveh_f, w_ref_f, lane_weight.element_weight_ar_[t_ele_idx_u8]);
  }
}
void WeightDistribution::RawElementWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f,
                                             const bc::float32_t w_ldveh_f, const bc::float32_t w_ref_f,
                                             ElementWeight& element_weight)
{
  if (element_weight.left_weight_ar_.valid_b_)
  {
    RawBoundaryWeightSet(w_per_f, w_map_f, w_ldveh_f, w_ref_f, element_weight.left_weight_ar_);
  }
  if (element_weight.right_weight_ar_.valid_b_)
  {
    RawBoundaryWeightSet(w_per_f, w_map_f, w_ldveh_f, w_ref_f, element_weight.right_weight_ar_);
  }
}

void WeightDistribution::RawBoundaryWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f,
                                              const bc::float32_t w_ldveh_f, const bc::float32_t w_ref_f,
                                              BoundaryWeight& boundary_weight)
{
  /** need to consider the split of lane and lane*/

  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_raw_f_ *= w_per_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_raw_f_ *= w_map_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_raw_f_ *= w_ldveh_f;
    boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_raw_f_ *= w_ref_f;
  }
  boundary_weight.c0_weight_.w_per_cs_.w_raw_f_ *= w_per_f;
  boundary_weight.c0_weight_.w_map_cs_.w_raw_f_ *= w_map_f;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_raw_f_ *= w_ldveh_f;
  boundary_weight.c0_weight_.w_ref_cs_.w_raw_f_ *= w_ref_f;
  boundary_weight.c1_weight_.w_per_cs_.w_raw_f_ *= w_per_f;
  boundary_weight.c1_weight_.w_map_cs_.w_raw_f_ *= w_map_f;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_raw_f_ *= w_ldveh_f;
  boundary_weight.c1_weight_.w_ref_cs_.w_raw_f_ *= w_ref_f;
}
bc::bool_t WeightDistribution::CheckLaneSelfSplit(const LaneWeight& lane_weight)
{
  bc::uint8_t t_valid_ele_num_u8 = 0;
  for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
  {
    if (lane_weight.element_weight_ar_[t_ele_idx_u8].valid_b_)
    {
      t_valid_ele_num_u8++;
    }
  }
  if (t_valid_ele_num_u8 > 1)
  {
    return bc::true_v;
  }
  else
  {
    return bc::false_v;
  }
}

/**CalcScenarioWeight*/
void WeightDistribution::CalcScenarioWeight(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                            const BoundaryPack& boundary_pack, const EmParam& param_st,
                                            const SemanticPack& semantic_pack, WeightPack& weight_pack)
{
  /** Calc ScenarioWeight and put result into w_scenario. */
  /** 1. Select the nearest Scenario type and its start/end point from map_guide_pts_ in map data */

  /** 2. Calc weights under each Scenario in a Switch-Case loop. */
  /// while in map, do init --- put per and ldveh point weight 0
  if (semantic_pack.map_geofence_.is_in_map_)
  {
    bc::bool_t t_map_end_b = bc::false_v;
    bc::float32_t start_s = 1000.f;
    bc::float32_t end_s = 1000.f;
    for (bc::uint8_t map_guide_idx = 0; map_guide_idx < kMaxMapGuidePtNum; map_guide_idx++)
    {
      start_s = semantic_pack.map_guide_pts_[map_guide_idx].start_s_;
      end_s = semantic_pack.map_guide_pts_[map_guide_idx].end_s_;
      if (semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kRemainDistance && end_s < x_ref[99])
      {
        t_map_end_b = bc::true_v;
      }
    }
    if (t_map_end_b)
    {
      for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
      {
        if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
            ElementLengthPack t_ele_len_pack_ar =
                length_pack.lane_length_pack_[t_lane_idx_u8].element_length_pack_[t_ele_idx_u8];
            ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
            if (t_ele_pack_ar.valid_b_)
            {
              if (t_ele_wei_ar.left_weight_ar_.valid_b_ && t_ele_pack_ar.left_pack_cs_.map_cs_.valid_b_)
              {
                if (t_ele_pack_ar.left_pack_cs_.map_cs_.dy_end_idx_u8_ < kFrontStartIdx)
                {
                  t_ele_wei_ar.left_weight_ar_.c0_weight_.w_map_cs_.w_scenario_f_ = 0.f;
                }
                for (bc::uint8_t t_pt_idx_u8 = t_ele_pack_ar.left_pack_cs_.map_cs_.dy_end_idx_u8_ + 1;
                     t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
                {
                  t_ele_wei_ar.left_weight_ar_.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_scenario_f_ = 0.f;
                }
              }
              if (t_ele_wei_ar.right_weight_ar_.valid_b_ && t_ele_pack_ar.right_pack_cs_.map_cs_.valid_b_)
              {
                if (t_ele_pack_ar.right_pack_cs_.map_cs_.dy_end_idx_u8_ < kFrontStartIdx)
                {
                  t_ele_wei_ar.right_weight_ar_.c0_weight_.w_map_cs_.w_scenario_f_ = 0.f;
                }
                for (bc::uint8_t t_pt_idx_u8 = t_ele_pack_ar.right_pack_cs_.map_cs_.dy_end_idx_u8_ + 1;
                     t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
                {
                  t_ele_wei_ar.right_weight_ar_.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_scenario_f_ = 0.f;
                }
              }
            }
          }
        }
      }
    }
    else
    {
      for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
      {
        if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
            ElementLengthPack t_ele_len_pack_ar =
                length_pack.lane_length_pack_[t_lane_idx_u8].element_length_pack_[t_ele_idx_u8];
            ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
            if (t_ele_pack_ar.valid_b_)
            {
              if (t_ele_wei_ar.left_weight_ar_.valid_b_ && t_ele_pack_ar.left_pack_cs_.map_cs_.valid_b_)
              {
                ScenSourceWeightInterpCurv(t_ele_len_pack_ar.left_boundary_length_pack_.ldveh_boundary_length_, 0, 0,
                                           param_st.wei_param.p_w_sce_ldveh_ramp_f,
                                           param_st.wei_param.p_w_sce_ldveh_out_ramp_f, t_ele_wei_ar.left_weight_ar_,
                                           kSourceLdveh);
                // ScenRampBoundaryWeightSet(t_ele_len_pack_ar.left_boundary_length_pack_, 0, 0,
                // t_ele_pack_ar.left_pack_cs_,
                //                           param_st.wei_param, t_ele_wei_ar.left_weight_ar_);
                ScenCheckSourceSetValid(kSourcePer, 1.f, t_ele_wei_ar.left_weight_ar_.c0_weight_);
                ScenCheckSourceSetValid(kSourcePer, 0.f, t_ele_wei_ar.left_weight_ar_.c1_weight_);
                ScenCheckSourceSetValid(kSourceLdveh, 0.f, t_ele_wei_ar.left_weight_ar_.c0_weight_);
                // ScenCheckSourceSetValid(kSourceLdveh, 0.f, t_ele_wei_ar.left_weight_ar_.c1_weight_);
              }
              if (t_ele_wei_ar.right_weight_ar_.valid_b_ && t_ele_pack_ar.right_pack_cs_.map_cs_.valid_b_)
              {
                ScenSourceWeightInterpCurv(t_ele_len_pack_ar.right_boundary_length_pack_.ldveh_boundary_length_, 0, 0,
                                           param_st.wei_param.p_w_sce_ldveh_ramp_f,
                                           param_st.wei_param.p_w_sce_ldveh_out_ramp_f, t_ele_wei_ar.right_weight_ar_,
                                           kSourceLdveh);
                // ScenRampBoundaryWeightSet(t_ele_len_pack_ar.right_boundary_length_pack_, 0, 0,
                //                           t_ele_pack_ar.right_pack_cs_, param_st.wei_param,
                //                           t_ele_wei_ar.right_weight_ar_);
                ScenCheckSourceSetValid(kSourcePer, 1.f, t_ele_wei_ar.right_weight_ar_.c0_weight_);
                ScenCheckSourceSetValid(kSourcePer, 0.f, t_ele_wei_ar.right_weight_ar_.c1_weight_);
                ScenCheckSourceSetValid(kSourceLdveh, 0.f, t_ele_wei_ar.right_weight_ar_.c0_weight_);
                ScenCheckSourceSetValid(kSourceLdveh, 0.f, t_ele_wei_ar.right_weight_ar_.c1_weight_);
              }
            }
          }
        }
      }
      bc::float32_t start_s = 1000.f;
      bc::float32_t end_s = 1000.f;
      bc::float32_t special_point_s = 1000.f;
      /// Intersection
      bc::bool_t t_intersection_b = bc::false_v;
      bc::bool_t t_guide_intersection_b = bc::false_v;
      for (bc::uint8_t intersection_idx = 0;
           intersection_idx < kMaxLanePropertySegsInOneLaneElement && !t_intersection_b; intersection_idx++)
      {
        t_intersection_b = semantic_pack.map_semantic_ar_[kHostLane]
                               .lane_semantic_ar_[0]
                               .is_in_intersection_segs_[intersection_idx]
                               .valid_ &&
                           semantic_pack.map_semantic_ar_[kHostLane]
                                   .lane_semantic_ar_[0]
                                   .is_in_intersection_segs_[intersection_idx]
                                   .start_s_ < 100.f &&
                           semantic_pack.map_semantic_ar_[kHostLane]
                                   .lane_semantic_ar_[0]
                                   .is_in_intersection_segs_[intersection_idx]
                                   .start_s_ > -100.f;
      }
      for (bc::uint8_t map_guide_idx = 0; map_guide_idx < kMaxMapGuidePtNum && !t_guide_intersection_b; map_guide_idx++)
      {
        t_guide_intersection_b = (semantic_pack.map_guide_pts_[map_guide_idx].valid_ &&
                                  semantic_pack.map_guide_pts_[map_guide_idx].start_s_ <= 200) &&
                                 semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kIntersection;
      }
      if (t_intersection_b || t_guide_intersection_b || semantic_pack.map_geofence_.is_on_intersection_)
      {
        bc::float32_t t_wei_per_c0_f = 0.f;
        for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
            ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
            {
              ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f,
                                      t_ele_wei_ar.left_weight_ar_.point_weight_ar_[t_pt_idx_u8]);
            }
            ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f, t_ele_wei_ar.left_weight_ar_.c0_weight_);
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
            {
              ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f,
                                      t_ele_wei_ar.right_weight_ar_.point_weight_ar_[t_pt_idx_u8]);
            }
            ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f, t_ele_wei_ar.right_weight_ar_.c0_weight_);
          }
        }
      }

      /// Highway
      if (semantic_pack.map_geofence_.is_on_ramp_)
      {
        special_point_s = 0.f;
      }
      else
      {
        /// Road level
        for (bc::uint8_t map_guide_idx = 0; map_guide_idx < kMaxMapGuidePtNum; map_guide_idx++)
        {
          start_s = semantic_pack.map_guide_pts_[map_guide_idx].start_s_;
          end_s = semantic_pack.map_guide_pts_[map_guide_idx].end_s_;
          bc::bool_t t_guide_ramp_b =
              (semantic_pack.map_guide_pts_[map_guide_idx].valid_ && start_s < 100.f && end_s > -100.f) &&
              (semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kEntranceRamp ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kExitRamp ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kSwitchEntranceRamp ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kSwitchExitRamp ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kSwitchEntranceJCT ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kSwitchExitJCT ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kRoadMerge ||
               semantic_pack.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kRoadSplit);
          if (t_guide_ramp_b)
          {
            if (start_s < 0.f && end_s > 0.f)
            {
              special_point_s = 0.f;
            }
            else if (end_s < 0.f && fabsf(end_s) < fabsf(special_point_s))
            {
              special_point_s = end_s;
            }
            else if (start_s > 0.f && fabsf(start_s) < fabsf(special_point_s))
            {
              special_point_s = start_s;
            }
          }
        }
        /// Lane level
        for (bc::uint8_t t_lane_idx_u8 = kLeftLane; t_lane_idx_u8 <= kRightLane; t_lane_idx_u8++)
        {
          if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
          {
            for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
            {
              bc::TCArray<RefLineSegLaneTransitionDir, kMaxLaneTransitionSegsInOneLaneElement>
                  lane_transition_dir_segs = semantic_pack.map_semantic_ar_[t_lane_idx_u8]
                                                 .lane_semantic_ar_[t_ele_idx_u8]
                                                 .lane_transition_dir_segs_;
              for (bc::uint8_t t_seg_idx_u8 = 0; t_seg_idx_u8 < kMaxLaneTransitionSegsInOneLaneElement; t_seg_idx_u8++)
              {
                if (lane_transition_dir_segs[t_seg_idx_u8].valid_ &&
                    lane_transition_dir_segs[t_seg_idx_u8].start_s_ < 100.f &&
                    lane_transition_dir_segs[t_seg_idx_u8].end_s_ > -100.f &&
                    (lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ == LaneTransitionDirection::kSplitToLeft ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ == LaneTransitionDirection::kSplitToRight ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ ==
                         LaneTransitionDirection::kMergeFromLeft ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ ==
                         LaneTransitionDirection::kMergeFromRight ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ == LaneTransitionDirection::kMergeToLeft ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ == LaneTransitionDirection::kMergeToRight ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ ==
                         LaneTransitionDirection::kSplitFromLeft ||
                     lane_transition_dir_segs[t_seg_idx_u8].lane_trans_dir_ ==
                         LaneTransitionDirection::kSplitFromRight))
                {
                  if (lane_transition_dir_segs[t_seg_idx_u8].start_s_ < 0.f &&
                      lane_transition_dir_segs[t_seg_idx_u8].end_s_ > 0.f)
                  {
                    special_point_s = 0.f;
                  }
                  else if (lane_transition_dir_segs[t_seg_idx_u8].end_s_ < 0.f &&
                           fabsf(lane_transition_dir_segs[t_seg_idx_u8].end_s_) < fabsf(special_point_s))
                  {
                    special_point_s = lane_transition_dir_segs[t_seg_idx_u8].end_s_;
                  }
                  else if (lane_transition_dir_segs[t_seg_idx_u8].start_s_ > 0.f &&
                           fabsf(lane_transition_dir_segs[t_seg_idx_u8].start_s_) < fabsf(special_point_s))
                  {
                    special_point_s = lane_transition_dir_segs[t_seg_idx_u8].start_s_;
                  }
                }
              }
            }
          }
        }
      }
      CurvParam<bc::float32_t, 4> t_curve_dis_cur;
      t_curve_dis_cur.x_arr = {-100.f, -kSpecialDirDistance, kSpecialDirDistance, 100.f};
      t_curve_dis_cur.y_arr = {1, 0, 0, 1};
      bc::float32_t t_wei_per_c0_f = InterpCurv(t_curve_dis_cur, special_point_s);
      for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
      {
        if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            ElementPack t_ele_pack_ar = boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8];
            ElementWeight& t_ele_wei_ar = weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8];
            if (t_ele_pack_ar.left_pack_cs_.map_cs_.valid_b_)
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
              {
                ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f,
                                        t_ele_wei_ar.left_weight_ar_.point_weight_ar_[t_pt_idx_u8]);
              }
              ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f, t_ele_wei_ar.left_weight_ar_.c0_weight_);
            }
            if (t_ele_pack_ar.right_pack_cs_.map_cs_.valid_b_)
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
              {
                ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f,
                                        t_ele_wei_ar.right_weight_ar_.point_weight_ar_[t_pt_idx_u8]);
              }
              ScenCheckSourceSetValid(kSourcePer, t_wei_per_c0_f, t_ele_wei_ar.right_weight_ar_.c0_weight_);
            }
          }
        }
      }
    }
  }
}
void WeightDistribution::ScenRampBoundaryWeightSet(const BoundaryLengthPack& boundary_length_pack,
                                                   const bc::float32_t start_s, const bc::float32_t end_s,
                                                   const BoundaryPointPack& boundary_point_pack,
                                                   const WeiDisParam& wei_param, BoundaryWeight& boundary_weight)
{
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    ScenSourceWeightInterpCurv(boundary_length_pack.per_boundary_length_, start_s, end_s, wei_param.p_w_sce_per_ramp_f,
                               wei_param.p_w_sce_per_out_ramp_f, boundary_weight, kSourcePer);
  }
  // if (boundary_point_pack.map_cs_.valid_b_)
  // {
  //   ScenSourceWeightInterpCurv(boundary_length_pack.map_boundary_length_, start_s, end_s,
  //   wei_param.p_w_sce_map_ramp_f,
  //                              wei_param.p_w_sce_map_out_ramp_f, boundary_weight, kSourceMap);
  // }
  if (boundary_point_pack.ldveh_cs_.valid_b_)
  {
    ScenSourceWeightInterpCurv(boundary_length_pack.ldveh_boundary_length_, start_s, end_s,
                               wei_param.p_w_sce_ldveh_ramp_f, wei_param.p_w_sce_ldveh_out_ramp_f, boundary_weight,
                               kSourceLdveh);
  }
  // if (boundary_point_pack.ref_cs_.valid_b_)
  // {
  //   ScenSourceWeightInterpCurv(boundary_length_pack.ref_boundary_length_, start_s, end_s,
  //   wei_param.p_w_sce_ref_ramp_f,
  //                              wei_param.p_w_sce_ref_out_ramp_f, boundary_weight, kSourceRef);
  // }
}
void WeightDistribution::ScenIntersectionBoundaryWeightSet(const BoundaryLengthPack& boundary_length_pack,
                                                           const bc::float32_t start_s, const bc::float32_t end_s,
                                                           const BoundaryPointPack& boundary_point_pack,
                                                           const WeiDisParam& wei_param,
                                                           BoundaryWeight& boundary_weight)
{
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    ScenSourceWeightInterpCurv(boundary_length_pack.per_boundary_length_, start_s, end_s,
                               wei_param.p_w_sce_per_intersection_f, wei_param.p_w_sce_per_out_intersection_f,
                               boundary_weight, kSourcePer);
  }
  // if (boundary_point_pack.map_cs_.valid_b_)
  // {
  //   ScenSourceWeightInterpCurv(boundary_length_pack.map_boundary_length_, start_s, end_s,
  //                              wei_param.p_w_sce_map_intersection_f, wei_param.p_w_sce_map_out_intersection_f,
  //                              boundary_weight, kSourceMap);
  // }
  if (boundary_point_pack.ldveh_cs_.valid_b_)
  {
    ScenSourceWeightInterpCurv(boundary_length_pack.ldveh_boundary_length_, start_s, end_s,
                               wei_param.p_w_sce_ldveh_intersection_f, wei_param.p_w_sce_ldveh_out_intersection_f,
                               boundary_weight, kSourceLdveh);
  }
  if (boundary_point_pack.ref_cs_.valid_b_)
  {
    ScenSourceWeightInterpCurv(boundary_length_pack.ref_boundary_length_, start_s, end_s,
                               wei_param.p_w_sce_ref_intersection_f, wei_param.p_w_sce_ref_out_intersection_f,
                               boundary_weight, kSourceRef);
  }
}
void WeightDistribution::ScenSourceWeightInterpCurv(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                                    const bc::float32_t start_s, const bc::float32_t end_s,
                                                    const bc::float32_t w_ramp_f, const bc::float32_t w_out_ramp_f,
                                                    BoundaryWeight& boundary_weight, bc::uint8_t weight_source)
{
  CurvParam<bc::float32_t, 4> t_curve_length_cur;
  t_curve_length_cur.x_arr = {start_s - kRangeDistance, start_s, end_s, end_s + kRangeDistance};
  t_curve_length_cur.y_arr = {w_out_ramp_f, w_ramp_f, w_ramp_f, w_out_ramp_f};
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
  {
    bc::float32_t t_w_cur_point_secn_f = InterpCurv(t_curve_length_cur, boundary_length[t_pt_idx_u8]);
    ScenCheckSourceSetValid(weight_source, t_w_cur_point_secn_f, boundary_weight.point_weight_ar_[t_pt_idx_u8]);
  }
  /// USE C0
  ScenCheckSourceSetValid(weight_source, 1.f, boundary_weight.c0_weight_);
  ScenCheckSourceSetValid(weight_source, 0.f, boundary_weight.c1_weight_);
}
void WeightDistribution::ScenCheckSourceSetValid(const bc::uint8_t weight_source, const bc::float32_t weight_scen_f,
                                                 PointWeight& point_weight)
{
  switch (weight_source)
  {
    case kSourcePer:
      point_weight.w_per_cs_.w_scenario_f_ *= weight_scen_f;
      break;
    case kSourceMap:
      point_weight.w_map_cs_.w_scenario_f_ *= weight_scen_f;
      break;
    case kSourceLdveh:
      point_weight.w_ldveh_cs_.w_scenario_f_ *= 0.f;
      break;
    case kSourceRef:
      point_weight.w_ref_cs_.w_scenario_f_ *= weight_scen_f;
      break;
    default:
      point_weight.w_per_cs_.w_scenario_f_ *= weight_scen_f;
      break;
  }
}

/**CalcQualityWeight*/
void WeightDistribution::CalcQualityWeight(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                           const BoundaryPack& boundary_pack, const EmParam& param_st,
                                           EgoPoseCollection& ego_motion_cs, WeightPack& weight_pack)
{
  /** CalcQualityWeight and put result into w_quality = w_prob * w_variance * w_type * w_length. */

  /** 1. Calc QualityWeight by boundary type. */
  QualityWeightType(boundary_pack, param_st, weight_pack);

  /** 2. Calc QualityWeight by boundary length. */
  QualityWeightLength(boundary_pack, param_st, x_ref, weight_pack);

  /** 3. Optional, calc QualityWeight by exist probability, currently for perception lane only. Use a parameter here
   * to trigger following function. */
  if (param_st.wei_param.p_do_qua_existprob_b)
  {
    QualityWeightExistProb(boundary_pack, weight_pack);
  }
  /** 4. Optional, calc QualityWeight by variance, currently for perception lane only. Use a parameter here to
   * trigger following function. */
  if (param_st.wei_param.p_do_qua_variance_b)
  {
    QualityWeightVariance(x_ref, boundary_pack, weight_pack);
  }

  QualityWeightVeh(ego_motion_cs, weight_pack);

  QualityWeightLifetime(boundary_pack, weight_pack);
}

/**QualityWeightType*/
void WeightDistribution::QualityWeightType(const BoundaryPack& boundary_pack, const EmParam& param_st,
                                           WeightPack& weight_pack)
{
  /** Weight of virtual boundary should be punished here. */
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
    {
      BoundaryPointPack left_boundary_pack =
          boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].left_pack_cs_;
      BoundaryPointPack right_boundary_pack =
          boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].right_pack_cs_;
      BoundaryWeight& left_boundary_weight =
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_;
      BoundaryWeight& right_boundary_weight =
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_;
      /** per*/
      QualityPerBoundaryType(left_boundary_pack, param_st, left_boundary_weight);
      QualityPerBoundaryType(right_boundary_pack, param_st, right_boundary_weight);

      /** map*/
      QualityMapBoundaryType(left_boundary_pack, param_st, left_boundary_weight);
      QualityMapBoundaryType(right_boundary_pack, param_st, right_boundary_weight);

      /** ref*/
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        left_boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_ref_f;
        right_boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_ref_f;
      }
      /** ldveh*/
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        left_boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_ldveh_f;
        right_boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_ldveh_f;
      }
    }
  }
}

void WeightDistribution::QualityPerBoundaryType(const BoundaryPointPack& boundary_point_pack, const EmParam& param_st,
                                                BoundaryWeight& boundary_weight)
{
  switch (boundary_point_pack.per_cs_.boundary_type_en_)
  {
    case LaneBoundaryType::kVirtual:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_virtual_f;
      }
      boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_virtual_f;
      boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_virtual_f;
      break;
    case LaneBoundaryType::kPhysical:
    case LaneBoundaryType::kSolid:
    case LaneBoundaryType::kDash:

      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_physical_f;
      }
      boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_physical_f;
      boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_physical_f;
      break;
    case LaneBoundaryType::kUnknown:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      }
      boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      break;
    default:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      }
      boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      break;
  }
}
void WeightDistribution::QualityMapBoundaryType(const BoundaryPointPack& boundary_point_pack, const EmParam& param_st,
                                                BoundaryWeight& boundary_weight)
{
  switch (boundary_point_pack.map_cs_.boundary_type_en_)
  {
    case LaneBoundaryType::kVirtual:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_virtual_f;
      }
      boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_virtual_f;
      boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_virtual_f;
      break;
    case LaneBoundaryType::kPhysical:
    case LaneBoundaryType::kSolid:
    case LaneBoundaryType::kDash:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_quality_f_ =
            param_st.wei_param.p_w_qua_type_physical_f;
      }
      boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_physical_f;
      boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_physical_f;
      break;
    case LaneBoundaryType::kUnknown:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      }
      boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      break;
    default:
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      }
      boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ = param_st.wei_param.p_w_qua_type_unk_f;
      break;
  }
}

/**QualityWeightLength*/
void WeightDistribution::QualityWeightLength(const BoundaryPack& boundary_pack, const EmParam& param_st,
                                             const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                             WeightPack& weight_pack)
{
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].valid_b_)
        {
          BoundaryWeight& left_boundary_weight =
              weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_;
          BoundaryWeight& right_boundary_weight =
              weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_;
          BoundaryPointPack left_boundary_point_pack =
              boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].left_pack_cs_;
          BoundaryPointPack right_boundary_point_pack =
              boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].right_pack_cs_;
          BoundaryLengthPack left_boundary_length_pack = length_pack.lane_length_pack_[t_lane_idx_u8]
                                                             .element_length_pack_[t_ele_idx_u8]
                                                             .left_boundary_length_pack_;
          BoundaryLengthPack right_boundary_length_pack = length_pack.lane_length_pack_[t_lane_idx_u8]
                                                              .element_length_pack_[t_ele_idx_u8]
                                                              .right_boundary_length_pack_;
          CalcLengthWeight(left_boundary_point_pack, x_ref, left_boundary_length_pack, left_boundary_weight);
          CalcLengthWeight(right_boundary_point_pack, x_ref, right_boundary_length_pack, right_boundary_weight);

          /**per*/
          // CalcPerLenWeight(left_boundary_point_pack.per_cs_, left_boundary_length_pack.per_boundary_length_,
          //                  left_boundary_weight);
          // CalcPerLenWeight(right_boundary_point_pack.per_cs_, right_boundary_length_pack.per_boundary_length_,
          //                  right_boundary_weight);
          /**per*/
          // LenQuaWeight(left_boundary_point_pack.per_cs_, left_boundary_length_pack.per_boundary_length_,
          //              left_boundary_weight);
          /**ref*/
          // CalcRefLenWeight(left_boundary_point_pack.ref_cs_, left_boundary_length_pack.ref_boundary_length_,
          //                  left_boundary_weight);
          // CalcRefLenWeight(right_boundary_point_pack.ref_cs_, right_boundary_length_pack.ref_boundary_length_,
          //                  right_boundary_weight);
          // QualityElementLength(left_boundary_point_pack.ref_cs_, left_boundary_length_pack.ref_boundary_length_,
          //                      param_st.wei_param.p_w_qua_len_ref_lower_f,
          //                      param_st.wei_param.p_w_qua_len_ref_lower_f,
          //                      param_st.wei_param.p_w_qua_len_ref_middle_f,
          //                      param_st.wei_param.p_w_qua_len_ref_upper_f,
          //                      param_st.wei_param.p_w_qua_len_invalid_f, kSourceRef, left_boundary_weight);
          // QualityElementLength(right_boundary_point_pack.ref_cs_, right_boundary_length_pack.ref_boundary_length_,
          //                      param_st.wei_param.p_w_qua_len_ref_lower_f,
          //                      param_st.wei_param.p_w_qua_len_ref_lower_f,
          //                      param_st.wei_param.p_w_qua_len_ref_middle_f,
          //                      param_st.wei_param.p_w_qua_len_ref_upper_f,
          //                      param_st.wei_param.p_w_qua_len_invalid_f, kSourceRef, right_boundary_weight);

          /**map*/
          // bc::float32_t t_w_cur_point_quality_f = param_st.wei_param.p_w_qua_len_map_upper_f;
          // for (bc::uint16_t t_pt_idx = kFrontStartIdx; t_pt_idx < kMaxNumBoundary; t_pt_idx++)
          // {
          //   left_boundary_weight.point_weight_ar_[t_pt_idx].w_map_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
          //   right_boundary_weight.point_weight_ar_[t_pt_idx].w_map_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
          // }
        }
      }
    }
  }
}
// void WeightDistribution::LenQuaWeight(const BoundaryPoint& boundary_point,
//                                       const bc::TCArray<bc::float32_t, kMaxNumBoundary> left_boundary_length,
//                                       const bc::TCArray<bc::float32_t, kMaxNumBoundary> right_boundary_length,
//                                       BoundaryWeight& left_boundary_weight, BoundaryWeight& right_boundary_weight)
// {
//   bc::uint8_t t_start_idx_u8 = boundary_point.dy_start_idx_u8_;
//   bc::uint8_t t_end_idx_u8 = boundary_point.dy_end_idx_u8_;
//   if (boundary_length[t_end_idx_u8] - boundary_length[t_start_idx_u8] < 15)
//   {
//     for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 <= kMaxNumBoundary; t_pt_idx_u8++)
//     {
//       boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= 0.2;
//     }
//   }
// }
void WeightDistribution::CalcLengthWeight(const BoundaryPointPack& boundary_point_pack,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                          const BoundaryLengthPack& boundary_length_pack,
                                          BoundaryWeight& boundary_weight)
{
  bc::float32_t t_s_per_start_f = 0.f;
  bc::float32_t t_s_per_end_f = 0.f;
  bc::float32_t t_s_per_length_f = 0.f;
  bc::float32_t t_w_ldveh_decay_by_per_len_f = 1.f;
  bc::float32_t t_per_valid_length = 40.f;
  bc::float32_t t_ldveh_dis_f = 0.f;
  if (boundary_point_pack.per_cs_.valid_b_)
  {
    t_s_per_start_f = boundary_length_pack.per_boundary_length_[boundary_point_pack.per_cs_.dy_start_idx_u8_];
    t_s_per_end_f = boundary_length_pack.per_boundary_length_[boundary_point_pack.per_cs_.dy_end_idx_u8_];
    t_s_per_length_f = t_s_per_end_f - t_s_per_start_f;
    t_s_per_length_f = std::max(t_s_per_length_f, 0.f);
    t_ldveh_dis_f = boundary_point_pack.ldveh_cs_.valid_b_
                        ? boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_end_idx_u8_]
                        : 0.f;
    t_per_valid_length = std::max(std::min(40.f, t_ldveh_dis_f), 5.f);
    t_w_ldveh_decay_by_per_len_f = LinearInterpolation(0.f, 1.0, t_per_valid_length, 0.000001, t_s_per_length_f);
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
  {
    /** PER*/
    if (boundary_point_pack.per_cs_.valid_b_)
    {
      bc::float32_t t_per_weight_f = 1.f;
      bc::uint8_t t_delta_idx_u8 = 0;
      bc::float32_t t_s_f = boundary_length_pack.per_boundary_length_[t_idx_u8];

      if (boundary_point_pack.per_cs_.dy_end_idx_u8_ >= boundary_point_pack.per_cs_.dy_start_idx_u8_)
      {
        t_delta_idx_u8 = boundary_point_pack.per_cs_.dy_end_idx_u8_ - boundary_point_pack.per_cs_.dy_start_idx_u8_;
      }
      else
      {
        t_delta_idx_u8 = 0;
      }
      bc::uint8_t t_idx_point23_u8 = (bc::uint8_t)(2 * t_delta_idx_u8 / 3);
      t_idx_point23_u8 = boundary_point_pack.per_cs_.dy_start_idx_u8_ + t_idx_point23_u8;
      if (t_idx_u8 < boundary_point_pack.per_cs_.dy_start_idx_u8_)
      {
        t_per_weight_f = exp(-(t_s_f - t_s_per_start_f) * (t_s_f - t_s_per_start_f) / 10.f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_quality_f_ *= t_per_weight_f;
      }
      else if (t_idx_u8 > t_idx_point23_u8)
      {
        bc::float32_t t_s_point32_f = boundary_length_pack.per_boundary_length_[t_idx_point23_u8];
        bc::float32_t t_s_exp_scale_f = t_s_per_end_f - t_s_point32_f;
        t_s_exp_scale_f = std::max(t_s_exp_scale_f, 1.0f);

        t_s_exp_scale_f = 2 * t_s_exp_scale_f * t_s_exp_scale_f;
        t_per_weight_f = exp(-(t_s_f - t_s_point32_f) * (t_s_f - t_s_point32_f) / t_s_exp_scale_f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_quality_f_ *= t_per_weight_f;
      }
      else
      {
        /// DO NOTHING
      }
      boundary_weight.point_weight_ar_[t_idx_u8].w_per_cs_.w_quality_f_ *= 1e3;
    }
    /** REF*/
    if (boundary_point_pack.ref_cs_.valid_b_)
    {
      bc::float32_t t_ref_weight_f = 1.f;
      if (t_idx_u8 < boundary_point_pack.ref_cs_.dy_start_idx_u8_)
      {
        t_ref_weight_f =
            exp(-(boundary_length_pack.ref_boundary_length_[t_idx_u8] -
                  boundary_length_pack.ref_boundary_length_[boundary_point_pack.ref_cs_.dy_start_idx_u8_]) *
                (boundary_length_pack.ref_boundary_length_[t_idx_u8] -
                 boundary_length_pack.ref_boundary_length_[boundary_point_pack.ref_cs_.dy_start_idx_u8_]) /
                200.f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_quality_f_ *= t_ref_weight_f;
      }
      else if (t_idx_u8 > boundary_point_pack.ref_cs_.dy_end_idx_u8_)
      {
        t_ref_weight_f = exp(-(boundary_length_pack.ref_boundary_length_[t_idx_u8] -
                               boundary_length_pack.ref_boundary_length_[boundary_point_pack.ref_cs_.dy_end_idx_u8_]) *
                             (boundary_length_pack.ref_boundary_length_[t_idx_u8] -
                              boundary_length_pack.ref_boundary_length_[boundary_point_pack.ref_cs_.dy_end_idx_u8_]) /
                             200.f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_quality_f_ *= t_ref_weight_f;
      }
      else
      {
        boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_quality_f_ *= 1.f;
      }
      boundary_weight.point_weight_ar_[t_idx_u8].w_ref_cs_.w_quality_f_ *= 1e3;
    }
    /** MAP*/
    if (boundary_point_pack.map_cs_.valid_b_)
    {
      boundary_weight.point_weight_ar_[t_idx_u8].w_map_cs_.w_quality_f_ *= 1e3;
    }
    /** LDVEH*/
    if (boundary_point_pack.ldveh_cs_.valid_b_)
    {
      bc::float32_t t_ldveh_weight_f = 1.f;
      if (t_idx_u8 < boundary_point_pack.ldveh_cs_.dy_start_idx_u8_)
      {
        // t_ldveh_weight_f = 0;
        t_ldveh_weight_f =
            exp(-(boundary_length_pack.ldveh_boundary_length_[t_idx_u8] -
                  boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_]) *
                (boundary_length_pack.ldveh_boundary_length_[t_idx_u8] -
                 boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_]) /
                10.f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_ldveh_weight_f;
      }
      else if (t_idx_u8 > boundary_point_pack.ldveh_cs_.dy_end_idx_u8_)
      {
        // t_ldveh_weight_f = 0;
        t_ldveh_weight_f =
            exp(-(boundary_length_pack.ldveh_boundary_length_[t_idx_u8] -
                  boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_end_idx_u8_]) *
                (boundary_length_pack.ldveh_boundary_length_[t_idx_u8] -
                 boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_end_idx_u8_]) /
                10.f);
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_ldveh_weight_f;
      }
      else
      {
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= 1.f;
      }
      boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_w_ldveh_decay_by_per_len_f;
      boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= 1e3;
    }
  }
  if (boundary_point_pack.ldveh_cs_.valid_b_)
  {
    if (boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_] <= 10)
    {
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= 1.8f;
      }
    }
    else if (boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_] <= 40)
    {
      bc::float32_t t_lv_qua_w_f = LinearInterpolation(
          10, 1.8, 40, 0.000001,
          boundary_length_pack.ldveh_boundary_length_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_]);
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_lv_qua_w_f;
      }
    }
    else
    {
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; t_idx_u8++)
      {
        boundary_weight.point_weight_ar_[t_idx_u8].w_ldveh_cs_.w_quality_f_ *= 0.000001f;
      }
    }
  }
  boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_per_cs_.w_quality_f_;
  boundary_weight.c0_weight_.w_map_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_map_cs_.w_quality_f_;
  boundary_weight.c0_weight_.w_ldveh_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_ldveh_cs_.w_quality_f_;
  boundary_weight.c0_weight_.w_ref_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_ref_cs_.w_quality_f_;
  boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_per_cs_.w_quality_f_;
  boundary_weight.c1_weight_.w_map_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_map_cs_.w_quality_f_;
  boundary_weight.c1_weight_.w_ldveh_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_ldveh_cs_.w_quality_f_;
  boundary_weight.c1_weight_.w_ref_cs_.w_quality_f_ =
      boundary_weight.point_weight_ar_[kFrontStartIdx].w_ref_cs_.w_quality_f_;
}

void WeightDistribution::CalcPerLenWeight(const BoundaryPoint& boundary_point,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                          BoundaryWeight& boundary_weight)
{
  if (boundary_point.valid_b_)
  {
    bc::uint8_t t_start_idx_u8 = boundary_point.dy_start_idx_u8_;
    bc::uint8_t t_end_idx_u8 = boundary_point.dy_end_idx_u8_;
    bc::float32_t t_per_pt_wei_f = 1;
    for (bc::uint8_t t_pt_idx_u8 = t_start_idx_u8; t_pt_idx_u8 <= t_end_idx_u8; t_pt_idx_u8++)
    {
      t_per_pt_wei_f = LinearInterpolation(boundary_length[t_start_idx_u8], 1, boundary_length[t_end_idx_u8], 0.6,
                                           boundary_length[t_pt_idx_u8]);
      boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_per_pt_wei_f;
    }
    if (kMaxNumBoundary > t_end_idx_u8)
    {
      for (bc::uint8_t t_pt_idx_u8 = t_end_idx_u8; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        t_per_pt_wei_f =
            ExponentialInterpolation(boundary_length[t_end_idx_u8], 0.7, boundary_length[kMaxNumBoundary - 1], 0.0001,
                                     boundary_length[t_pt_idx_u8]);
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_per_pt_wei_f;
      }
    }
    if (t_start_idx_u8 > 0)
    {
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < t_start_idx_u8; t_pt_idx_u8++)
      {
        t_per_pt_wei_f = ExponentialInterpolation(boundary_length[0], 0.2, boundary_length[t_start_idx_u8], 1,
                                                  boundary_length[t_pt_idx_u8]);
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_per_pt_wei_f;
      }
    }
  }
}

void WeightDistribution::CalcRefLenWeight(const BoundaryPoint& boundary_point,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                          BoundaryWeight& boundary_weight)
{
  if (boundary_point.valid_b_)
  {
    bc::uint8_t t_start_idx_u8 = boundary_point.dy_start_idx_u8_;
    bc::uint8_t t_end_idx_u8 = boundary_point.dy_end_idx_u8_;
    bc::float32_t t_ref_pt_wei_f = 1;
    for (bc::uint8_t t_pt_idx_u8 = t_start_idx_u8; t_pt_idx_u8 <= t_end_idx_u8; t_pt_idx_u8++)
    {
      t_ref_pt_wei_f = LinearInterpolation(boundary_length[t_start_idx_u8], 1, boundary_length[t_end_idx_u8], 0.6,
                                           boundary_length[t_pt_idx_u8]);
      boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ *= t_ref_pt_wei_f;
    }
    if (kMaxNumBoundary > t_end_idx_u8)
    {
      for (bc::uint8_t t_pt_idx_u8 = t_end_idx_u8; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        t_ref_pt_wei_f =
            ExponentialInterpolation(boundary_length[t_end_idx_u8], 0.7, boundary_length[kMaxNumBoundary - 1], 0.0001,
                                     boundary_length[t_pt_idx_u8]);
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ *= t_ref_pt_wei_f;
      }
    }
    if (t_start_idx_u8 > 0)
    {
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < t_start_idx_u8; t_pt_idx_u8++)
      {
        t_ref_pt_wei_f = ExponentialInterpolation(boundary_length[0], 0.2, boundary_length[t_start_idx_u8], 1,
                                                  boundary_length[t_pt_idx_u8]);
        boundary_weight.point_weight_ar_[t_pt_idx_u8].w_ref_cs_.w_quality_f_ *= t_ref_pt_wei_f;
      }
    }
  }
}

// void WeightDistribution::QualityWeightLength(const BoundaryPack& boundary_pack, const EmParam& param_st,
//                                              WeightPack& weight_pack)
// {
//   /** Each valid boundary has its dx_start and dx_end. Boundary points in this area will get larger weight. Weights
//   of
//    * those points outside the area will be decayed exponentially(Use Gaussian kernal). */
//   for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
//   {
//     if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].valid_b_)
//     {
//       for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
//       {
//         if (boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].valid_b_)
//         {
//           BoundaryWeight& left_boundary_weight =
//               weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_;
//           BoundaryWeight& right_boundary_weight =
//               weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_;
//           BoundaryPointPack left_boundary_point_pack =
//               boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].left_pack_cs_;
//           BoundaryPointPack right_boundary_point_pack =
//               boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].right_pack_cs_;
//           BoundaryLengthPack left_boundary_length_pack = length_pack.lane_length_pack_[t_lane_idx_u8]
//                                                              .element_length_pack_[t_ele_idx_u8]
//                                                              .left_boundary_length_pack_;
//           BoundaryLengthPack right_boundary_length_pack = length_pack.lane_length_pack_[t_lane_idx_u8]
//                                                               .element_length_pack_[t_ele_idx_u8]
//                                                               .right_boundary_length_pack_;
//           /**per*/
//           QualityElementLength(left_boundary_point_pack.per_cs_, left_boundary_length_pack.per_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_per_lowest_f,
//                                param_st.wei_param.p_w_qua_len_per_lower_f,
//                                param_st.wei_param.p_w_qua_len_per_middle_f,
//                                param_st.wei_param.p_w_qua_len_per_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourcePer, left_boundary_weight);
//           QualityElementLength(right_boundary_point_pack.per_cs_, right_boundary_length_pack.per_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_per_lowest_f,
//                                param_st.wei_param.p_w_qua_len_per_lower_f,
//                                param_st.wei_param.p_w_qua_len_per_middle_f,
//                                param_st.wei_param.p_w_qua_len_per_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourcePer, right_boundary_weight);
//           /**map*/
//           QualityElementLength(left_boundary_point_pack.map_cs_, left_boundary_length_pack.map_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_per_lowest_f,
//                                param_st.wei_param.p_w_qua_len_map_lower_f,
//                                param_st.wei_param.p_w_qua_len_map_middle_f,
//                                param_st.wei_param.p_w_qua_len_map_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourceMap, left_boundary_weight);
//           QualityElementLength(right_boundary_point_pack.map_cs_, right_boundary_length_pack.map_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_per_lowest_f,
//                                param_st.wei_param.p_w_qua_len_map_lower_f,
//                                param_st.wei_param.p_w_qua_len_map_middle_f,
//                                param_st.wei_param.p_w_qua_len_map_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourceMap, right_boundary_weight);
//           /**ldveh*/
//           QualityLdvehElementLength(left_boundary_point_pack.ldveh_cs_,
//                                     left_boundary_length_pack.ldveh_boundary_length_,
//                                     param_st.wei_param.p_w_qua_len_ldveh_upper_f,
//                                     param_st.wei_param.p_w_qua_len_invalid_f, left_boundary_weight);
//           QualityLdvehElementLength(right_boundary_point_pack.ldveh_cs_,
//                                     right_boundary_length_pack.ldveh_boundary_length_,
//                                     param_st.wei_param.p_w_qua_len_ldveh_upper_f,
//                                     param_st.wei_param.p_w_qua_len_invalid_f, right_boundary_weight);
//           /**ref*/
//           QualityElementLength(left_boundary_point_pack.ref_cs_, left_boundary_length_pack.ref_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_ref_lower_f,
//                                param_st.wei_param.p_w_qua_len_ref_lower_f,
//                                param_st.wei_param.p_w_qua_len_ref_middle_f,
//                                param_st.wei_param.p_w_qua_len_ref_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourceRef, left_boundary_weight);
//           QualityElementLength(right_boundary_point_pack.ref_cs_, right_boundary_length_pack.ref_boundary_length_,
//                                param_st.wei_param.p_w_qua_len_ref_lower_f,
//                                param_st.wei_param.p_w_qua_len_ref_lower_f,
//                                param_st.wei_param.p_w_qua_len_ref_middle_f,
//                                param_st.wei_param.p_w_qua_len_ref_upper_f,
//                                param_st.wei_param.p_w_qua_len_invalid_f, kSourceRef, right_boundary_weight);
//         }
//       }
//     }
//   }
// }

void WeightDistribution::SetCurvParam(const bc::uint8_t start_idx_u8, const bc::uint8_t end_idx_u8,
                                      const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                      const bc::float32_t w_lowest_f, const bc::float32_t w_lower_f,
                                      const bc::float32_t w_middle_f, const bc::float32_t w_upper_f,
                                      CurvParam<bc::float32_t, 5>& curve_length_cur)
{
  curve_length_cur.x_arr = {boundary_length[start_idx_u8] - 2, boundary_length[start_idx_u8],
                            boundary_length[end_idx_u8], boundary_length[end_idx_u8] + 10,
                            boundary_length[end_idx_u8] + 20};
  curve_length_cur.y_arr = {w_lower_f, w_upper_f, w_middle_f, w_lower_f, w_lowest_f};
}

void WeightDistribution::QualityElementLength(const BoundaryPoint& boundary_point,
                                              const bc::TCArray<bc::float32_t, kMaxNumBoundary> boundary_length,
                                              const bc::float32_t w_lowest_f, const bc::float32_t w_lower_f,
                                              const bc::float32_t w_middle_f, const bc::float32_t w_upper_f,
                                              const bc::float32_t w_invalid_f, bc::uint8_t source_type,
                                              BoundaryWeight& boundary_weight)
{
  if (boundary_point.valid_b_)
  {
    bc::uint8_t t_start_idx_u8 = boundary_point.dy_start_idx_u8_;
    bc::uint8_t t_end_idx_u8 = boundary_point.dy_end_idx_u8_;
    // if (t_end_idx_u8 > t_start_idx_u8)
    // {
    //   bc::TCArray<Point2D, 100> pts;
    //   pts[0] = Point2D(boundary_length[t_start_idx_u8] - 2, w_lower_f);
    //   pts[1] = Point2D(boundary_length[t_start_idx_u8], w_upper_f);
    //   pts[2] = Point2D(boundary_length[t_end_idx_u8], w_middle_f);
    //   pts[3] = Point2D(boundary_length[t_end_idx_u8] + 20, w_lowest_f);
    //   PolynomialRegression poly_fit;
    //   Eigen::Vector4d t_local_coeff{0.f, 0.f, 0.f, 0.f};
    //   poly_fit.SetPara(pts, 4, 3);
    //   t_local_coeff = poly_fit.PolyGen();
    //   ClothoidModel t_clothoid;
    //   t_clothoid.c0_position = t_local_coeff[0];
    //   t_clothoid.c1_heading_angle = t_local_coeff[1];
    //   t_clothoid.c2_curvature = t_local_coeff[2];
    //   t_clothoid.c3_curvature_derivative = t_local_coeff[3];

    //   if (source_type == kSourcePer)
    //   {
    //     QualityPerLength(t_clothoid, boundary_length, boundary_weight);
    //   }
    //   else if (source_type == kSourceMap)
    //   {
    //     QualityMapLength(t_clothoid, boundary_length, boundary_weight);
    //   }
    //   else
    //   {
    //     QualityRefLength(t_clothoid, boundary_length, boundary_weight);
    //   }
    CurvParam<bc::float32_t, 5> t_curve_length_cur;
    SetCurvParam(t_start_idx_u8, t_end_idx_u8, boundary_length, w_lowest_f, w_lower_f, w_middle_f, w_upper_f,
                 t_curve_length_cur);
    if (source_type == kSourcePer)
    {
      QualityPerInterpCurv(t_curve_length_cur, boundary_length, boundary_weight);
    }
    else if (source_type == kSourceMap)
    {
      QualityMapInterpCurv(t_curve_length_cur, boundary_length, boundary_weight);
    }
    else
    {
      QualityRefInterpCurv(t_curve_length_cur, boundary_length, boundary_weight);
    }
  }
  else
  {
    for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
    {
      if (source_type == kSourcePer)
      {
        boundary_weight.point_weight_ar_[point_idx].w_per_cs_.w_quality_f_ *= w_invalid_f;
      }
      else if (source_type == kSourceMap)
      {
        boundary_weight.point_weight_ar_[point_idx].w_map_cs_.w_quality_f_ *= w_invalid_f;
      }
      else
      {
        boundary_weight.point_weight_ar_[point_idx].w_ref_cs_.w_quality_f_ *= w_invalid_f;
      }
    }
  }
}

void WeightDistribution::QualityPerLength(const ClothoidModel& clothoid,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                          BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = clothoid.GetDyFromCloModel(boundary_length[point_idx]);
    t_w_cur_point_quality_f = t_w_cur_point_quality_f > 0 ? t_w_cur_point_quality_f : 0.001;
    boundary_weight.point_weight_ar_[point_idx].w_per_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::QualityMapLength(const ClothoidModel& clothoid,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                          BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = clothoid.GetDyFromCloModel(boundary_length[point_idx]);
    t_w_cur_point_quality_f = t_w_cur_point_quality_f > 0 ? t_w_cur_point_quality_f : 0.001;
    boundary_weight.point_weight_ar_[point_idx].w_map_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::QualityRefLength(const ClothoidModel& clothoid,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                          BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = clothoid.GetDyFromCloModel(boundary_length[point_idx]);
    t_w_cur_point_quality_f = t_w_cur_point_quality_f > 0 ? t_w_cur_point_quality_f : 0.001;
    boundary_weight.point_weight_ar_[point_idx].w_ref_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::SetLdvehCurvParam(const bc::uint8_t start_idx_u8,
                                           const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                           const bc::float32_t w_upper_f, const bc::float32_t w_invalid_f,
                                           CurvParam<bc::float32_t, 3>& curve_length_cur)
{
  curve_length_cur.x_arr = {boundary_length[start_idx_u8] - 10, boundary_length[start_idx_u8],
                            boundary_length[kMaxNumBoundary - 1]};
  curve_length_cur.y_arr = {w_invalid_f, w_invalid_f, w_upper_f};
}
void WeightDistribution::QualityLdvehElementLength(const BoundaryPoint& boundary_point,
                                                   const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                                   const bc::float32_t w_upper_f, const bc::float32_t w_invalid_f,
                                                   BoundaryWeight& boundary_weight)
{
  if (boundary_point.valid_b_)
  {
    bc::uint8_t t_start_idx_u8 = boundary_point.dy_start_idx_u8_;
    bc::uint8_t t_end_idx_u8 = boundary_point.dy_end_idx_u8_;
    if (t_end_idx_u8 > t_start_idx_u8)
    {
      CurvParam<bc::float32_t, 3> t_curve_length_cur;
      SetLdvehCurvParam(t_start_idx_u8, boundary_length, w_upper_f, w_invalid_f, t_curve_length_cur);
      QualityLdvehInterpCurv(t_curve_length_cur, boundary_length, boundary_weight);
    }
  }
  else
  {
    for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
    {
      boundary_weight.point_weight_ar_[point_idx].w_ldveh_cs_.w_quality_f_ *= w_invalid_f;
    }
  }
}
void WeightDistribution::QualityPerInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                              BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = InterpCurv(t_curve_length_cur, boundary_length[point_idx]);
    boundary_weight.point_weight_ar_[point_idx].w_per_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::QualityMapInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                              BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = InterpCurv(t_curve_length_cur, boundary_length[point_idx]);
    boundary_weight.point_weight_ar_[point_idx].w_map_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::QualityLdvehInterpCurv(const CurvParam<bc::float32_t, 3> t_curve_length_cur,
                                                const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                                BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = InterpCurv(t_curve_length_cur, boundary_length[point_idx]);
    boundary_weight.point_weight_ar_[point_idx].w_ldveh_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
void WeightDistribution::QualityRefInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                              BoundaryWeight& boundary_weight)
{
  for (bc::uint16_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
  {
    bc::float32_t t_w_cur_point_quality_f = InterpCurv(t_curve_length_cur, boundary_length[point_idx]);
    boundary_weight.point_weight_ar_[point_idx].w_ref_cs_.w_quality_f_ *= t_w_cur_point_quality_f;
  }
}
/**QualityWeightExistProb*/
void WeightDistribution::QualityWeightExistProb(const BoundaryPack& boundary_pack, WeightPack& weight_pack)
{
  /** 1. According to latest element definition, pereption lane is assigned to element[0], so this weight is available
   * for element[0]. As to a perception boundary, no matter it's real or virtual, this weight can be calculated by
   * prob_exist from per_quality_cs_, and no less than a threshold, for example, 0.5. Virtual perception boundary will
   * be punished in other function. */

  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
    {
      bc::float32_t left_boundary_ref_exist_prob = boundary_pack.lane_pack_ar_[t_lane_idx_u8]
                                                       .element_pack_ar_[t_ele_idx_u8]
                                                       .left_pack_cs_.per_quality_cs_.prob_exist_f_;
      bc::float32_t right_boundary_ref_exist_prob = boundary_pack.lane_pack_ar_[t_lane_idx_u8]
                                                        .element_pack_ar_[t_ele_idx_u8]
                                                        .right_pack_cs_.per_quality_cs_.prob_exist_f_;
      for (bc::uint8_t point_idx = 0; point_idx < kMaxNumBoundary; point_idx++)
      {
        WeightDist& left_point_per_weight_w = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                  .element_weight_ar_[t_ele_idx_u8]
                                                  .left_weight_ar_.point_weight_ar_[point_idx]
                                                  .w_per_cs_;
        WeightDist& right_point_per_weight_w = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                   .element_weight_ar_[t_ele_idx_u8]
                                                   .right_weight_ar_.point_weight_ar_[point_idx]
                                                   .w_per_cs_;
        left_point_per_weight_w.w_quality_f_ *= LinearInterpolation(0.f, 0.5f, 1.f, 1.f, left_boundary_ref_exist_prob);
        right_point_per_weight_w.w_quality_f_ *=
            LinearInterpolation(0.f, 0.5f, 1.f, 1.f, right_boundary_ref_exist_prob);
      }
      weight_pack.lane_weight_ar_[t_lane_idx_u8]
          .element_weight_ar_[t_ele_idx_u8]
          .left_weight_ar_.c0_weight_.w_per_cs_.w_quality_f_ = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                                   .element_weight_ar_[t_ele_idx_u8]
                                                                   .left_weight_ar_.point_weight_ar_[kFrontStartIdx]
                                                                   .w_per_cs_.w_quality_f_;
      weight_pack.lane_weight_ar_[t_lane_idx_u8]
          .element_weight_ar_[t_ele_idx_u8]
          .left_weight_ar_.c1_weight_.w_per_cs_.w_quality_f_ = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                                   .element_weight_ar_[t_ele_idx_u8]
                                                                   .left_weight_ar_.point_weight_ar_[kFrontStartIdx]
                                                                   .w_per_cs_.w_quality_f_;
    }
  }
}
/**QualityWeightVariance*/
void WeightDistribution::QualityWeightVariance(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                               const BoundaryPack& boundary_pack, WeightPack& weight_pack)
{
  /** 1. As to perception boundary, variance is represented by the standard deviation. So we can create an upper and a
   * lower bound for the detected boundary by applying StdDev to its coefficient.
   * Upper Bound: c_upper = c_coeff +  c_std
   * Lower Bound: c_upper = c_coeff -  c_std
   */

  /** 2. For each point, check its x coordinate and calculate corresponding dy on Upper and Lower Bound */

  /** 3. Calc distance between dy_upper/lower and dy by a Gaussian kernal:
   * d_upper/lower = exp(-(dy_upper/lower - dy)^2)/(2*sig2). sig2 can be set as 1. */

  /** 4. calc min(d_upper, d_lower) as weight of variance. */
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
    {
      QualityBoundaryVariance(
          x_ref, boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].left_pack_cs_,
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_);
      QualityBoundaryVariance(
          x_ref, boundary_pack.lane_pack_ar_[t_lane_idx_u8].element_pack_ar_[t_ele_idx_u8].right_pack_cs_,
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_);
    }
  }
}
void WeightDistribution::QualityBoundaryVariance(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                                                 const BoundaryPointPack& boundary_point_pack,
                                                 BoundaryWeight& boundary_weight)
{
  if (boundary_point_pack.valid_b_)
  {
    for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
    {
      BoundaryPoint cur_boundary_point = boundary_point_pack.per_cs_;
      bc::float32_t t_cur_dx_f = x_ref[t_pt_idx_u8];
      bc::float32_t t_cur_dy_f = cur_boundary_point.dy_ar_[t_pt_idx_u8];
      BoundaryQualityPer coeff_std = boundary_point_pack.per_quality_cs_;
      bc::float32_t t_delta_variance_dy_f = coeff_std.std_c0_f_ + coeff_std.std_c1_f_ * t_cur_dx_f +
                                            coeff_std.std_c2_f_ * t_cur_dx_f * t_cur_dx_f +
                                            coeff_std.std_c3_f_ * t_cur_dx_f * t_cur_dx_f * t_cur_dx_f;
      bc::float32_t t_gaussian_distance_f =
          exp(-(t_cur_dy_f - t_delta_variance_dy_f) * (t_cur_dy_f - t_delta_variance_dy_f)) /
          (2 * kGuassianSigma * kGuassianSigma);
      boundary_weight.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_gaussian_distance_f;
    }
    boundary_weight.c0_weight_.w_per_cs_.w_quality_f_ =
        boundary_weight.point_weight_ar_[kFrontStartIdx].w_per_cs_.w_quality_f_;
    boundary_weight.c1_weight_.w_per_cs_.w_quality_f_ =
        boundary_weight.point_weight_ar_[kFrontStartIdx].w_per_cs_.w_quality_f_;
  }
}

/**CalcVehicleWeight*/
void WeightDistribution::QualityWeightVeh(EgoPoseCollection& ego_motion_cs, WeightPack& weight_pack)
{
  CurvParam<bc::float32_t, 3> t_curve_veh_cur;
  t_curve_veh_cur.x_arr = {0, 5 / 3.6, 15 / 3.6};
  t_curve_veh_cur.y_arr = {0, 0.5, 1};
  bc::float32_t t_ego_veh_f = ego_motion_cs.GetCurrentEgoPose().GetLongVelocity();
  bc::float32_t t_w_cur_veh_f = InterpCurv(t_curve_veh_cur, t_ego_veh_f);
  bc::float32_t t_w_squ_veh_f = t_w_cur_veh_f * t_w_cur_veh_f;
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
    {
      BoundaryWeight& left =
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_;
      BoundaryWeight& right =
          weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_;
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
      {
        left.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
        left.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
        right.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
        right.point_weight_ar_[t_pt_idx_u8].w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
      }

      left.c0_weight_.w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
      left.c0_weight_.w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
      left.c1_weight_.w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
      left.c1_weight_.w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
      right.c0_weight_.w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
      right.c0_weight_.w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
      right.c1_weight_.w_per_cs_.w_quality_f_ *= t_w_squ_veh_f;
      right.c1_weight_.w_ldveh_cs_.w_quality_f_ *= t_w_squ_veh_f;
    }
  }
}

/**CalcLifetimeWeight*/
void WeightDistribution::QualityWeightLifetime(const BoundaryPack& boundary_pack, WeightPack& weight_pack)
{
  CurvParam<bc::float32_t, 3> t_curve_lifetime_cur;
  t_curve_lifetime_cur.x_arr = {0, 0.5, 1};
  t_curve_lifetime_cur.y_arr = {0, 0.5, 1};
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (weight_pack.lane_weight_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].valid_b_)
        {
          BoundaryWeight& left =
              weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_;
          bc::float32_t t_left_per_lifetime_f = boundary_pack.lane_pack_ar_[t_lane_idx_u8]
                                                    .element_pack_ar_[t_ele_idx_u8]
                                                    .left_pack_cs_.per_cs_.per_life_time_;
          bc::float32_t t_w_left_cur_lifetime_f = InterpCurv(t_curve_lifetime_cur, t_left_per_lifetime_f);
          bc::float32_t t_w_left_squ_lifetime_f = t_w_left_cur_lifetime_f * t_w_left_cur_lifetime_f;
          for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
          {
            left.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_w_left_squ_lifetime_f;
          }
          left.c0_weight_.w_per_cs_.w_quality_f_ *= t_w_left_squ_lifetime_f;
          left.c1_weight_.w_per_cs_.w_quality_f_ *= t_w_left_squ_lifetime_f;
          BoundaryWeight& right =
              weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_;
          bc::float32_t t_right_per_lifetime_f = boundary_pack.lane_pack_ar_[t_lane_idx_u8]
                                                     .element_pack_ar_[t_ele_idx_u8]
                                                     .right_pack_cs_.per_cs_.per_life_time_;
          bc::float32_t t_w_right_cur_lifetime_f = InterpCurv(t_curve_lifetime_cur, t_right_per_lifetime_f);
          bc::float32_t t_w_right_squ_lifetime_f = t_w_right_cur_lifetime_f * t_w_right_cur_lifetime_f;
          for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
          {
            right.point_weight_ar_[t_pt_idx_u8].w_per_cs_.w_quality_f_ *= t_w_right_squ_lifetime_f;
          }

          right.c0_weight_.w_per_cs_.w_quality_f_ *= t_w_right_squ_lifetime_f;
          right.c1_weight_.w_per_cs_.w_quality_f_ *= t_w_right_squ_lifetime_f;
        }
      }
    }
  }
}

/**Normalize*/
void WeightDistribution::Normalize(WeightPack& weight_pack)
{
  /** For each lane/elment/point */

  /** 1. For each valid source, calculate w_total = w_raw * w_scenario * w_quality. */

  /** 2. Normalize w_total over all valid sources, and calculate w_norm for each source. */

  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (weight_pack.lane_weight_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].valid_b_)
        {
          if (weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_.valid_b_)
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
            {
              PointWeight& left_point_weight_w = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                     .element_weight_ar_[t_ele_idx_u8]
                                                     .left_weight_ar_.point_weight_ar_[t_pt_idx_u8];
              BoundaryNormalize(left_point_weight_w);
            }
            BoundaryNormalize(
                weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_.c0_weight_);
            BoundaryNormalize(
                weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].left_weight_ar_.c1_weight_);
          }
          if (weight_pack.lane_weight_ar_[t_lane_idx_u8].element_weight_ar_[t_ele_idx_u8].right_weight_ar_.valid_b_)
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxNumBoundary; t_pt_idx_u8++)
            {
              PointWeight& right_point_weight_w = weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                                      .element_weight_ar_[t_ele_idx_u8]
                                                      .right_weight_ar_.point_weight_ar_[t_pt_idx_u8];
              BoundaryNormalize(right_point_weight_w);
            }
            BoundaryNormalize(weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                  .element_weight_ar_[t_ele_idx_u8]
                                  .right_weight_ar_.c0_weight_);
            BoundaryNormalize(weight_pack.lane_weight_ar_[t_lane_idx_u8]
                                  .element_weight_ar_[t_ele_idx_u8]
                                  .right_weight_ar_.c1_weight_);
          }
        }
      }
    }
  }
}
void WeightDistribution::BoundaryNormalize(PointWeight& point_weight)
{
  point_weight.w_per_cs_.w_total_f_ =
      point_weight.w_per_cs_.w_raw_f_ * point_weight.w_per_cs_.w_scenario_f_ * point_weight.w_per_cs_.w_quality_f_;
  point_weight.w_map_cs_.w_total_f_ =
      point_weight.w_map_cs_.w_raw_f_ * point_weight.w_map_cs_.w_scenario_f_ * point_weight.w_map_cs_.w_quality_f_;
  point_weight.w_ldveh_cs_.w_total_f_ = point_weight.w_ldveh_cs_.w_raw_f_ * point_weight.w_ldveh_cs_.w_scenario_f_ *
                                        point_weight.w_ldveh_cs_.w_quality_f_;
  point_weight.w_ref_cs_.w_total_f_ =
      point_weight.w_ref_cs_.w_raw_f_ * point_weight.w_ref_cs_.w_scenario_f_ * point_weight.w_ref_cs_.w_quality_f_;
  bc::float32_t t_sum_of_all_source_f = point_weight.w_per_cs_.w_total_f_ + point_weight.w_map_cs_.w_total_f_ +
                                        point_weight.w_ldveh_cs_.w_total_f_ + point_weight.w_ref_cs_.w_total_f_;
  if (t_sum_of_all_source_f < 1e-8)
  {
    point_weight.w_per_cs_.w_norm_f_ = 0.f;
    point_weight.w_map_cs_.w_norm_f_ = 0.f;
    point_weight.w_ldveh_cs_.w_norm_f_ = 0.f;
    point_weight.w_ref_cs_.w_norm_f_ = 0.f;
  }
  else
  {
    point_weight.w_per_cs_.w_norm_f_ = point_weight.w_per_cs_.w_total_f_ / t_sum_of_all_source_f;
    point_weight.w_map_cs_.w_norm_f_ = point_weight.w_map_cs_.w_total_f_ / t_sum_of_all_source_f;
    point_weight.w_ldveh_cs_.w_norm_f_ = point_weight.w_ldveh_cs_.w_total_f_ / t_sum_of_all_source_f;
    point_weight.w_ref_cs_.w_norm_f_ = point_weight.w_ref_cs_.w_total_f_ / t_sum_of_all_source_f;
  }
}
bc::float32_t WeightDistribution::CalcSquRoot(const bc::float32_t x1, const bc::float32_t x2, const bc::float32_t y1,
                                              const bc::float32_t y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
}  // namespace environment_model
}  // namespace zone
