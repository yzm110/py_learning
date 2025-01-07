#pragma once
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
typedef std::vector<double> vecD;

using zone::data::em_data::kFusMaxObjNum;
using zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum;
using zone::data::em_data::kMaxLaneNum;
using zone::data::em_data::kMaxElemNumInOneLane;
using zone::data::em_data::kMaxBoundarySegment;
using zone::data::em_data::kMaxLanePropertySegsInOneLaneElement;
using zone::data::em_data::kMaxLaneTransitionSegsInOneLaneElement;
using zone::data::em_data::kMaxSpecialSituationSegsInOneLaneElement;
using zone::data::em_data::kMaxRefLinePtsNum;
using zone::data::em_data::kMaxMapGuidePtNum;
using zone::data::em_data::kUnknownLane;
using zone::data::em_data::kUnknownElementNum;
using zone::data::em_data::kInvalidObjectId;
using zone::data::em_data::kMaxObjectId;  // Valid Fusion Id range [1, 1023]
using zone::data::em_data::kInvalidObjIdx;
using zone::data::em_data::kMaxBoundaryPoint;
using zone::data::em_data::kInvalidEgoPointIdx;
using zone::data::em_data::kMaxLaneAssociationNum;

// do interpolate
class EhrAdapterHelper
{
 private:
  /* data */
 public:
  EhrAdapterHelper(/* args */);
  ~EhrAdapterHelper();
  // lane_flag -2, -1 , 0(center), 1, 2(left)
  // static void BSpineInterpolation(bc::TFixedVector<bc::float64_t, 1000> x, bc::TFixedVector<bc::float64_t, 1000> y,
  // bc::TFixedVector<bc::float64_t, 1000>& rx_, bc::TFixedVector<bc::float64_t, 1000>& ry_,
  //                                 bc::TFixedVector<bc::float64_t, 1000>& rs_, bc::float64_t ds,
  //                                 bc::TFixedVector<bc::float64_t, 1000>& rh_,
  //                                 bc::TFixedVector<bc::float64_t, 1000>& rc_, bc::uint32_t& idx,
  //                                 uint32_t& current_idx,
  //                                 int8_t lane_flag = 0);

  static void BSpineInterpolation(bc::TFixedVector<bc::float64_t, 1000>& x, bc::TFixedVector<bc::float64_t, 1000>& y,
                                  bc::TFixedVector<bc::float64_t, 1000>& rx_,
                                  bc::TFixedVector<bc::float64_t, 1000>& ry_, bc::uint32_t& idx, uint32_t& current_idx);

  // static void BSpineInterpolationCenterline(
  //      bc::TFixedVector<bc::float64_t, 1000> &poins_offset, bc::TFixedVector<bc::float64_t, 1000> &rpoins_offset_,
  //      bc::TFixedVector<bc::float64_t, 1000> x, bc::TFixedVector<bc::float64_t, 1000> y,
  //      bc::TFixedVector<bc::float64_t, 1000>& rx_,
  //     bc::TFixedVector<bc::float64_t, 1000>& ry_, bc::TFixedVector<bc::float64_t, 1000>& rs_, bc::float64_t ds,
  //     bc::TFixedVector<bc::float64_t, 1000>& rh_, bc::TFixedVector<bc::float64_t, 1000>& rc_,
  //     bc::uint32_t& idx, uint32_t& current_idx, int8_t lane_flag = 0);
  static void BSpineInterpolationCenterline(bc::TFixedVector<bc::float64_t, 1000>& poins_offset,
                                            bc::TFixedVector<bc::float64_t, 1000>& rpoins_offset_,
                                            bc::TFixedVector<bc::float64_t, 1000>& x,
                                            bc::TFixedVector<bc::float64_t, 1000>& y,
                                            bc::TFixedVector<bc::float64_t, 1000>& rx_,
                                            bc::TFixedVector<bc::float64_t, 1000>& ry_, bc::uint32_t& idx,
                                            uint32_t& current_idx);
};
