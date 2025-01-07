#ifndef POINT_BASED_HLMF_H_INCLUDE_
#define POINT_BASED_HLMF_H_INCLUDE_

#include "Eigen/Dense"
#include "common/data/em_data.h"
#include "common/data/em_dbg_data.h"
#include "common/data/navigation_data.h"
#include "common/data/navigation_plus_data.h"
#include "environment_model_internal.h"
#include "math/bitwise_operation.h"
#include "math/linear_interpolation.h"
#include "math/low_pass_filter.h"
#include "math/multi_dimention_interpolation.h"
#include "math/polynomial_regression.h"
namespace zone {
namespace environment_model {

using ::zone::common::EmPreDbgData;
using ::zone::common::Point3D;
using ::zone::common::StaticObject;
using ::zone::data::em_data::AgentCurrentPosProjOnToRefLine;
using ::zone::data::em_data::EmData;
using ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum;
using ::zone::data::em_data::kHDMapSpdLmtSegIdx;
using ::zone::data::em_data::kHostLane;
using ::zone::data::em_data::kInvalidEgoPointIdx;
using ::zone::data::em_data::kLeftLane;
using ::zone::data::em_data::kLLLane;
using ::zone::data::em_data::kMaxBoundaryPoint;
using ::zone::data::em_data::kMaxElemNumInOneLane;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kMaxLanePropertySegsInOneLaneElement;
using ::zone::data::em_data::kMaxRefLinePtsNum;
using ::zone::data::em_data::kRightLane;
using ::zone::data::em_data::kRRLane;
using ::zone::data::em_data::kSDMapEleEyeSpdLmtSegIdx;
using ::zone::data::em_data::kSDMapNormalSpdLmtSegIdx;
using ::zone::data::em_data::kSDMapSectionSpdLmtSegIdx;
using ::zone::data::em_data::kPerNormalSpdLmtSegIdx;
using ::zone::data::em_data::kPerSpdLmtSegIdx;
using ::zone::data::em_data::kPerSpdLmtRevSegIdx;
using ::zone::data::em_data::LaneBoundary;
using ::zone::data::em_data::LaneData;
using ::zone::data::em_data::LaneElement;
using ::zone::data::em_data::ReferenceLine;
using ::zone::data::em_data::RefLineCoeff;
using ::zone::data::em_data::RefLineSegSpeedLimit;
using ::zone::data::em_data::RefLineSegSpeedLimitSource;
using ::zone::helper::math::Lin_Interp_Method;
using ::zone::helper::math::PolynomialRegression;
using ::zone::helper::math::SetBitU8;
using ::zone::helper::math::GetBitU8;
using ::zone::common::CurvParam;
using ::zone::helper::math::InterpCurv;
using ::zone::helper::math::LowPass;
using ::zone::common::EmParam;
static const bc::float32_t kMinRMSE = 0.5f;

class Hlmf
{
 public:
  Hlmf(EmData& em_collection, EmData& em_collection_lst_cycle);

  ~Hlmf() = default;
  void SetInput(const BoundaryPoint& ldveh_cen_st_pack) { ldveh_cen_st_pack_ = ldveh_cen_st_pack; }
  void Run(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_, const BoundaryPack& boundary_pack,
           const WeightPack& weight_pack, const SemanticPack& semantic_pack_cs, StEndIdxPack& last_em_st_end_idx_,
           EgoPoseCollection& ego_motion_cs, const EmData& em_output_lst_cycle_cs_, const EmParam& param_st_,
           const bc::float32_t time_cycle_f);
  void GetDbgData(EmPreDbgData& dbg_data_cs);
  void Reset(EmData& em_collection, EmData& em_collection_lst_cycle)
  {
    em_collection_cs_ = &em_collection;
    em_collection_lst_cycle_ = &em_collection_lst_cycle;
    semantic_pack_cs = SemanticPack();
    ldveh_cen_st_pack_ = BoundaryPoint();
    host_ref_exist_b = bc::false_v;
    map_exist_b = bc::false_v;
    for (bc::uint8_t t_i_u8 = 0; t_i_u8 < kMaxNumBoundary; t_i_u8++)
    {
      host_left_heading_ar[t_i_u8] = 0.f;
      host_right_heading_ar[t_i_u8] = 0.f;
    }
  }

 private:
  /******************************************************************************************/
  /** function*/
  void LaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                          const BoundaryPack& boundary_pack, const WeightPack& weight_pack,
                          StEndIdxPack& last_em_st_end_idx_, EgoPoseCollection& ego_motion_cs);
  void ReflineFusion(const StEndIdxPack& last_em_st_end_idx_, const EmData& em_last_output,
                     EgoPoseCollection& ego_motion_cs, const EmParam& param_st_, const bc::float32_t time_cycle_f);
  void SetValidity(const BoundaryPack& boundary_pack);
  void HostLaneProcess(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar, EgoPoseCollection& ego_motion_cs,
                       LaneStEndIdxPack& lane_st_end_idx);
  void GenerateHostLane(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar, bc::float32_t lane_width_f,
                        bc::TCArray<bc::float32_t, 4> coeff_ar);
  void HostLaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_, const LanePack& lane_pack,
                              const LaneWeight& lane_weight, LaneData& lane_data, LaneStEndIdxPack& lane_st_end_idx);
  void NeighborLaneGeometryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                                  const LanePack& lane_pack_tar, const LanePack& lane_pack_ref,
                                  const LaneWeight& lane_weight, const bc::uint8_t dir_u8,
                                  const LaneData& ref_lane_data, const LaneStEndIdxPack& ref_lane_st_end_idx,
                                  const bc::TCArray<bc::float32_t, kMaxNumBoundary>& host_bd_heading_ar,
                                  LaneData& tar_lane_data, LaneStEndIdxPack& lane_st_end_idx);
  void HostBoundaryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                          const BoundaryPointPack& boundary_point_pack, const BoundaryWeight& boundary_weight,
                          const ElementStEndIdxPack& st_end_idx, LaneBoundary& lane_boundary);
  void HostDeltaHeadingFusion(const BoundaryPointPack& boundary_point_pack,
                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                              const BoundaryWeight& boundary_weight, const bc::uint8_t datum_idx,
                              LaneBoundary& lane_boundary);
  void NeighDeltaHeadingFusion(const BoundaryPointPack& boundary_point_pack,
                               const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                               const BoundaryWeight& boundary_weight, const bc::uint8_t datum_idx,
                               LaneBoundary& lane_boundary);
  void BoundaryFusion(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar_,
                      const BoundaryPointPack& boundary_point_pack, const BoundaryWeight& boundary_weight,
                      const ElementStEndIdxPack& st_end_idx, LaneBoundary& lane_boundary);
  bc::bool_t CheckSplit(const BoundaryPointPack& tar_boundary_pack, const BoundaryPointPack& ref_boundary_pack);
  void StoreParStartEndIdx(const ElementStEndIdxPack& ref_idx, const BoundaryPointPack& left_pack,
                           const BoundaryPointPack& right_pack, ElementStEndIdxPack& st_end_idx);
  void StoreStartEndIdx(const BoundaryPointPack& left_pack, const BoundaryPointPack& right_pack,
                        ElementStEndIdxPack& st_end_idx);
  void CheckLaneCross(bc::TCArray<LaneData, kMaxLaneNum>& lanes, StEndIdxPack& last_em_st_end_idx_);
  bc::bool_t FindEgoIdx(LaneBoundary& boundary);
  void CutInvPoint(const bc::uint8_t ele_idx_u8, LaneData& lane_data);
  void ReflineFillIn(const LaneBoundary& left, const LaneBoundary& right, const ElementStEndIdxPack& st_end_idx,
                     ReferenceLine& refline, const EmData& em_last_cycle_output, const EmParam& param_st_,
                     const bc::float32_t time_cycle_f, const bc::float32_t long_velocity_f, const bc::uint8_t lane_idx,
                     const bc::uint8_t element_idx);
  void CheckEndIdx(const bc::uint8_t left_end_idx, const bc::uint8_t right_end_idx, LaneBoundary& left_boundary,
                   LaneBoundary& right_boundary);
  bc::bool_t CenterLineGenerator(const LaneBoundary& left, const LaneBoundary& right, ReferenceLine& refer_line);
  bc::float32_t CalcSquRoot(const bc::float32_t x1, const bc::float32_t x2, const bc::float32_t y1,
                            const bc::float32_t y2);

  void CalcDyBoundary(const LaneElement& element_cs, const bc::float32_t dx_tsr_f, bc::float32_t& dy_boundary_left_f,
                      bc::float32_t& dy_boundary_right_f);

  void SemanticInfoSet(const SemanticPack& semantic_pack_cs);
  bc::float32_t CalculateMean(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& raw_kappa, bc::uint8_t start_idx,
                              bc::uint8_t end_idx);
  /******************************************************************************************/
  /** input and output classes */
  EmData* em_collection_cs_;
  EmData* em_collection_lst_cycle_;
  SemanticPack semantic_pack_cs;
  BoundaryPoint ldveh_cen_st_pack_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> host_left_heading_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> host_right_heading_ar;
  bc::bool_t host_ref_exist_b = false;
  bc::bool_t map_exist_b = false;
  bc::bool_t in_map_b = false;
  bc::uint8_t per_id_counter_ = 0;
  /******************************************************************************************/
  /** internal classes and structures */
  bc::TCArray<bc::float32_t, 100> per_ar;
  bc::TCArray<bc::float32_t, 100> ref_ar;
  bc::TCArray<bc::float32_t, 100> ldveh_ar;
  bc::TCArray<bc::float32_t, 100> map_ar;
  /******************************************************************************************/
  /** arrays and vectors */

  /******************************************************************************************/
  /** variables */
};
}  // namespace environment_model
}  // namespace zone
#endif
