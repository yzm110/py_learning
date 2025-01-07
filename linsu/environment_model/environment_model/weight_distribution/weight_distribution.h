#ifndef environment_model_WEIGHTDISTRIBUTION_H
#define environment_model_WEIGHTDISTRIBUTION_H
#include "common/data/common_data.h"
#include "common/data/em_param.h"
#include "environment_model_internal.h"
#include "math/exponential_interpolation.h"
#include "math/linear_interpolation.h"
#include "math/multi_dimention_interpolation.h"
#include "math/polynomial_regression.h"

namespace zone {
namespace environment_model {

static const bc::float32_t kLeftFactor = -1.f;
static const bc::float32_t kRightFactor = 1.f;
static const bc::float32_t kGuassianSigma = 1.f;
static const bc::float32_t kEpsilon = 0.0000001;
static const bc::float32_t kRangeDistance = 100.f;
using ::zone::common::CurvParam;
using ::zone::common::EmParam;
using ::zone::common::WeiDisParam;
using ::zone::data::em_data::kHostLane;
using ::zone::data::em_data::kLeftLane;
using ::zone::data::em_data::kLLLane;
using ::zone::data::em_data::kMaxElemNumInOneLane;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kRightLane;
using ::zone::data::em_data::kRRLane;
using ::zone::data::em_data::LaneBoundaryType;
using ::zone::data::em_data::LaneTransitionDirection;
using ::zone::data::em_data::MapGuidePointType;
using ::zone::environment_model::kMaxNumBoundary;
using ::zone::environment_model::PointWeight;
using ::zone::environment_model::WeightPack;
using ::zone::environment_model::kSourcePer;
using ::zone::environment_model::kSourceMap;
using ::zone::environment_model::kSourceLdveh;
using ::zone::environment_model::kSourceRef;
using ::zone::helper::math::ExponentialInterpolation;
using ::zone::helper::math::InterpCurv;
using ::zone::helper::math::LinearInterpolation;
using ::zone::helper::math::PolynomialRegression;

class BoundaryLengthPack
{
 public:
  bc::TCArray<bc::float32_t, kMaxNumBoundary> per_boundary_length_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> map_boundary_length_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> ldveh_boundary_length_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> ref_boundary_length_;
};

class ElementLengthPack
{
 public:
  BoundaryLengthPack left_boundary_length_pack_;
  BoundaryLengthPack right_boundary_length_pack_;
};

class LaneLengthPack
{
 public:
  bc::TCArray<ElementLengthPack, kMaxElemNumInOneLane> element_length_pack_;
};

class LengthPack
{
 public:
  bc::TCArray<LaneLengthPack, kMaxLaneNum> lane_length_pack_;
  LengthPack() : lane_length_pack_{} {}
};

class WeightDistribution
{
 public:
  WeightDistribution() {}
  ~WeightDistribution() = default;

  void Run(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref, const BoundaryPack& boundary_pack,
           const EmParam& param_st, const SemanticPack& semantic_pack, EgoPoseCollection& ego_motion_cs,
           WeightPack& weight_pack);
  WeightPack WeightOutput() { return weight_pack; }
  BoundaryPoint LdvehCenOutput() { return ldveh_cen_pt_pack_cs; }
  //  private:
  /******************************************************************************************/
  /** function */
  void Initialize(const BoundaryPack& boundary_pack, const EmParam& param_st, WeightPack& weight_pack);
  void CalcLengthPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref, const BoundaryPack& boundary_pack);
  void CalcRawWeight(const BoundaryPack& boundary_pack, const EmParam& param_st, WeightPack& weight_pack);
  void CalcScenarioWeight(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref, const BoundaryPack& boundary_pack,
                          const EmParam& param_st, const SemanticPack& semantic_pack, WeightPack& wesight_pack);
  void CalcQualityWeight(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref, const BoundaryPack& boundary_pack,
                         const EmParam& param_st, EgoPoseCollection& ego_motion_cs, WeightPack& weight_pack);
  void Normalize(WeightPack& weight_pack);

  /**Initialize*/
  void BoundaryRawWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void BoundaryScenWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void BoundaryQualityWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void BoundaryTotalWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void BoundaryNormWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void BoundaryWeightInit(const bc::float32_t w_default_f, BoundaryWeight& boundary_weight);
  void ElementWeightInit(const bc::float32_t w_default_f, ElementWeight& elem_weight);
  void LaneWeightInit(const bc::float32_t w_default_f, LaneWeight& lane_weight);
  void SourceBoundaryInit(const bc::float32_t w_default_f, WeightDist& weight_dist);
  void BoundaryInit(const BoundaryPointPack& boundary_point_pack, const bc::float32_t invalid_weight,
                    const bc::float32_t valid_weight, BoundaryWeight& boundary_weight);

  /**CalcLengthPack*/
  void CalcSrcBoundaryLenPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_dy,
                              bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length);
  void CalcBoundaryLengthPack(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                              const BoundaryPointPack& boundary_point_pack, BoundaryLengthPack& boundary_length_pack);

  /**CalcRawWeight*/
  void RawWeightEgo(const EmParam& param_st, LaneWeight& Lane_weight);
  void RawWeightNeighbor(const LanePack& lane_pack_tar, const LanePack& lane_pack_ref, const EmParam& param_st,
                         const bc::uint8_t dir_u8, const bc::uint8_t tar_lane_idx, LaneWeight& lane_weight);
  bc::bool_t CheckLaneSelfSplit(const LaneWeight& lane_weight);
  void RawBoundaryWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f, const bc::float32_t w_ldveh_f,
                            const bc::float32_t w_ref_f, BoundaryWeight& boundary_weight);
  void RawElementWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f, const bc::float32_t w_ldveh_f,
                           const bc::float32_t w_ref_f, ElementWeight& element_weight);
  void RawLaneWeightSet(const bc::float32_t w_per_f, const bc::float32_t w_map_f, const bc::float32_t w_ldveh_f,
                        const bc::float32_t w_ref_f, LaneWeight& lane_weight);
  /**CalcScenarioWeight*/
  void ScenIntersectionBoundaryWeightSet(const BoundaryLengthPack& boundary_length_pack, const bc::float32_t start_s,
                                         const bc::float32_t end_s, const BoundaryPointPack& boundary_point_pack,
                                         const WeiDisParam& wei_param, BoundaryWeight& boundary_weight);
  void ScenRampBoundaryWeightSet(const BoundaryLengthPack& boundary_length_pack, const bc::float32_t start_s,
                                 const bc::float32_t end_s, const BoundaryPointPack& boundary_point_pack,
                                 const WeiDisParam& wei_param, BoundaryWeight& boundary_weight);

  void ScenSourceWeightInterpCurv(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                  const bc::float32_t start_s, const bc::float32_t end_s, const bc::float32_t w_ramp_f,
                                  const bc::float32_t w_out_ramp_f, BoundaryWeight& boundary_weight,
                                  bc::uint8_t weight_source);
  void ScenCheckSourceSetValid(const bc::uint8_t weight_source, const bc::float32_t weight_scen_f,
                               PointWeight& point_weight);

  /**CalcQualityWeight*/
  void QualityWeightType(const BoundaryPack& boundary_pack, const EmParam& param_st, WeightPack& weight_pack);
  void QualityWeightLength(const BoundaryPack& boundary_pack, const EmParam& param_st,
                           const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref, WeightPack& weight_pack);
  void QualityWeightExistProb(const BoundaryPack& boundary_pack, WeightPack& weight_pack);
  void QualityWeightVariance(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                             const BoundaryPack& boundary_pack, WeightPack& weight_pack);
  void QualityWeightVeh(EgoPoseCollection& ego_motion_cs, WeightPack& weight_pack);
  /**QualityWeightType*/
  void QualityPerBoundaryType(const BoundaryPointPack& boundary_point_pack, const EmParam& param_st,
                              BoundaryWeight& boundary_weight);
  void QualityMapBoundaryType(const BoundaryPointPack& boundary_point_pack, const EmParam& param_st,
                              BoundaryWeight& boundary_weight);
  /**QualityWeightLength*/
  void CalcLengthWeight(const BoundaryPointPack& boundary_point_pack,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                        const BoundaryLengthPack& boundary_length_pack, BoundaryWeight& boundary_weight);
  void CalcPerLenWeight(const BoundaryPoint& boundary_point,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                        BoundaryWeight& boundary_weight);
  void CalcRefLenWeight(const BoundaryPoint& boundary_point,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                        BoundaryWeight& boundary_weight);
  void SetCurvParam(const bc::uint8_t start_idx_u8, const bc::uint8_t end_idx_u8,
                    const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length, const bc::float32_t w_lowest_f,
                    const bc::float32_t w_lower_f, const bc::float32_t w_middle_f, const bc::float32_t w_upper_f,
                    CurvParam<bc::float32_t, 5>& curve_length_cur);
  void QualityElementLength(const BoundaryPoint& boundary_point,
                            const bc::TCArray<bc::float32_t, kMaxNumBoundary> boundary_length,
                            const bc::float32_t w_lowest_f, const bc::float32_t w_lower_f,
                            const bc::float32_t w_middle_f, const bc::float32_t w_upper_f,
                            const bc::float32_t w_invalid_f, const bc::uint8_t source_type,
                            BoundaryWeight& boundary_weight);
  void QualityPerInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                            const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                            BoundaryWeight& boundary_weight);
  void QualityMapInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                            const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                            BoundaryWeight& boundary_weight);
  void QualityLdvehInterpCurv(const CurvParam<bc::float32_t, 3> t_curve_length_cur,
                              const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                              BoundaryWeight& boundary_weight);
  void QualityRefInterpCurv(const CurvParam<bc::float32_t, 5> t_curve_length_cur,
                            const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                            BoundaryWeight& boundary_weight);
  void QualityPerLength(const ClothoidModel& clothoid,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                        BoundaryWeight& boundary_weight);
  void QualityMapLength(const ClothoidModel& clothoid,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                        BoundaryWeight& boundary_weight);
  void QualityRefLength(const ClothoidModel& clothoid,
                        const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                        BoundaryWeight& boundary_weight);
  void SetLdvehCurvParam(const bc::uint8_t start_idx_u8,
                         const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                         const bc::float32_t w_upper_f, const bc::float32_t w_invalid_f,
                         CurvParam<bc::float32_t, 3>& curve_length_cur);
  void QualityLdvehElementLength(const BoundaryPoint& boundary_point,
                                 const bc::TCArray<bc::float32_t, kMaxNumBoundary>& boundary_length,
                                 const bc::float32_t w_upper_f, const bc::float32_t w_invalid_f,
                                 BoundaryWeight& boundary_weight);
  /**QualityWeightLdvehConfidence*/
  void QualityWeightLdvehConfidence(const BoundaryPack& boundary_pack, const EmParam& param_st,
                                    WeightPack& weight_pack);
  /**QualityWeightVariance*/
  void QualityBoundaryVariance(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref,
                               const BoundaryPointPack& boundary_point_pack, BoundaryWeight& boundary_weight);
  /**CalcLifetimeWeight*/
  void QualityWeightLifetime(const BoundaryPack& boundary_pack, WeightPack& weight_pack);
  /**Normalize*/
  void BoundaryNormalize(PointWeight& point_weight);

  /******************************************************************************************/
  /** input and output classes */

  /******************************************************************************************/
  /** internal classes and structures */
  bc::float32_t CalcSquRoot(const bc::float32_t x1, const bc::float32_t x2, const bc::float32_t y1,
                            const bc::float32_t y2);
  /******************************************************************************************/
  /** arrays and vectors */

  /******************************************************************************************/
  /** variables */
  WeightPack weight_pack;
  LengthPack length_pack;
  BoundaryPoint ldveh_cen_pt_pack_cs;
};
}  // namespace environment_model
}  // namespace zone
#endif
