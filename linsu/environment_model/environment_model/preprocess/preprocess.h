#ifndef POINT_BASED_EM_PREPROCESS_H
#define POINT_BASED_EM_PREPROCESS_H

#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/ego_motion_data.h"
#include "common/data/em_data.h"
#include "common/data/em_dbg_data.h"
#include "common/data/em_param.h"
// #include "common/data/fusion_object_output.h"
#include "../static_obj_process/sop.h"
#include "DiagManagerWrap.h"
#include "Diag_Fusion_Type.h"
#include "common/data/horizon_static_object.h"
#include "common/data/navigation_data.h"
#include "common/data/navigation_plus_data.h"
#include "common/data/p_fusion_freespace.h"
#include "common/data/p_fusion_road_edge.h"
#include "common/data/p_horizon_fusion_lane.h"
#include "common/data/p_horizon_fusion_object.h"
#include "common/data/p_horizon_workcondition.h"
#include "common/data/zone_shared_data.h"
#include "ehradapter/ehrdatahandle.h"
#include "environment_model_internal.h"
#include "math/bitwise_operation.h"
#include "math/boundary_point_alignment.h"
#include "math/linear_interpolation.h"
#include "math/low_pass_filter.h"
#include "math/multi_dimention_interpolation.h"
#include "math/polynomial_regression.h"
#include "math/time_sync.h"

namespace zone {
namespace environment_model {

using ::saic::diag::Diag_Event_Info;
using ::saic::diag::FUSION_MODULE;
using ::saic::diag::IPD_Diag_type;
using ::saic::diag::IPD_Function_Module;
using ::zone::common::CurvParam;
using ::zone::common::EmParam;
using ::zone::common::EmPreDbgData;
using ::zone::common::EmPreLVDbg;
using ::zone::common::EmFollowVehDbg;
using ::zone::common::EmPreMapData;
using ::zone::common::Point3D;
using ::zone::common::PreProParam;
using ::zone::common::StaticObject;
using ::zone::common::StaticObjectData;
using ::zone::common::navi::NavigationData;
using ::zone::common::navi_plus::NavigationPlusData;
using ::zone::common::navi_plus::FirstTBTGuideInfo;
using ::zone::common::Point3D;
using ::zone::common::JointPointData;
using ::zone::data::em_data::EmAdapterHDmapData;
using ::zone::data::em_data::EmData;
using ::zone::data::em_data::EmStaticObject;
using ::zone::data::em_data::EmStaticObjectData;
using ::zone::data::em_data::FusionSource;
using ::zone::data::em_data::GuideAction;
using ::zone::data::em_data::kHDMapSpdLmtSegIdx;
using ::zone::data::em_data::kHostLane;
using ::zone::data::em_data::kInvalidElementId;
using ::zone::data::em_data::kLeftLane;
using ::zone::data::em_data::kLLLane;
using ::zone::data::em_data::kMainLaneElement;
using ::zone::data::em_data::kMaxBoundaryPoint;
using ::zone::data::em_data::kMaxElemNumInOneLane;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kMaxStaticObjNum;
using ::zone::data::em_data::kRightLane;
using ::zone::data::em_data::kRRLane;
using ::zone::data::em_data::kMaxFusionCipvNum;
using ::zone::data::em_data::LaneAssociation;
using ::zone::data::em_data::LaneData;
using ::zone::data::em_data::LaneElement;
using ::zone::data::em_data::LaneTransitionDirection;
using ::zone::data::em_data::MapGuidePointType;
using ::zone::data::em_data::ObjTrackCornerPoint;
using ::zone::data::em_data::ReferenceLine;
using ::zone::data::em_data::TrafficAgentData;
using ::zone::data::em_data::BrakeLightStatus;
using ::zone::data::em_data::TurnLightStatus;
using ::zone::data::em_data::Weather;
using ::zone::data::em_data::Light;
using ::zone::fusion::LaneMarking;
using ::zone::fusion::LaneMarkings;
using ::zone::fusion::ObjTrackFusedQuality;
using ::zone::fusion::ObjTrackFusedState;
using ::zone::fusion::ObjTrackNaturalAttr;
using ::zone::fusion::ObjTracks;
using ::zone::fusion::LaneLineSource;
using ::zone::common::horizon::WorkCondition;
using ::zone::helper::math::BoundaryPointAlignment;
using ::zone::helper::math::DiagStatus;
using ::zone::helper::math::GetBitU16;
using ::zone::helper::math::GetBitU32;
using ::zone::helper::math::Lin_Interp_Method;
using ::zone::helper::math::LinearInterpolation;
using ::zone::helper::math::LowPass;
using ::zone::helper::math::TimeSync;
using ::zone::helper::math::TargetMotionModel;
using ::zone::helper::math::PolynomialRegression;
using ::zone::helper::math::SetBitU8;
using ::zone::helper::math::GetBitU8;
using ::zone::helper::math::SetBitU32;
using ::zone::helper::math::InterpCurv;
using ::zone::data::em_data::MapPerSimiliarityCheckState;
using ::zone::data::em_data::RoadEdge;
using ::zone::data::em_data::RoadEdgeType;
using ::zone::environment_model::kLeftBoundary;
using ::zone::environment_model::kRightBoundary;
using ::zone::environment_model::kSourcePer;
using ::zone::environment_model::kSourceMap;
using ::zone::environment_model::kSourceLdveh;
using ::zone::environment_model::kSourceRef;
using ::zone::data::em_data::EmLaneMarking;
using ::zone::data::em_data::kMaxLaneMarkingNum;
using ::zone::data::em_data::EmCrossStopLine;
using ::zone::data::em_data::kMaxCrossStopLineNum;
using ::zone::data::em_data::EmTrafficLight;
using ::zone::data::em_data::kMaxTrafficLightNum;
static const bc::float32_t kObjMaxLongPosition = 60.f;
static const bc::float32_t kObjMaxLatPosition = 20.f;
static const bc::float32_t kLaneChangeGapCommonBoundary = 0.4;
static const bc::float32_t kLaneChangeGapDiffBoundary = 1.8;
static const bc::float32_t kObjMinLaneAssProb = 0.5f;
static const bc::float32_t kPointGapThreshDy = 1.5f;
static const bc::uint8_t kMinValidTrajPointNum = 5;
static const bc::float32_t kDefaultLdVehLaneWidth = 3.5f;
static const bc::float32_t kPerMapLaneCheckTimeFactor = 0.5f;
static const bc::uint8_t kActivePreparationTime = 1;
/// TODO: use yaml to configue kLaneWidthTimeFactor
static const bc::float32_t kLaneWidthTimeFactor = 0.5f;
// In unilateral lane line loss case, if ego vel is lower than this boundary, use width of lst cycle
static const bc::float32_t kEgoLowSpeedBoundary = 0.1f;
static const bc::uint16_t kIdxPointInvalid = 65535;
static const bc::uint8_t kFusionLinesNum = FUSION_LINES_MAX_NUM;
static const bc::uint8_t kFusionLineCurvesNum = 10;
static const bc::float32_t kTollBoothStartDistance = 100.f;
static const bc::float32_t kTollBoothEndDistance = -100.f;
static const bc::float32_t kDistanceTolerance = 0.1f;
static const bc::float32_t kPSimiliarity = 0.9f;
static const bc::float32_t kPNSimiliarity = 0.1f;
static const bc::float32_t kPerSigmaDividingDis = 20.f;
static const bc::float64_t kMaxDeltaTimeStamp = 1.f;
static const bc::float64_t kWarningDeltaTimeStamp = 0.3f;
static const bc::uint8_t kLeftRoadedgeIndex = 8;
static const bc::uint8_t kRightRoadedgeIndex = 9;
static const bc::uint8_t kLeftRoadedgeLinePosition = 9;
static const bc::uint8_t kRightRoadedgeLinePosition = 10;
static const bc::uint8_t kMaxComparedBoundary = 3;
static const bc::uint8_t kMaxPerBoundary = 4;
static const bc::float32_t kMaxDistanceGap = 3.f;
static const bc::float32_t kLongDisHysteresis = 1.f;

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Lane-Element-boundary structure for following lane/map collection, internal use.
 * BoundaryPoint is defined in envrionment_model_internal.h */
class LaneBoundaryPoint
{
 public:
  LaneBoundaryPoint()
      : left_point_cs_(BoundaryPoint()),
        right_point_cs_(BoundaryPoint()),
        element_id_(0),
        map_unavailable_b_(bc::false_v),
        valid_b_(bc::false_v)
  {
  }
  BoundaryPoint left_point_cs_;
  BoundaryPoint right_point_cs_;
  bc::uint16_t element_id_;
  bc::bool_t map_unavailable_b_;
  bc::bool_t valid_b_;
};

class LaneElementPoint
{
 public:
  LaneElementPoint() : lane_element_point_ar_{LaneBoundaryPoint()}, valid_b_(bc::false_v) {}
  bc::TCArray<LaneBoundaryPoint, kMaxElemNumInOneLane> lane_element_point_ar_;
  bc::bool_t valid_b_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** coefficient for perception */
class PerLaneBoundaryCoeff
{
 public:
  PerLaneBoundaryCoeff()
      : coeff_cs_(ClothoidModel()),
        dx_end_f_(0.f),
        dx_start_f_(0.f),
        std_c0_f_(FLOAT32_MAX),
        std_c1_f_(FLOAT32_MAX),
        std_c2_f_(FLOAT32_MAX),
        std_c3_f_(FLOAT32_MAX),
        prob_exist_f_(0.0),
        valid_b_(bc::false_v),
        boundary_type_en_(LaneBoundaryType::kUnknown),
        boundary_color_en_(LaneBoundaryColor::kUnknown),
        per_line_id_s32_(-1),
        is_paralleled_b(bc::false_v),
        life_time_f_(0.f)
  {
  }

  ClothoidModel coeff_cs_;
  bc::float32_t dx_end_f_;
  bc::float32_t dx_start_f_;
  bc::float32_t std_c0_f_;
  bc::float32_t std_c1_f_;
  bc::float32_t std_c2_f_;
  bc::float32_t std_c3_f_;
  bc::float32_t prob_exist_f_;
  bc::bool_t valid_b_;
  LaneBoundaryType boundary_type_en_;
  LaneBoundaryColor boundary_color_en_;
  bc::int32_t per_line_id_s32_;
  bc::bool_t is_paralleled_b;
  bc::float32_t life_time_f_;
};

class PerLaneCoeff
{
 public:
  PerLaneCoeff()
      : left_coeff_cs_(PerLaneBoundaryCoeff()), right_coeff_cs_(PerLaneBoundaryCoeff()), valid_b_(bc::false_v)
  {
  }
  PerLaneBoundaryCoeff left_coeff_cs_;
  PerLaneBoundaryCoeff right_coeff_cs_;
  bc::bool_t valid_b_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Collection definition */
class PerLaneCollection
{
 public:
  PerLaneCollection() : per_coeff_ar_{PerLaneCoeff()}, per_point_ar_{LaneBoundaryPoint()} {}
  bc::TCArray<PerLaneCoeff, kMaxLaneNum> per_coeff_ar_;
  bc::TCArray<LaneBoundaryPoint, kMaxLaneNum> per_point_ar_;
};

class MapLaneCollection
{
 public:
  MapLaneCollection() : map_point_ar_{LaneElementPoint()}, map_quality_ar_(BoundaryQualityMap()) {}
  bc::TCArray<LaneElementPoint, kMaxLaneNum> map_point_ar_;
  BoundaryQualityMap map_quality_ar_;
};

class LdVehLaneCollection
{
  /** TODO: Change size to 5 in the future. Currently size=1, means only calc ego lane geometry from vehicles on ego
   * lane. */
 public:
  LdVehLaneCollection() : veh_id_u16_(0), ldveh_point_ar_{LaneBoundaryPoint()} {}
  bc::uint16_t veh_id_u16_;
  bc::TCArray<LaneBoundaryPoint, 1> ldveh_point_ar_;
};

class EgoTrackLaneCollection
{
  /** TODO: Change size to 5 lanes and 2 elements in the future. Currently size=1 means only track ego lanes. */
 public:
  EgoTrackLaneCollection() : ego_track_point_ar_{LaneElementPoint()} {}
  bc::TCArray<LaneElementPoint, kMaxLaneNum> ego_track_point_ar_;
};

enum class LaneChangeType : bc::int8_t
{
  kLaneKeeping = 0,
  kLaneChangeLeft = -1,
  kLaneChangeRight = 1,
};

enum class PerLaneWidthFilterState : bc::uint8_t
{
  kDefault = 0,
  kMeasured = 1,
  kPredicted = 2,
  kMeaToPred = 3,
  kPredToMea = 4,
};

enum class PerLaneChangeState : bc::uint8_t
{
  kDrvStraight = 0,
  kDrvLeft = 1,
  kDrvRight = 2,
};

class PerMappingResult
{
 public:
  PerMappingResult()
      : valid_b_(bc::false_v),
        lane_idx_u8_(0),
        element_idx_u8_(0),
        per_line_id_s32_(-1),
        matching_id_u8_(0),
        boundary_position_u8_(-1),  // 0 Left, 1 Right
        history_x_f_(0.f),
        history_y_f_(0.f),
        gap_y_f_(0.f),
        likelihood_f_(0.f)
  {
  }
  bc::bool_t valid_b_;
  bc::uint8_t lane_idx_u8_;
  bc::uint8_t element_idx_u8_;
  bc::int32_t per_line_id_s32_;
  bc::uint8_t matching_id_u8_;
  bc::int8_t boundary_position_u8_;
  bc::float32_t history_x_f_;
  bc::float32_t history_y_f_;
  bc::float32_t gap_y_f_;
  bc::float32_t likelihood_f_;
};

class ComparedBoundaryCollection
{
 public:
  ComparedBoundaryCollection()
      : valid_b_(bc::false_v),
        per_length_f_(0.f),
        per_x_f_(0.f),
        per_y_f_(0.f),
        max_gap_y_f_(0.f),
        max_gap_y_idx_u8_(kMaxComparedBoundary),
        dy_ar_{0.f},
        start_idx_u8_(0),
        end_idx_u8_(0),
        result_count_u8_(0),
        max_likelihood_idx_u8_(kMaxComparedBoundary),
        target_lane_idx_u8_(0),
        target_element_idx_u8_(0),
        target_boundary_position_u8_(-1),
        compared_boundary_ar_({PerMappingResult()})
  {
  }
  bc::bool_t valid_b_;
  bc::float32_t per_length_f_;
  bc::float32_t per_x_f_;
  bc::float32_t per_y_f_;
  bc::float32_t max_gap_y_f_;
  bc::uint8_t max_gap_y_idx_u8_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dy_ar_;
  bc::uint8_t start_idx_u8_;
  bc::uint8_t end_idx_u8_;
  bc::uint8_t result_count_u8_;
  bc::uint8_t max_likelihood_idx_u8_;
  bc::uint8_t target_lane_idx_u8_;          // PostProcess中确认
  bc::uint8_t target_element_idx_u8_;       // PostProcess中确认
  bc::int8_t target_boundary_position_u8_;  // PostProcess中确认
  bc::TCArray<PerMappingResult, kMaxComparedBoundary> compared_boundary_ar_;
};

class PerLifeTimeCollection
{
 public:
  PerLifeTimeCollection() : line_id_s32_(0), initial_life_time_f_(0.f), used_life_time_f_(0.f), valid_b_(bc::false_v) {}
  bc::int32_t line_id_s32_;
  bc::float32_t initial_life_time_f_;
  bc::float32_t used_life_time_f_;
  bc::bool_t valid_b_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Preprocess Definition */
class Preprocess
{
  Preprocess(const Preprocess&) = delete;
  void operator=(const Preprocess&) = delete;

 public:
  Preprocess(EmData& em_collection);
  ~Preprocess() = default;

  void SetParam();

  void Run(const ObjTracks& per_obj_cs, const LaneMarkings& per_lane_cs, const StaticObjectData& per_static_obj_cs,
           const FreespaceList& per_freespace_cs, const EgoMotionData& ego_motion_cs, const EhrToEmData& ehr2em_cs,
           const NavigationData& navi_cs, const NavigationPlusData& navi_plus_cs, const WorkCondition& world_info_cs,
           const EmData& em_output_lst1_cs, const EmParam& param_st,
           const bc::TCArray<bc::bool_t, kNumInputChk> input_update_ar, const bc::float32_t time_cycle_f,
           const StEndIdxPack& last_em_st_end_idx_, const bc::float64_t& time_stamp_f64, BoundaryPack& boundary_pack_cs,
           SemanticPack& semantic_pack_cs, EmData& em_output_cs, StaticObjProcess& static_obj_process_cs_);

  bc::TCArray<bc::float32_t, kMaxNumBoundary> getCoordinateRef() const { return x_ref_ar_; }
  EgoPoseCollection& getEgoPoseCollection() { return ego_pose_collection_cs_; }
  PerObjCollection& getPerObjCollection() { return obj_collection_cs_; }
  bc::int8_t& getMapLaneChangeDir() { return map_lane_change_s8; }
  bc::int8_t& getRefLaneChangeDir() { return lane_match_s8; }
  bc::uint16_t GetMapHostLaneElementId() { return map_host_lane_element_id_; }
  bc::float32_t GetMapDiagTime() { return debug_map_time_; }
  bc::float32_t GetMapDiagStatus() { return debug_map_status_; }
  bc::float32_t GetRawMapElmId0() { return raw_map_ele_id_0; }
  bc::float32_t GetRawMapElmId1() { return raw_map_ele_id_1; }
  bc::float32_t GetRawMapElmId2() { return raw_map_ele_id_2; }
  bc::float32_t GetRawMapElmId3() { return raw_map_ele_id_3; }
  bc::float32_t GetRawMapElmId4() { return raw_map_ele_id_4; }
  bc::TCArray<InputDiagInfo, kNumInputChk> input_diag_manager_ar_;
  bc::TCArray<InputDiagInfo, kNumInputChk> input_diag_manager_lst1_ar_;
  void GetDbgData(EmPreDbgData& dbg_data_cs);
  InputDiagInfo GetInputDiagInfo(const bc::uint8_t idx_input_u8);
  bc::float32_t GetLeftMapPerSimilarityData() { return left_map_per_similarity_f_; }
  bc::float32_t GetRightMapPerSimilarityData() { return right_map_per_similarity_f_; }
  MapPerSimiliarityCheckState GetleftMapPerIsCheckedData() { return left_similiarity_check_state_; }
  MapPerSimiliarityCheckState GetrightMapPerIsCheckedData() { return right_similiarity_check_state_; }

  bc::float32_t GetLeftPostP() { return left_post_map_per_similarity_f_; }
  bc::float32_t GetRightPostP() { return right_post_map_per_similarity_f_; }

  zone::data::em_data::ScenarioClassification GetTollBoothScenario() { return tollbooth_scenario_cs_; }

  bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum> GetMapLanemarking() { return map_adapter_cs_.lane_marking_ar_; }
  bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum> GetMapCrossStopLine()
  {
    return map_adapter_cs_.cross_stop_line_ar_;
  }
  bc::TCArray<EmTrafficLight, kMaxTrafficLightNum> GetMapTrafficLight() { return map_adapter_cs_.traffic_light_ar_; }
  zone::data::em_data::EmIntersectionData GetMapIntersection() { return map_adapter_cs_.intersection_data_; }

  void Reset()
  {
    ego_pose_collection_cs_ = EgoPoseCollection();
    obj_traj_collection_cs_ = ObjectTrajectoryCollection();
    map_lane_collection_cs_ = MapLaneCollection();
    ldveh_lane_collection_cs_ = LdVehLaneCollection();
    ego_track_lane_collection_cs_ = EgoTrackLaneCollection();
    obj_collection_cs_ = PerObjCollection();
    per_lane_collection_cs_ = PerLaneCollection();
    map_adapter_cs_ = EmAdapterHDmapData();
    temp_map_adapter_ = EmAdapterHDmapData();
    for (bc::uint8_t t_i_u8 = 0; t_i_u8 < kMaxNumBoundary; t_i_u8++)
    {
      x_ref_ar_[t_i_u8] = 0.f;
    }
    ld_veh_lst1_id_u16_ = kInvalidObjectId;
    ld_veh_lst1_idx_u8_ = kInvalidObjIdx;
    // last_heading_angle_ = 0.0;
    filtered_delta_heading_ = 0.f;
    filtered_delta_offset_ = 0.f;
    filtered_ego_curve_ = 0.f;
    ego_curve_filter_b_ = bc::false_v;
  }

 private:
  /******************************************************************************************/
  /** function */
  /** @brief Check the validity of input sources, and manage their diagnosis information.
   *  @param per_obj_cs, objects from perception
   *  @param per_lane_cs, lane from perception
   *  @param per_road_edges_cs, road ege from perception
   *  @param per_freespace_cs, free space from perception
   *  @param ego_motion_cs, ego motion
   *  @param ehr2em_cs, ehr from map
   *  @param input_update_ar, an array indicating if each input node is updated
   */
  void InputValidityCheck(const ObjTracks& per_obj_cs, const LaneMarkings& per_lane_cs,
                          const StaticObjectData& per_static_obj_cs, const NavigationData& navi_cs,
                          const NavigationPlusData& navi_plus_cs, const FreespaceList& per_freespace_cs,
                          const EgoMotionData& ego_motion_cs, const EhrToEmData& ehr2em_cs,
                          const WorkCondition& world_info_cs, const EmParam& param_st,
                          const bc::TCArray<bc::bool_t, kNumInputChk> input_update_ar,
                          const bc::float32_t time_cycle_f);

  /** @brief Calc an array of x coordinate in Cartesian coordination, this is the reference to discrete lanes. The
   * output is x_ref_ar_.
   */
  void CalcDiscreteCoordinate(const EmParam& param_st);

  /** @brief Store ego motion data into ego_pose_collection_cs.
   *  @param ego_motion_cs, ego motion data
   */
  void EgoMotionProcess(const EgoMotionData& ego_motion_cs, const PreProParam prep_param_st,
                        const bc::float32_t time_cycle_f);

  /** @brief Mapping object list to internal variables and do time sycn. Input is per_obj_cs, output is
   * obj_collection_cs_.
   *  @param per_obj_cs, objects from perception
   */
  void PerObjProcess(const ObjTracks& per_obj_cs, const bc::float32_t time_cycle_f, const PreProParam prep_param_st);

  /** @brief Mapping lane info from perception to internal variables, do time sycn and discrete according to x_ref.
   * Input is per_lane_cs, output is per_lane_collection_cs_.
   *  @param per_lane_cs, lane from perception
   *  @param em_lane_lst1_ar, lane from last cycle em
   */
  void PerLaneProcess(const LaneMarkings& per_lane_cs, const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                      const PreProParam prep_param_st, const bc::float32_t time_cycle_f, const bc::float32_t ego_vel_f);

  void MapProcess(const EhrToEmData& ehr2em_cs, const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                  const PreProParam prep_param_st, const bc::float32_t time_cycle_f,
                  EmAdapterHDmapData& map_adapter_cs);

  void WorldInfoProcess(const WorkCondition& world_info_cs, const EmData& em_output_lst1_cs);

  void ReorganizeMapData(EmAdapterHDmapData& map_em_data);
  void MapPerCrossCheck(const MapLaneCollection& map_lane_data, const PerLaneCollection& per_lane_data,
                        const EmParam& param_st, bc::float32_t time_cycle_f);

  void CheckJumpPoint();
  /** @brief Tracking relevant objects' history trajectories, choose the LdVeh and maintain the virtual lane geometry
   * derived from LdVeh. Input is obj_collection_cs, outputs are obj_traj_collection_cs_ and ldveh_lane_collection_cs_.
   *  @param obj_collection_cs, internal object collection
   *  @param em_output_lst1_cs, last EM output
   *  @param time_cycle_f, cycle time between current and last cycle
   */
  void LdVehLaneTrack(const PerObjCollection& obj_collection_cs, const EmData& em_output_lst1_cs,
                      const bc::float32_t& time_cycle_f, const PreProParam& prep_param_st);

  /** @brief Tracking ego lane geometry from last EM output and align to x_ref. Input is em_output_lst1_cs and cycle
   * time, and output is ego_track_lane_collection_cs_.
   *  @param em_output_lst1_cs, last EM output
   *  @param time_cycle_f, cycle time between current and last cycle
   */
  void EgoLaneTrack(const EmData& em_output_lst1_cs, const bc::float32_t& time_cycle_f,
                    const StEndIdxPack& last_em_st_end_idx_, const PreProParam& prep_param_st,
                    const EmAdapterHDmapData& map_adapter_cs);
  void DoRefMapMatch(const EmAdapterHDmapData& map_adapter_cs);
  bc::uint8_t FindXRefIdx(const bc::float32_t temp_x);
  bc::uint8_t FindXRefStartIdx(const bc::float32_t temp_x);
  void PerMapMatching(const EmData& em_output_lst1_cs, const EhrToEmData& ehr2em_cs, const LaneMarkings& per_lane_cs,
                      const bc::float32_t& time_cycle_f, const EgoMotionData& ego_motion_data_st,
                      EmAdapterHDmapData& map_adapter_cs);
  void BoundaryRepack(const EhrToEmData& ehr2em_cs, const EmAdapterHDmapData& map_adapter_cs,
                      const float32_t joint_similarity_threshold, BoundaryPack& boundary_pack_cs);
  void BoundaryJoint(const float32_t joint_similarity_threshold, BoundaryPack& boundary_pack_cs);
  void SemanticRepack(const StaticObjectData& per_static_obj_cs, const EmAdapterHDmapData& map_adapter_cs,
                      const NavigationPlusData& navi_plus_cs, SemanticPack& semantic_pack_cs);
  void PerSpdLmtSemanticRepack(const StaticObjectData& per_static_obj_cs, SemanticPack& semantic_pack_cs);
  void MapSemanticRepack(const EmAdapterHDmapData& map_adapter_cs, SemanticPack& semantic_pack_cs);
  void SDSemanticRepack(const NavigationPlusData& navi_plus_cs, SemanticPack& semantic_pack_cs);
  void StaticObjInfoTransmission(const StaticObjectData& per_static_obj_cs);
  bc::bool_t GetRoadedgeType(RoadEdge& roadedge_cs, LaneLineSource per_roadedge_type_s32);
  void StoreSingleRoadedge(RoadEdge& roadedge_cs, const LaneMarking& per_roadedge_cs, bc::uint8_t line_pos_u8);
  void RoadedgeStore(const LaneMarkings& per_lane_cs);
  void ValidityCheckPerObj(const ObjTracks& per_obj_cs, const bc::float32_t time_cycle_f,
                           const bc::bool_t input_update_b);
  void ValidityCheckPerLane(const LaneMarkings& per_lane_cs, const bc::float32_t time_cycle_f,
                            const bc::bool_t input_update_b);
  void ValidityCheckMap(const EhrToEmData& ehr2em_cs, const bc::float32_t time_cycle_f,
                        const bc::bool_t input_update_b);
  void ValidityCheckEgoMotion(const EgoMotionData& ego_motion_cs, const bc::float32_t time_cycle_f,
                              const bc::bool_t input_update_b);
  void ValidityCheckPerStaticObj(const StaticObjectData& per_static_obj_cs, const bc::float32_t time_cycle_f,
                                 const bc::bool_t input_update_b);
  void ValidityCheckNavi(const NavigationData& navi_cs, const bc::float32_t time_cycle_f,
                         const bc::bool_t input_update_b);
  void ValidityCheckNaviPlus(const NavigationPlusData& navi_plus_cs, const bc::float32_t time_cycle_f,
                             const bc::bool_t input_update_b);
  void ValidityCheckPerFreeSpace(const FreespaceList& per_freespace_cs, const bc::float32_t& time_cycle_f,
                                 const bc::bool_t& input_update_b);
  void ValidityCheckWorldInfo(const WorkCondition& world_info_cs, const bc::float32_t time_cycle_f,
                              const bc::bool_t input_update_b);

  void TimeSyncObj();
  void TimeSyncPerLane();
  void TimeSyncMap(const LaneMarkings& per_lane_cs, const EgoMotionData& ego_motion_data_st,
                   EmAdapterHDmapData& map_em_data);

  void PerLaneMapping(const LaneMarkings& per_lane_cs,
                      const bc::TCArray<LaneElementPoint, kMaxLaneNum>& ref_collection_ar,
                      const bc::uint8_t left_source_type_u8, const bc::uint8_t right_source_type_u8,
                      const bc::float32_t time_cycle_f);
  void DiscretePerLane();

  /** @brief Tracking relevant objects history trajectories using this cycle perception object collection with some
   *  rules and clear unnecessary history trajectories in each cycle.
   *  @param obj_collection_cs internal perception object collection
   */
  void ObjHistTrajTrack(const PerObjCollection& obj_collection_cs,
                        const bc::TCArray<TrafficAgentData, kFusMaxObjNum>& last_em_agents);

  /** @brief Select the leading vehicle based on the specific rules and generate a series of dy according to given
   *  x_ref_.
   *  @param em_output_lst1_cs, last EM output
   *  @param time_cycle_f, cycle time between current and last cycle
   */
  void LdVehLaneGen(const EmData& em_output_lst1_cs, const PreProParam& prep_param_st,
                    const bc::float32_t time_cycle_f);

  void DynamicAdaptiveCoordinate(bc::float32_t& dx_f, bc::float32_t vel_bm_f);
  void PerObjTrack(const bc::float32_t dt_f, FusionObj& obj_cs);
  void StorePerObjsToInternal(const ObjTracks& per_obj_cs);
  void StorePerLineToInternal(const LaneMarking& external_cs, const bc::int8_t target_lane_u8,
                              const bc::uint8_t target_element_u8, const bc::int8_t target_boundary_u8);
  void DiscretePerLaneBoundary(const PerLaneBoundaryCoeff& per_coeff_cs, BoundaryPoint& per_points_cs);
  bc::float32_t FindPointDy(const bc::float32_t dx, const LaneBoundary& lane_boundary, const bc::uint8_t end_idx_u8);
  void TrackBoundaryPoints(bc::TCArray<Point3D, kMaxBoundaryPoint>& points_ar, Point2D& end_point,
                           Point2D& start_point);
  void TrackLstCrossPoint(JointPointData& joint_point);
  void TrackSegsPoint(const LaneBoundarySegment& segment, float32_t& start_dx_f, float32_t& end_dx_f);
  void TrackPointDx(const bc::TCArray<bc::float32_t, 4> rot_matrix_ar, const bc::float32_t translation_dx_f_,
                    const float32_t translation_dy_f_, const Point2D& track_point, bc::float32_t& track_dx);
  /** Copy points to internal.*/
  /** input_valid_num: berore alignment valid total num  input_ar
   *  input_st_idx_u8: before alignment start idx  input_ar
   *  ref_output_st_idx_u8: after transform real start idx -- for ref
   *  ref_output_end_idx_u8: after transform real end idx -- for ref
   */
  bc::bool_t StoreTrackBoundaryPoints(const bc::TCArray<Point3D, kMaxBoundaryPoint>& points_ar,
                                      BoundaryPoint& track_points_cs, const bc::uint8_t input_valid_num,
                                      const bc::uint8_t input_st_idx_u8, const bc::uint8_t ref_output_st_idx_u8 = 0,
                                      const bc::uint8_t ref_output_end_idx_u8 = kMaxBoundaryPoint - 1,
                                      const bc::bool_t ref_source_b = bc::false_v);
  void IndividualBoundaryRepack(BoundaryPointPack& repack_1boundary_cs, bc::uint8_t lane_idx_u8, bc::uint8_t ele_idx_u8,
                                bc::uint8_t l_or_r_u8, bc::bool_t near_intersection_b);
  void IndividualBoundaryJoint(const bc::uint8_t dir_u8, const float32_t joint_similarity_threshold,
                               BoundaryPointPack& boundary_point_pack);
  void SourceBoundaryJoint(const bc::uint8_t dir_u8, const bc::uint8_t source_u8, const BoundaryPoint& ref_boundary,
                           BoundaryPoint& tar_boundary);
  bc::uint8_t FindCrossingPoint(const BoundaryPoint& ref_boundary, BoundaryPoint& tar_boundary, Point2D& cross_point);
  Point2D FindCrossPoint(const Point2D& p_A, const Point2D& p_B, const Point2D& p_C, const Point2D& p_D);
  void DeductOneLaneBoundary(BoundaryPack& boundary_pack_cs, bc::uint8_t lane_idx);
  void DeductPerBoundary(const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar, const bc::float32_t time_cycle_f,
                         const bc::float32_t ego_vel_f);
  void ParallelLaneGenerator();
  void StoreHDMapLaneInfoToInternal(const EmAdapterHDmapData& map_adapter_cs);
  void StoreMapLineToInternal(const LaneBoundary& external_bound_cs, BoundaryPoint& internal_bound_cs);
  void LaneBoundaryOffsetsForMatching(const LaneData& lane_cs, const bc::TCArray<bc::uint8_t, 4> end_ar,
                                      bc::TCArray<bc::float32_t, 4>& offset_ar);
  void FixElementBoundaryBend(EmAdapterHDmapData& map_adapter_cs);
  void MapMatchingWithLastEm(const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                             EmAdapterHDmapData& map_adapter_cs);
  bc::bool_t MapDataAdapter(const LaneMarkings& per_lane_cs, const bc::float32_t time_cycle_f,
                            const EgoMotionData& ego_motion_data_st, EmAdapterHDmapData& map_em_data);
  void TransformMapdata(const bc::float32_t time_cycle_f, EmAdapterHDmapData& map_em_data);
  void CalcC1(const LaneBoundary& boundary, const bc::uint8_t& ego_idx, bc::float32_t& t_c1);
  void SplitMapLaneData(EmAdapterHDmapData& map_em_data);
  void MapScenarioDistinction(EmAdapterHDmapData& map_em_data);
  void FindMarginalBoundaryPtsWithSameDx(const LaneBoundary& ele0_boundary, const LaneBoundary& ele1_boundary,
                                         bc::uint8_t& min_ele0_index, bc::uint8_t& min_ele1_index,
                                         bc::uint8_t& max_ele0_index, bc::uint8_t& max_ele1_index);
  /** @brief Calculate and Output Diagnosis Management Status*/
  void DiagnosticManagement(const bc::float64_t& time_stamp_f64);
  void DiagEventPushBack(const DiagMan& man_cs, const DiagMan& man_older_cs, const FUSION_MODULE& event_name_en,
                         Diag_Event_Info& event_info);
  void OutputDiagInfo(const bc::TCArray<InputDiagInfo, kNumInputChk>& input_diag_manager_ar, EmData& em_output_cs);
  bc::float32_t CheckMapPerLaneSimilarity(const bc::TCArray<bc::float32_t, kMaxNumBoundary> cur_x_ref,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& per_boundary,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& ref_boundary,
                                          const bc::uint8_t start_index, bc::uint8_t end_index);
  void FilterSimilarBoundary(const BoundaryPoint& lane_boundary, const bc::uint8_t lane_idx,
                             const bc::uint8_t element_idx, const bc::uint8_t boundary_flag,
                             ComparedBoundaryCollection& boundary_collection);
  void PerRefMatchingPostProcess(bc::TCArray<ComparedBoundaryCollection, kMaxPerBoundary>& per_ref_mapping);
  void StorePerLineToInternalWithMatchingId(const LaneMarking& external_cs, const bc::uint8_t target_lane_u8,
                                            const bc::uint8_t target_element_u8, const bc::int8_t target_boundary_u8,
                                            const bc::TCArray<LaneElementPoint, kMaxLaneNum>& ref_collection_ar);
  void DetectParallelLane();
  void DoRefPerceptionMatch(const PerLaneCollection& per_collection_cs);
  void ScenarioRecognition();

  bc::float32_t CheckMapPerLaneSimilarity(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& per_boundary,
                                          const bc::TCArray<bc::float32_t, kMaxNumBoundary>& ref_boundary,
                                          const EmParam& param_st, const bc::uint8_t start_index,
                                          bc::uint8_t end_index);

  MapPerSimiliarityCheckState MapPerSimiliarityStateUpdate(const MapPerSimiliarityCheckState state,
                                                           const bc::float32_t time_cycle_f,
                                                           const bc::bool_t valid_similarity_b,
                                                           bc::float32_t& accumu_time);

  void NaviGuideInfo(const NavigationPlusData& navi_plus_cs, const EmData& em_output_lst1_cs);
  /******************************************************************************************/
  /** input and output classes */

  /******************************************************************************************/
  /** internal classes and structures */
  EmData* em_collection_ptr_;
  EgoPoseCollection ego_pose_collection_cs_;
  ObjectTrajectoryCollection obj_traj_collection_cs_;
  MapLaneCollection map_lane_collection_cs_;
  LdVehLaneCollection ldveh_lane_collection_cs_;
  EgoTrackLaneCollection ego_track_lane_collection_cs_;
  PerObjCollection obj_collection_cs_;
  PerLaneCollection per_lane_collection_cs_;
  EmAdapterHDmapData map_adapter_cs_;
  EmAdapterHDmapData temp_map_adapter_;
  EhrDataHandle ehrdatahandle_;
  EhrToEmData lst_ehr2em_cs_;
  LaneMarkings lst_perlane_cs_;
  WorkCondition lst_world_info_cs_;
  EmPreLVDbg lv_dbg_cs_;
  TimeSync time_sync_cs_;
  int32_t lst_left_line_id_s32_ = -1;
  int32_t lst_right_line_id_s32_ = -1;
  PerLaneCoeff lst_per_host_lane_coeff_cs_;
  PerLaneCollection per_lane_collection_lst_cs_;
  /******************************************************************************************/
  /** arrays and vectors */

  bc::TCArray<bc::float32_t, kMaxNumBoundary> x_ref_ar_;
  bc::TCArray<ComparedBoundaryCollection, kMaxPerBoundary> per_mapping_collection_ar_;

  /** 0   left 0   per0
   *  1   right 1  per0
   *  2   left 0   ldveh2
   *  3   right 1  ldveh2 */
  bc::TCArray<JointPointData, 4> joint_points_collection_cs_;
  bc::float32_t left_per_ref_similarity_f_ = 0.f;
  bc::float32_t right_per_ref_similarity_f_ = 0.f;
  bc::TCArray<PerLifeTimeCollection, kFusionLinesNum> lst_per_life_time_ar_;
  bc::TCArray<PerLifeTimeCollection, kFusionLinesNum> new_per_life_time_ar_;
  /******************************************************************************************/
  /** variables */
  bc::bool_t modified_ref_b = bc::false_v;
  bc::uint8_t one_to_two_lane_ctn_u8_ = 0;
  bc::uint16_t ld_veh_lst1_id_u16_ = kInvalidObjectId;
  bc::uint8_t ld_veh_lst1_idx_u8_ = kInvalidObjIdx;
  // bc::float64_t last_heading_angle_ = 0.0;
  // bc::TCArray<bc::TCArray<bc::uint8_t, 2>, 5> last_em_end_idx_;
  bc::float32_t filtered_delta_heading_ = 0.0f;
  bc::float32_t filtered_delta_offset_ = 0.0f;
  bc::float32_t filtered_ego_curve_ = 0.0f;
  bc::uint8_t per_lc_reason_u8_ = 0;
  bc::float32_t map_end_x_ = 0.0f;
  bc::float32_t map_end_idx_ = 0.0f;
  bc::int8_t map_lane_change_s8 = 0;
  bc::int8_t lane_match_s8 = 0;
  bc::uint16_t map_host_lane_element_id_ = 0;
  bc::float32_t debug_map_time_ = 0.0f;
  bc::float32_t debug_map_status_ = 0.0f;
  bc::float32_t raw_map_ele_id_0 = 0.0f;
  bc::float32_t raw_map_ele_id_1 = 0.0f;
  bc::float32_t raw_map_ele_id_2 = 0.0f;
  bc::float32_t raw_map_ele_id_3 = 0.0f;
  bc::float32_t raw_map_ele_id_4 = 0.0f;
  bc::bool_t ehr_analy_b_ = false;
  bc::bool_t per_analy_b_ = false;
  bc::bool_t world_info_analy_b_ = false;
  bc::bool_t ego_curve_filter_b_ = bc::false_v;
  bc::float32_t left_map_per_similarity_f_ = 0;
  bc::float32_t right_map_per_similarity_f_ = 0;
  MapPerSimiliarityCheckState left_similiarity_check_state_ = MapPerSimiliarityCheckState::kOff;
  MapPerSimiliarityCheckState right_similiarity_check_state_ = MapPerSimiliarityCheckState::kOff;
  bc::float32_t left_accumu_time_ = 0.f;
  bc::float32_t right_accumu_time_ = 0.f;

  bc::float32_t left_post_map_per_similarity_f_ = 0.99f;
  bc::float32_t right_post_map_per_similarity_f_ = 0.99f;

  EmPreMapData map_dbg_data_;

  bc::uint8_t lst_cross_point_idx_ = 0;
  Point2D lst_cross_point_ = Point2D(0.f, 0.f);
  bc::bool_t same_per_b_ = bc::false_v;
  bc::bool_t still_exist_b_ = bc::false_v;
  bc::float32_t per_lane_width_lst1_f_ = kDefaultElementWidth;
  bc::float32_t per_left_c0_lst_f_ = 0.f;
  bc::float32_t per_right_c0_lst_f_ = 0.f;
  bc::float32_t target_width_f_ = kDefaultElementWidth;
  PerLaneChangeState per_lane_change_state_ = PerLaneChangeState::kDrvStraight;
  // Left boundary in index 0 and right boundary in index 1
  bc::TCArray<PerLaneWidthFilterState, 2> per_lane_width_filter_state_ar_ = {PerLaneWidthFilterState::kDefault};
  zone::data::em_data::ScenarioClassification tollbooth_scenario_cs_;
  zone::common::Point left_boundary_ptn_;
  zone::common::Point right_boundary_ptn_;
};
}  // namespace environment_model
}  // namespace zone
#endif