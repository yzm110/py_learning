#ifndef POINT_BASED_EM_INTERNAL_H
#define POINT_BASED_EM_INTERNAL_H

#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/em_data.h"
#include "common/data/horizon_static_object.h"
#include "common/data/p_vision_tsr.h"
#include "common/data/zone_shared_data.h"
#include "ego_pose.h"
#include "math/diagnosis_management.h"
#include "trajectory.h"

namespace zone {
namespace environment_model {

static const bc::uint8_t kLeftLeftLine = 0;
static const bc::uint8_t kHostLeftLine = 1;
static const bc::uint8_t kHostRightLine = 2;
static const bc::uint8_t kRightRightLine = 3;
static const bc::uint8_t kPerParallelLineNum = 4;
static const bc::uint8_t kMaxNumRearBoundary = 20;
static const bc::uint8_t kFrontStartIdx = kMaxNumRearBoundary;
static const bc::uint8_t kMaxNumBoundary = 100;
static const bc::uint8_t kMaxNumFrontBoundary = kMaxNumBoundary - kMaxNumRearBoundary;
static const bc::uint8_t MAX_NUMBER_OF_COURSE_SEGMENTS = 8U;
static const bc::uint8_t kLeftBoundary = 0;
static const bc::uint8_t kRightBoundary = 1;
static const bc::float32_t kDefaultElementWidth = 3;
static const bc::int8_t kLaneChangeToLeft = 1;
static const bc::int8_t kLaneChangeToLL = 2;
static const bc::int8_t kLaneChangeToRight = -1;
static const bc::int8_t kLaneChangeToRR = -2;
static const bc::float32_t kLaneChangeDebounceDy = 0.3f;
static const bc::float32_t kSpecialDirDistance = 50.f;
static const bc::uint8_t kSourcePer = 0;
static const bc::uint8_t kSourceMap = 1;
static const bc::uint8_t kSourceLdveh = 2;
static const bc::uint8_t kSourceRef = 3;
using ::zone::common::ClothoidModel;
using ::zone::data::em_data::FusionSource;
using ::zone::data::em_data::kFusMaxObjNum;
using ::zone::data::em_data::kInputChkPerObj;
using ::zone::data::em_data::kInputChkPerLane;
using ::zone::data::em_data::kInputChkPerRdEdge;
using ::zone::data::em_data::kInputChkPerFreeSp;
using ::zone::data::em_data::kInputChkEgoMotion;
using ::zone::data::em_data::kInputChkMap;
using ::zone::data::em_data::kInputChkPerTsr;
using ::zone::data::em_data::kInputChkRef;
using ::zone::data::em_data::kInputChkLdveh;
using ::zone::data::em_data::kInputChkNavi;
using ::zone::data::em_data::kInputChkNaviPlus;
using ::zone::data::em_data::kInputChkSlam;
using ::zone::data::em_data::kInputChkPerStaticObj;
using ::zone::data::em_data::kInputChkWorldInfo;
using ::zone::data::em_data::kNumInputChk;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kMaxMapGuidePtNum;
using ::zone::data::em_data::kMaxLaneMarkingNum;
using ::zone::data::em_data::kMaxCrossStopLineNum;
using ::zone::data::em_data::kMaxTrafficLightNum;
using ::zone::data::em_data::LaneBoundary;
using ::zone::data::em_data::LaneBoundaryColor;
using ::zone::data::em_data::LaneBoundaryType;
using ::zone::data::em_data::MotionPattern;
using ::zone::data::em_data::ObjCategoryType;
using ::zone::data::em_data::ObjTrackCornerPoint;
using ::zone::data::em_data::RefLineSegSpeedLimit;
using ::zone::data::em_data::BrakeLightStatus;
using ::zone::data::em_data::TurnLightStatus;
using ::zone::helper::math::DiagMan;
using ::zone::data::em_data::EmLaneMarking;
using ::zone::data::em_data::EmCrossStopLine;
using ::zone::data::em_data::EmTrafficLight;
using ::zone::data::em_data::EmIntersectionData;
using ::zone::data::em_data::EmIntersectionData;

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Per Object collection structure definition */
class FusionObj
{
 public:
  FusionObj()
      : corner_point_cs_(),
        id(0),
        category(ObjCategoryType::kUnknown),
        fusion_source(FusionSource::kUnknown),
        motion_pattern(MotionPattern::kUnknown),
        brake_light_status(BrakeLightStatus::kUnknown),
        turn_light_status(TurnLightStatus::kUnknown),
        x(0.f),
        y(0.f),
        heading(0.f),
        yaw(0.f),
        yawrate(0.f),
        length(0.f),
        width(0.f),
        height(0.f),
        distance_to_left_line(0.f),
        distance_to_right_line(0.f),
        long_velocity_abs(0.f),
        lat_velocity_abs(0.f),
        long_velocity_relative(0.f),
        lat_velocity_relative(0.f),
        long_acceleration_abs(0.f),
        lat_acceleration_abs(0.f),
        long_acceleration_relative(0.f),
        lat_acceleration_relative(0.f),
        long_position_std_dev(0.f),
        lat_position_std_dev(0.f),
        is_cipv(false),
        is_valid(bc::false_v)
  {
  }
  ObjTrackCornerPoint corner_point_cs_;
  bc::uint16_t id;
  ObjCategoryType category;
  FusionSource fusion_source;
  MotionPattern motion_pattern;
  BrakeLightStatus brake_light_status;
  TurnLightStatus turn_light_status;
  bc::float32_t x;
  bc::float32_t y;
  bc::float32_t heading;
  bc::float32_t yaw;
  bc::float32_t yawrate;
  bc::float32_t length;
  bc::float32_t width;
  bc::float32_t height;
  bc::float32_t distance_to_left_line;
  bc::float32_t distance_to_right_line;
  bc::float32_t long_velocity_abs;
  bc::float32_t lat_velocity_abs;
  bc::float32_t long_velocity_relative;
  bc::float32_t lat_velocity_relative;
  bc::float32_t long_acceleration_abs;
  bc::float32_t lat_acceleration_abs;
  bc::float32_t long_acceleration_relative;
  bc::float32_t lat_acceleration_relative;
  bc::float32_t long_position_std_dev;
  bc::float32_t lat_position_std_dev;
  bc::bool_t is_cipv;
  bc::bool_t is_valid;
};

class PerObjCollection
{
 public:
  PerObjCollection() : objs_ar_{FusionObj()} {}
  bc::TCArray<FusionObj, kFusMaxObjNum> objs_ar_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
/** Per Roadedge collection structure definition */
class FusionRoadedge
{
 public:
  FusionRoadedge() : coeff(), is_valid(bc::false_v) {}
  ClothoidModel coeff;
  bc::bool_t is_valid = bc::false_v;
};
class PerRdEdgeCollection
{
 public:
  PerRdEdgeCollection() : roadedge_ar_() {}
  bc::TCArray<FusionRoadedge, 2> roadedge_ar_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Boundary Pack structure definition */
class BoundaryPoint
{
 public:
  BoundaryPoint()
      : dy_ar_{0.0},
        boundary_type_en_(LaneBoundaryType::kUnknown),
        boundary_color_en_(LaneBoundaryColor::kUnknown),
        dx_end_f_(0.0),
        dx_start_f_(0.0),
        dy_end_idx_u8_(0),
        dy_start_idx_u8_(0),
        total_valid_num_u8_(0),
        source_type_(0),
        valid_b_(bc::false_v),
        per_line_id_s32_(-1),
        matching_id_u8_(0),
        per_life_time_(0.f)
  {
  }
  bc::TCArray<bc::float32_t, kMaxNumBoundary> dy_ar_;
  LaneBoundaryType boundary_type_en_;
  LaneBoundaryColor boundary_color_en_;
  bc::float32_t dx_end_f_;
  bc::float32_t dx_start_f_;
  bc::uint8_t dy_end_idx_u8_;
  bc::uint8_t dy_start_idx_u8_;
  bc::uint8_t total_valid_num_u8_;
  bc::uint8_t source_type_;
  bc::bool_t valid_b_;
  bc::int32_t per_line_id_s32_;
  bc::uint8_t matching_id_u8_;
  bc::float32_t per_life_time_;
};

class BoundaryQualityPer
{
 public:
  BoundaryQualityPer()
  {
    std_c0_f_ = FLOAT32_MAX;
    std_c1_f_ = FLOAT32_MAX;
    std_c2_f_ = FLOAT32_MAX;
    std_c3_f_ = FLOAT32_MAX;
    prob_exist_f_ = 0.0;
    valid_b_ = bc::false_v;
  }
  bc::float32_t std_c0_f_;
  bc::float32_t std_c1_f_;
  bc::float32_t std_c2_f_;
  bc::float32_t std_c3_f_;
  bc::float32_t prob_exist_f_;
  bc::bool_t valid_b_;
  ClothoidModel coeff_;
};

class BoundaryQualityMap
{
 public:
  BoundaryQualityMap() : prob_exist_f_(0.0), valid_b_(bc::false_v) {}
  bc::float32_t prob_exist_f_;
  bc::bool_t valid_b_;
};

class BoundaryPointPack
{
 public:
  BoundaryPointPack()
  {
    per_cs_ = BoundaryPoint();
    map_cs_ = BoundaryPoint();
    ldveh_cs_ = BoundaryPoint();
    ref_cs_ = BoundaryPoint();
    per_quality_cs_ = (BoundaryQualityPer());
    map_quality_cs_ = BoundaryQualityMap();
    valid_b_ = bc::false_v;
  }
  BoundaryPoint per_cs_;
  BoundaryPoint map_cs_;
  BoundaryPoint ldveh_cs_;
  BoundaryPoint ref_cs_;
  BoundaryQualityPer per_quality_cs_;
  BoundaryQualityMap map_quality_cs_;
  bc::bool_t valid_b_;
};

class ElementPack
{
 public:
  ElementPack() : left_pack_cs_(BoundaryPointPack()), right_pack_cs_(BoundaryPointPack()), valid_b_(bc::false_v) {}
  BoundaryPointPack left_pack_cs_;
  BoundaryPointPack right_pack_cs_;
  bc::bool_t valid_b_;
};

class LanePack
{
 public:
  LanePack() : element_pack_ar_{ElementPack()}, valid_b_(bc::false_v) {}
  bc::TCArray<ElementPack, 2> element_pack_ar_;
  bc::bool_t valid_b_;
};

class BoundaryPack
{
 public:
  BoundaryPack() : lane_pack_ar_{LanePack()} {}
  bc::TCArray<LanePack, 5> lane_pack_ar_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Weight Distribution structure definition */

class WeightDist
{
 public:
  bc::float32_t w_raw_f_;
  bc::float32_t w_scenario_f_;
  bc::float32_t w_quality_f_;
  bc::float32_t w_total_f_;
  bc::float32_t w_norm_f_;
};

class PointWeight
{
 public:
  WeightDist w_per_cs_;
  WeightDist w_map_cs_;
  WeightDist w_ldveh_cs_;
  WeightDist w_ref_cs_;
};

class BoundaryWeight
{
 public:
  bc::TCArray<PointWeight, kMaxNumBoundary> point_weight_ar_;
  PointWeight c0_weight_;
  PointWeight c1_weight_;
  bc::bool_t valid_b_;
};

class ElementWeight
{
 public:
  BoundaryWeight left_weight_ar_;
  BoundaryWeight right_weight_ar_;
  bc::bool_t valid_b_;
};

class LaneWeight
{
 public:
  bc::TCArray<ElementWeight, 2> element_weight_ar_;
  bc::bool_t valid_b_;
};

class WeightPack
{
 public:
  WeightPack() : lane_weight_ar_{} {}
  bc::TCArray<LaneWeight, 5> lane_weight_ar_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Diagnosis management structure definition */

class InputDiagInfo
{
 public:
  void Reset()
  {
    timeout_man_cs_.Reset();
    status_man_cs_.Reset();
  }
  DiagMan timeout_man_cs_;
  DiagMan status_man_cs_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Semantic structure definition */
using ::zone::data::em_data::kMaxBoundarySegment;
using ::zone::data::em_data::kMaxElemNumInOneLane;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kMaxLanePropertySegsInOneLaneElement;
using ::zone::data::em_data::kMaxLaneTransitionSegsInOneLaneElement;
using ::zone::data::em_data::kMaxSpecialSituationSegsInOneLaneElement;
using ::zone::data::em_data::LaneBoundarySegment;
using ::zone::data::em_data::MapCarPosition;
using ::zone::data::em_data::MapGeoFence;
using ::zone::data::em_data::MapGuidePoint;
using ::zone::data::em_data::RefLineSegIsInIntersection;
using ::zone::data::em_data::RefLineSegIsOnRoute;
using ::zone::data::em_data::RefLineSegLaneArrowType;
using ::zone::data::em_data::RefLineSegLaneCurvature;
using ::zone::data::em_data::RefLineSegLaneSlope;
using ::zone::data::em_data::RefLineSegLaneTransitionDir;
using ::zone::data::em_data::RefLineSegLaneType;
using ::zone::data::em_data::RefLineSegSpecialSituation;
using ::zone::data::em_data::RefLineSegSuperElevation;
using ::zone::common::StaticObject;

class ElementSemanticPack
{
 public:
  ElementSemanticPack()
      : lane_type_(0x0),
        route_left_dis_(0.f),
        is_dest_lane_ele_(bc::false_v),
        element_id_(0),
        recommend_level_(0.f),
        relative_idx_(UINT8_MAX),
        merge_flag_(bc::false_v),
        direction_to_dest_(0),
        valid_b_(bc::false_v)
  {
  }
  bc::TCArray<RefLineSegIsOnRoute, kMaxLanePropertySegsInOneLaneElement> is_on_route_segs_;
  bc::TCArray<RefLineSegIsInIntersection, kMaxLanePropertySegsInOneLaneElement> is_in_intersection_segs_;
  bc::TCArray<RefLineSegSpeedLimit, kMaxLanePropertySegsInOneLaneElement> speed_limit_segs_;
  bc::TCArray<RefLineSegLaneType, kMaxLanePropertySegsInOneLaneElement> lane_type_segs_;
  bc::TCArray<RefLineSegLaneSlope, kMaxLanePropertySegsInOneLaneElement> slope_segs_;
  bc::TCArray<RefLineSegLaneTransitionDir, kMaxLaneTransitionSegsInOneLaneElement> lane_transition_dir_segs_;
  bc::TCArray<RefLineSegSuperElevation, kMaxLanePropertySegsInOneLaneElement> super_elevation_segs_;
  bc::TCArray<RefLineSegLaneCurvature, kMaxLanePropertySegsInOneLaneElement> curvature_segs_;
  // bc::TCArray<RefLineSegLaneHeading,
  // kMaxLanePropertySegsInOneLaneElement> heading_segs_;
  // bc::TCArray<RefLineSegLaneWidth,
  // kMaxLanePropertySegsInOneLaneElement> width_segs_;
  bc::TCArray<RefLineSegLaneArrowType, kMaxLanePropertySegsInOneLaneElement> arrow_segs_;
  bc::TCArray<RefLineSegSpecialSituation, kMaxSpecialSituationSegsInOneLaneElement> special_segs_;
  // bc::TCArray<AgentCurrentPosProjOnToRefLine,
  // kAgentsProjectedToOneRefLineMaxNum>
  // agents_projected_traj_;
  bc::uint32_t lane_type_;
  bc::float32_t route_left_dis_;
  bc::bool_t is_dest_lane_ele_;
  bc::uint16_t element_id_;
  bc::float32_t recommend_level_;
  bc::uint8_t relative_idx_;
  bc::bool_t merge_flag_;
  bc::int8_t direction_to_dest_;
  bc::TCArray<LaneBoundarySegment, kMaxBoundarySegment> left_segs_;
  bc::TCArray<LaneBoundarySegment, kMaxBoundarySegment> right_segs_;
  bc::bool_t valid_b_;
};
class LaneSemanticPack
{
 public:
  LaneSemanticPack() : main_lane_idx_u8_(UINT8_MAX), global_lane_idx_u32_(UINT32_MAX), valid_b_(bc::false_v) {}
  bc::TCArray<ElementSemanticPack, kMaxElemNumInOneLane> lane_semantic_ar_;
  bc::uint8_t main_lane_idx_u8_;
  bc::uint32_t global_lane_idx_u32_;
  bc::bool_t valid_b_;
};
class SemanticPack
{
 public:
  SemanticPack()
      : relat_dest_lane_(0),
        timestamp_(0.f),
        tja_target_id_(kInvalidObjectId),
        global_lane_num_(0),
        host_lane_idx_(-1),
        main_lane_num_(0),
        map_lane_marking_ar_{EmLaneMarking()},
        map_cross_stop_line_ar_{EmCrossStopLine()},
        map_traffic_light_ar_{EmTrafficLight()},
        map_intersection_data_(EmIntersectionData())
  {
  }
  bc::TCArray<StaticObject, HORIZON_OBJECT_MAX_NUM> traffic_spdlmt_sign_ar_;
  bc::TCArray<bc::uint8_t, HORIZON_OBJECT_MAX_NUM> traffic_spdlmt_sign_idx_ar_;
  bc::uint8_t traffic_spdlmt_sign_num_u8_ = 0U;
  bc::TCArray<StaticObject, HORIZON_OBJECT_MAX_NUM> traffic_spdlmt_rev_sign_ar_;
  bc::TCArray<bc::uint8_t, HORIZON_OBJECT_MAX_NUM> traffic_spdlmt_rev_sign_idx_ar_;
  bc::uint8_t traffic_spdlmt_rev_sign_num_u8_ = 0U;  // TSR
  bc::TCArray<LaneSemanticPack, kMaxLaneNum> map_semantic_ar_;
  bc::TCArray<MapGuidePoint, kMaxMapGuidePtNum> map_guide_pts_;
  MapGeoFence map_geofence_;
  MapCarPosition map_car_position_;
  bc::int8_t relat_dest_lane_;
  bc::float64_t timestamp_;
  bc::uint16_t tja_target_id_;
  bc::uint32_t global_lane_num_;
  bc::int32_t host_lane_idx_;
  bc::uint8_t main_lane_num_;

  bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum> map_lane_marking_ar_;
  bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum> map_cross_stop_line_ar_;
  bc::TCArray<EmTrafficLight, kMaxTrafficLightNum> map_traffic_light_ar_;
  EmIntersectionData map_intersection_data_;
  // Navi speed limit
  /** normal speed limit */
  bc::bool_t max_normal_spdlmt_valid_b_ = false;
  bc::uint8_t max_vel_normal_speed_limit_u8_ = 0U;
  bc::uint16_t max_dx_normal_speed_limit_u16_ = 0U;
  bc::bool_t min_normal_spdlmt_valid_b_ = false;
  bc::uint8_t min_vel_normal_speed_limit_u8_ = 0U;
  bc::uint16_t min_dx_normal_speed_limit_u16_ = 0U;
  /** electronic eye speed limit */
  bc::bool_t ele_eye_spdlmt_valid_b_ = false;
  bc::uint8_t vel_ele_eye_speed_limit_u8_ = 0U;
  bc::uint32_t dx_ele_eye_speed_limit_u32_ = 0U;
  /** section speed limit */
  bc::bool_t intervel_spdlmt_valid_b_ = false;
  bc::uint16_t dx_interval_starting_point_u16_ = 0U;
  bc::uint16_t dx_interval_ending_point_u16_ = 0U;
  bc::uint8_t vel_interval_speed_limit_u8_ = 0U;
  bc::float32_t c0 = 0.0f;
  bc::float32_t c1 = 0.0f;
  bc::float32_t c2 = 0.0f;
  bc::float32_t c3 = 0.0f;
  RefLineSegLaneTransitionDir nearst_split_scenario_;
};

class ElementStEndIdxPack
{
 public:
  ElementStEndIdxPack()
      : boundary_st_idx_(0), boundary_end_idx_(0), boundary_st_x_f_(0.f), boundary_end_x_f_(0.f), datum_idx_(0)
  {
  }
  bc::uint8_t boundary_st_idx_;
  bc::uint8_t boundary_end_idx_;
  bc::float32_t boundary_st_x_f_;
  bc::float32_t boundary_end_x_f_;
  bc::uint8_t datum_idx_;
};
class LaneStEndIdxPack
{
 public:
  bc::TCArray<ElementStEndIdxPack, 2> ele_st_end_idx_;
};
class StEndIdxPack
{
 public:
  bc::TCArray<LaneStEndIdxPack, 5> lane_st_end_idx_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------*/
/** Olr structure definition */
class LaneElementMapping
{
 public:
  LaneElementMapping()
      : valid_(bc::false_v),
        element_id_change_b_(bc::false_v),
        is_old_element_b_(bc::false_v),
        lst_lane_index_u8_(kMaxLaneNum),
        lst_element_index_u8_(kMaxElemNumInOneLane)
  {
  }
  bc::bool_t valid_;
  bc::bool_t element_id_change_b_;
  bc::bool_t is_old_element_b_;
  bc::uint8_t lst_lane_index_u8_;
  bc::uint8_t lst_element_index_u8_;
};
/*-------------------------------------------------------------------------------------------------------------------------*/
}  // namespace environment_model
}  // namespace zone
#endif
