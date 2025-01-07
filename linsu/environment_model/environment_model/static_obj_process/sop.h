#ifndef EM_STATIC_OBJ_PROCESS_H
#define EM_STATIC_OBJ_PROCESS_H

#include "common/data/em_data.h"
#include "common/data/horizon_static_object.h"
#include "environment_model_internal.h"
#include "math/bitwise_operation.h"
#include "math/linear_interpolation.h"
#include "math/multi_dimention_interpolation.h"
#include "convex_hull.h"
#include<list>
namespace zone {
namespace environment_model {

using ::zone::common::StaticObjectData;
using ::zone::common::StaticObject;
using ::zone::common::StPoint;
using ::zone::data::em_data::EmData;
using ::zone::data::em_data::EmStaticObject;
using ::zone::data::em_data::EmLaneMarking;
using ::zone::data::em_data::EmLaneMarkingTypeMsk;
using ::zone::data::em_data::EmCrossStopLine;
using ::zone::data::em_data::EmCrossStopLineType;
using ::zone::data::em_data::EmTrafficLight;
using ::zone::data::em_data::EmTrafficLightTypeMsk;
using ::zone::data::em_data::EmTrafficLightColor;
using ::zone::data::em_data::EmTrafficLightMode;
using ::zone::data::em_data::EmConeCollection;
using ::zone::data::em_data::EmCone;
using ::zone::data::em_data::EmConeCluster;
using ::zone::data::em_data::LaneBoundary;
using ::zone::data::em_data::ReferenceLine;
using ::zone::data::em_data::kMaxStaticObjNum;
using ::zone::data::em_data::kMaxLaneMarkingNum;
using ::zone::data::em_data::kMaxCrossStopLineNum;
using ::zone::data::em_data::kMaxTrafficLightNum;
using ::zone::data::em_data::kInvalidStaticObjId;
using ::zone::data::em_data::kInvalidStaticObjIdx;
using ::zone::data::em_data::kMaxLaneNum;
using ::zone::data::em_data::kUnknownLane;
using ::zone::data::em_data::kMaxRefLinePtsNum;
using ::zone::data::em_data::kMaxBoundaryPoint;
using ::zone::helper::math::DiagStatus;
using ::zone::helper::math::Lin_Interp_Method;
using ::zone::helper::math::SetBitU8;
using ::zone::helper::math::SetBitU16;
using ::zone::helper::math::SetBitU32;
using ::zone::helper::math::GetBitU8;
using ::zone::helper::math::GetBitU16;
using ::zone::helper::math::GetBitU32;
using ::zone::common::CurvParam;
using ::zone::helper::math::InterpCurv;
using ::bc::G_PI;
using ::bc::G_PI_2;

static const bc::uint8_t kMaxClusterNumLaneMarking = 10;
static const bc::uint8_t kMaxClusterNumCrossStopLine = 10;
static const bc::uint8_t kMaxClusterNumTrafficLight = 3;
static const bc::uint8_t kMaxClusterNumCone = 5;
static const bc::uint8_t kMaxNumLaneMarkingPack = 15;
static const bc::uint8_t kMaxNumCrossStopLinePack = 15;
static const bc::uint8_t kMaxNumTrafficLightPack = 10;
static const bc::uint8_t kMaxNumConePack = kMaxStaticObjNum;

static const bc::uint8_t kTrafficLightSemanticStraight = 0;
static const bc::uint8_t kTrafficLightSemanticLeft = 1;
static const bc::uint8_t kTrafficLightSemanticRight = 2;
static const bc::uint8_t kTrafficLightSemanticUturn = 3;
static const bc::uint8_t kTrafficLightSemanticPed = 4;
static const bc::uint8_t kTrafficLightSemanticCyc = 5;
static const bc::uint8_t kNumTrafficLightSemantic = 6;
static const bc::uint8_t kTrafficLightSemanticUnknown = 7;

static const bc::uint8_t kConeType = 11;
static const bc::uint8_t kConeInvalidtype = 0;
static const bc::uint8_t kConeInvalidSubtype = 0;
static const bc::float32_t kDropoutDistance = 0.f;
static const bc::float32_t kConeToClusterMaxDis = 7.f;
static const bc::float32_t kConeFilterDis = 0.35f;

enum ClusterTypeMask : bc::uint8_t
{
  kStopLineMask = 0,
  kCrossLineMask = 1,
};
enum class BulbTypeMask : bc::uint8_t
{
  kCircleMask = 0,
  kLeftArrowMask = 1,
  kRightArrowMask = 2,
  kUpArrowMask = 3,
  kDownArrowMask = 4,
  kUturnMask = 5,
  kForwardAndLeftMask = 6,
  kForwardAndRightMask = 7,
  kPedestrainMask = 8,
  kNonMotorMask = 9,
  kTimeMask = 10,
  kLeftAndUturnMask = 11,
  kNoDriveIntoMask = 12,
  kTextOfAllowPedMask = 13,
  kSignOfAllowPedMask = 14,
  kTestOfForbidPedMask = 15,
  kSignOfForbidPedMask = 16,
  kNumBulbTypeMask = 17,
  kUnknown = 255,
};

enum class TrafficLightStatus : bc::uint8_t
{
  kUnknown = 0,
  kGreen = 1,
  kYellow = 2,
  kRed = 3,
};

enum class StaticObjSource : bc::uint8_t
{
  kUnknown = 0,
  kPerception = 1,
  kMap = 2,
};

class LaneMarkingInfo
{
 public:
  LaneMarkingInfo()
      : idx_u8_(kInvalidStaticObjIdx),
        id_s32_(kInvalidStaticObjId),
        dx_f_(0.0),
        dy_f_(0.0),
        heading_f_(0.0),
        life_time_f_(0.0),
        type_u16_(0),
        lane_assigned_u8_(kUnknownLane),
        source_en_(StaticObjSource::kUnknown),
        updated_b_(bc::false_v)
  {
  }
  bc::uint8_t idx_u8_;
  bc::int32_t id_s32_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t heading_f_;
  bc::float32_t life_time_f_;
  bc::uint16_t type_u16_;
  bc::uint8_t lane_assigned_u8_;
  StaticObjSource source_en_;
  bc::bool_t updated_b_;
};

class LaneMarkingSubCluster
{
 public:
  LaneMarkingSubCluster() : lane_assigned_u8_(kUnknownLane), dy_mean_f_(0.0), heading_f_(0.0)
  {
    lane_marking_pack_vt_.clear();
  }
  bc::TFixedVector<LaneMarkingInfo, kMaxNumLaneMarkingPack> lane_marking_pack_vt_;
  bc::uint8_t lane_assigned_u8_;
  bc::float32_t dy_mean_f_;
  bc::float32_t heading_f_;
};

class LaneMarkingCluster
{
 public:
  LaneMarkingCluster()
      : dx_mean_f_(0.0), dx_tracking_f_(0.0), life_time_f_(0.0), num_lane_marking_(0), new_create_b_(bc::false_v)
  {
    lane_marking_subcluster_vt_.clear();
  }
  bc::TFixedVector<LaneMarkingSubCluster, kMaxLaneNum> lane_marking_subcluster_vt_;  // one subcluster for each lane
  bc::float32_t dx_mean_f_;
  bc::float32_t dx_tracking_f_;
  bc::float32_t life_time_f_;
  bc::uint8_t num_lane_marking_;
  bc::bool_t new_create_b_;
};

class CrossStopLineInfo
{
 public:
  CrossStopLineInfo()
      : idx_u8_(kInvalidStaticObjIdx),
        id_s32_(kInvalidStaticObjId),
        dx_f_(0.0),
        dy_f_(0.0),
        length_side_f_(0.0),
        width_side_f_(0.0),
        heading_f_(0.0),
        life_time_f_(0.0),
        prob_road_cover_f_(0.0),
        type_en_(EmCrossStopLineType::kUnknown),
        source_en_(StaticObjSource::kUnknown),
        updated_b_(bc::false_v),
        distilled_b_(bc::false_v)
  {
  }
  bc::uint8_t idx_u8_;
  bc::int32_t id_s32_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t length_side_f_;
  bc::float32_t width_side_f_;
  bc::float32_t heading_f_;
  bc::float32_t life_time_f_;
  bc::float32_t prob_road_cover_f_;
  EmCrossStopLineType type_en_;
  StaticObjSource source_en_;
  bc::bool_t updated_b_;
  bc::bool_t distilled_b_;
};

class CrossStopLineCluster
{
 public:
  CrossStopLineCluster()
      : cross_stop_line_pack_vt_(),
        dx_mean_f_(0.0),
        dx_tracking_f_(0.0),
        dy_mean_f_(0.0),
        length_side_mean_f_(0.0),
        heading_mean_f_(0.0),
        life_time_f_(0.0),
        cluster_type_en_(EmCrossStopLineType::kUnknown),
        num_cross_stop_line_(0),
        new_create_b_(bc::false_v)
  {
  }

  void UpdateClusterType(EmCrossStopLineType input_en)
  {
    switch (cluster_type_en_)
    {
      case EmCrossStopLineType::kUnknown:
      default:
        cluster_type_en_ = input_en;
        break;
      case EmCrossStopLineType::kCrossLine:
        if (input_en == EmCrossStopLineType::kStopLine)
        {
          cluster_type_en_ = EmCrossStopLineType::kCrossAndStopLine;
        }
        break;
      case EmCrossStopLineType::kStopLine:
        if (input_en == EmCrossStopLineType::kCrossLine)
        {
          cluster_type_en_ = EmCrossStopLineType::kCrossAndStopLine;
        }
        break;
      case EmCrossStopLineType::kCrossAndStopLine:
        /** do nothing */
        break;
    }
  }

  bc::TFixedVector<CrossStopLineInfo, kMaxNumCrossStopLinePack> cross_stop_line_pack_vt_;
  bc::float32_t dx_mean_f_;
  bc::float32_t dx_tracking_f_;
  bc::float32_t dy_mean_f_;
  bc::float32_t length_side_mean_f_;
  bc::float32_t heading_mean_f_;
  bc::float32_t life_time_f_;
  EmCrossStopLineType cluster_type_en_;
  bc::uint8_t num_cross_stop_line_;
  bc::bool_t new_create_b_;
};

class TrafficLightInfo
{
 public:
  TrafficLightInfo()
      : type_status_pair_vt_(),
        idx_u8_(kInvalidStaticObjIdx),
        id_s32_(kInvalidStaticObjId),
        dx_f_(0.0),
        dy_f_(0.0),
        life_time_f_(0.0),
        // type_mask_en_(BulbTypeMask::kUnknown),
        // status_s32_(0),
        in_lanes_b_(bc::false_v),
        updated_b_(bc::false_v),
        distilled_b_(bc::false_v)
  {
  }
  bc::TFixedVector<bc::TPair<bc::uint16_t, EmTrafficLightColor>, 20>
      type_status_pair_vt_;  // Pair<TrafficLightType, TrafficLightColor>
  bc::uint8_t idx_u8_;
  bc::int32_t id_s32_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t life_time_f_;
  // BulbTypeMask type_mask_en_;
  // bc::int32_t status_s32_;
  bc::bool_t in_lanes_b_;
  bc::bool_t updated_b_;
  bc::bool_t distilled_b_;
};

class TrafficLightCluster
{
 public:
  TrafficLightCluster()
      : traffic_light_pack_vt_(),
        dx_mean_f_(0.0),
        dx_tracking_f_(0.0),
        dy_mean_f_(0.0),
        life_time_f_(0.0),
        num_traffic_light_(0),
        new_create_b_(bc::false_v)
  {
  }
  bc::TFixedVector<TrafficLightInfo, kMaxNumTrafficLightPack> traffic_light_pack_vt_;
  bc::float32_t dx_mean_f_;
  bc::float32_t dx_tracking_f_;
  bc::float32_t dy_mean_f_;
  bc::float32_t life_time_f_;
  bc::uint8_t num_traffic_light_;
  bc::bool_t new_create_b_;
};

class ConeInfo
{
 public:
  ConeInfo()
      : idx_u8_(kInvalidStaticObjIdx),
        assigned_cluster_idx_(kMaxClusterNumCone),
        id_s32_(kInvalidStaticObjId),
        type_u16_(kConeInvalidtype),
        dx_f_(0.f),
        dy_f_(0.f),
        life_time_f_(0.f),
        updated_b_(bc::false_v)
  {
  }
  bc::uint8_t idx_u8_;
  bc::uint8_t assigned_cluster_idx_;
  bc::int32_t id_s32_;
  bc::uint16_t type_u16_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t life_time_f_;
  bc::bool_t updated_b_;
};

class ConeCluster
{
 public:
  ConeCluster()
      : cone_pack_vt_({}),
        dx_f_(0.f),
        dy_f_(0.f),
        width_f_(0.f),
        length_f_(0.f),
        heading_f_(0.f),
        valid_b_(bc::false_v)
  {
  }

  bc::TFixedVector<ConeInfo, kMaxNumConePack> cone_pack_vt_;
  bc::float32_t dx_f_;
  bc::float32_t dy_f_;
  bc::float32_t width_f_;
  bc::float32_t length_f_;
  bc::float32_t heading_f_;
  bc::bool_t valid_b_;
};
class StaticObjProcess
{
  StaticObjProcess(const StaticObjProcess&) = delete;
  void operator=(const StaticObjProcess&) = delete;

 public:
  StaticObjProcess(EmData& em_collection);
  ~StaticObjProcess() = default;

  void Run(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
           const StaticObjectData& per_static_obj_cs, const SemanticPack& semantic_pack_cs);
  /* matching input static objs to EM static objs according to id.
  The static obj with same id should be stored in same index. */
  void StaticObjMatching(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs);

  void SetInput(const StaticObject& obj_in_cs, const bc::uint8_t idx_u8, EmStaticObject& obj_out_cs);
  void SetTimeCycle(const bc::float32_t time_cycle_f) { time_cycle_f_ = time_cycle_f; }
  void SetLaneMarkingInputPer(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b, LaneMarkingInfo& obj_out_cs);
  void SetLaneMarkingInputMap(const EmLaneMarking& obj_in_cs, const bc::bool_t new_obj_b, LaneMarkingInfo& obj_out_cs);

  void StaticObjLaneAssignment();
  void LaneMarkingLaneAssignment();

  /** lane marking */
  void LaneMarkingStablization(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                               const StaticObjectData& per_static_obj_cs,
                               const bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum>& map_lane_marking_ar);
  void TrackingLaneMarkingArrayAndCluster(const EgoPoseCollection& ego_motion_cs);
  void UpdateLaneMarkingArray(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs,
                              const bc::TCArray<EmLaneMarking, kMaxLaneMarkingNum>& map_lane_marking_ar);
  void LaneMarkingArray2Vector();
  void LaneMarkingClustering();
  void CreateLaneMarkingCluster(const LaneMarkingInfo& lane_marking_cs);
  void InsertLaneMarkingToCluster(const LaneMarkingInfo& lane_marking_cs, LaneMarkingCluster& lane_marking_cluster_cs);
  void UpdateLaneMarkingClusterList();
  void LaneMarkingDistillation();
  void LaneMarkingOutput();
  bc::uint16_t LaneMarkingTypeMapping(const bc::int32_t type_horizon_s32);
  bc::int32_t LaneMarkingTypeMappingEm2Horizon(const bc::uint16_t type_em_u16);

  /** cross/stop line */
  void CrossStopLineStablization(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                                 const StaticObjectData& per_static_obj_cs,
                                 const bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum>(&map_cross_stop_line_ar));
  void TrackingCrossStopLineArrayAndCluster(const EgoPoseCollection& ego_motion_cs);
  void UpdateCrossStopLineArray(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs,
                                const bc::TCArray<EmCrossStopLine, kMaxCrossStopLineNum>(&map_cross_stop_line_ar));
  void SetCrossStopLineInputPer(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b,
                                CrossStopLineInfo& obj_out_cs);
  void SetCrossStopLineInputMap(const EmCrossStopLine& obj_in_cs, const bc::bool_t new_obj_b,
                                CrossStopLineInfo& obj_out_cs);
  void CalcLengthWidthHeading(const StaticObject& obj_cs, bc::float32_t& length_f, bc::float32_t& width_f,
                              bc::float32_t& heading_f);
  void CrossStopLineRoadAssignment();
  void CrossStopLineArray2Vector();
  void CrossStopLineClustering();
  void CreateCrossStopLineCluster(const CrossStopLineInfo& cross_stop_line_cs);
  void InsertCrossStopLineToCluster(const CrossStopLineInfo& cross_stop_line_cs,
                                    CrossStopLineCluster& cross_stop_line_cluster_cs);
  void UpdateCrossStopLineClusterList();
  void CrossStopLineDistillation();
  void CrossStopLineOutput();
  EmCrossStopLineType CrossStopLineTypeMapping(const bc::int32_t type_horizon_s32);

  /** traffic light */
  void TrafficLightStablization(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                                const StaticObjectData& per_static_obj_cs,
                                const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar));
  void TrackingTrafficLightArrayAndCluster(const EgoPoseCollection& ego_motion_cs);
  void UpdateTrafficLightArray(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs);
  void SetTrafficLightInput(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b, TrafficLightInfo& obj_out_cs);
  void TrafficLightRoadAssignment(const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar));
  bc::bool_t MatchPerTrafficLightWithMap(const TrafficLightInfo per_traffic_light_cs,
                                         const bc::TCArray<EmTrafficLight, kMaxTrafficLightNum>(&map_traffic_light_ar));
  void TrafficLightArray2Vector();
  void TrafficLightClustering();
  void CreateTrafficLightCluster(const TrafficLightInfo& traffic_light_cs);
  void InsertTrafficLightToCluster(const TrafficLightInfo& traffic_light_cs,
                                   TrafficLightCluster& traffic_light_cluster_cs);
  void UpdateTrafficLightClusterList();
  void TrafficLightDistillation();
  void TrafficLightSemanticUpdate(const bc::uint8_t traffic_light_semantic, const bc::int32_t id_input_s32,
                                  const EmTrafficLightColor color_input_en);

  void TrafficLightOutput();
  bc::TPair<bc::uint16_t, EmTrafficLightColor> TrafficLightTypeMapping(const bc::int32_t type_horizon_s32,
                                                                       const bc::int32_t color_horizon_s32);

  /** Cone */
  void ConeClusterAndConvexHullGeneration(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
                                const StaticObjectData& per_static_obj_cs);
  void TrackingConeArrayAndCluster(const EgoPoseCollection& ego_motion_cs);
  void FilterCone(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs);
  void UpdateConeMarkingArray(const InputDiagInfo& static_obj_diag_cs, const StaticObjectData& per_static_obj_cs);
  void SetConeInput(const StaticObject& obj_in_cs, const bc::bool_t new_obj_b, ConeInfo& obj_out_cs);
  void ConeClustering();
  void UpdateCluster(const EgoPoseCollection& ego_motion_cs);
  void GenerateConvexHullForCluster(ConeCluster& cluster_info, const EgoPose& ego_pose);
  void ConeOutput();
  void SetOutput();

  void ResetStaticObj();
  bc::TFixedVector<CrossStopLineInfo, kMaxStaticObjNum> cross_stop_line_output_vt_;
  bc::TFixedVector<TrafficLightInfo, kMaxStaticObjNum> traffic_light_output_vt_;

  bc::float64_t Distance(const StPoint p1, const StPoint p2);

 private:
  bc::bool_t FindNearestRefIdx(const ReferenceLine& ref_line_cs, const bc::float32_t x_f,
                               bc::uint16_t& nearest_idx_u16);
  bc::bool_t static LaneMarkingDxCompare(const LaneMarkingInfo& a, const LaneMarkingInfo& b)
  {
    return (a.dx_f_ < b.dx_f_);
  };
  bc::bool_t static LaneMarkingLifeTimeCompare(const LaneMarkingInfo& a, const LaneMarkingInfo& b)
  {
    return (a.life_time_f_ > b.life_time_f_);
  };
  bc::bool_t static CrossStopLineDxCompare(const CrossStopLineInfo& a, const CrossStopLineInfo& b)
  {
    return (a.dx_f_ < b.dx_f_);
  };
  bc::bool_t static CrossStopLineLifeTimeCompare(const CrossStopLineInfo& a, const CrossStopLineInfo& b)
  {
    return (a.life_time_f_ > b.life_time_f_);
  };

  bc::bool_t static TrafficLightDxCompare(const TrafficLightInfo& a, const TrafficLightInfo& b)
  {
    return (a.dx_f_ < b.dx_f_);
  };
  bc::bool_t static TrafficLightClusterDxCompare(const TrafficLightCluster& a, const TrafficLightCluster& b)
  {
    return (a.dx_tracking_f_ < b.dx_tracking_f_);
  };
  bc::bool_t static TrafficLightLifeTimeCompare(const TrafficLightInfo& a, const TrafficLightInfo& b)
  {
    return (a.life_time_f_ > b.life_time_f_);
  };

  void HeadingRangeNormalize(bc::float32_t& heading_f);
  bc::TCArray<EmStaticObject, kMaxStaticObjNum> static_obj_ar_;
  bc::TCArray<EmStaticObject, kMaxStaticObjNum> static_obj_lst1_ar_;
  bc::TCArray<bc::uint8_t, kMaxStaticObjNum> static_obj_lane_assign_ar_;
  /** lane marking array/vector/cluster */
  bc::TCArray<LaneMarkingInfo, kMaxStaticObjNum> lane_marking_ar_;
  bc::TFixedVector<LaneMarkingInfo, kMaxStaticObjNum> lane_marking_vt_;
  bc::TFixedVector<LaneMarkingCluster, kMaxClusterNumLaneMarking> lane_marking_cluster_vt_;
  /** cross/stop line array/vector/cluster*/
  bc::TCArray<CrossStopLineInfo, kMaxStaticObjNum> cross_stop_line_ar_;
  bc::TFixedVector<CrossStopLineInfo, kMaxStaticObjNum> cross_stop_line_vt_;
  bc::TFixedVector<CrossStopLineCluster, kMaxClusterNumCrossStopLine> cross_stop_line_cluster_vt_;

  /** traffic light array/vector/cluster*/
  bc::TCArray<TrafficLightInfo, kMaxStaticObjNum> traffic_light_ar_;
  bc::TCArray<BulbTypeMask, static_cast<bc::uint8_t>(BulbTypeMask::kNumBulbTypeMask)> bulb_type_mask_ar_;

  /** cone array/vector/cluster*/
  bc::TCArray<ConeInfo, kMaxStaticObjNum> cone_ar_;
  bc::TFixedVector<ConeCluster, kMaxClusterNumCone> cone_cluster_vt_;
  StaticObjectData cone_fliter_cs_;

  bc::TFixedVector<TrafficLightInfo, kMaxStaticObjNum> traffic_light_vt_;
  bc::TFixedVector<TrafficLightCluster, kMaxClusterNumTrafficLight> traffic_light_cluster_vt_;
  bc::TCArray<bc::TPair<bc::int32_t, EmTrafficLightColor>, kNumTrafficLightSemantic> traffic_light_semantic_type_ar_;
  bc::TCArray<bc::float32_t, kNumTrafficLightSemantic> traffic_light_dy_ar_;
  std::map<BulbTypeMask, bc::uint8_t> traffic_light_type_map_;
  EmData* em_collection_ptr_;
  LaneBoundary left_bound_cs_;
  LaneBoundary right_bound_cs_;

  bc::int64_t timestamp_s64_;
  bc::float32_t time_cycle_f_;
  bc::float32_t v_ego_f_;
  bc::float32_t d_after_intersection_f_;
  bc::bool_t traffic_light_enable_b_;
  bc::bool_t is_in_map_b_;
};

}  // namespace environment_model
}  // namespace zone
#endif