#ifndef EM_INTERSECTION_RECOGNITION_H
#define EM_INTERSECTION_RECOGNITION_H

#include "../static_obj_process/sop.h"
#include "common/data/em_data.h"
#include "common/data/horizon_static_object.h"
#include "environment_model_internal.h"
#include "math/bitwise_operation.h"
#include "math/linear_interpolation.h"
namespace zone {
namespace environment_model {
using ::zone::common::Point3D;
using ::zone::data::em_data::EmData;
using ::zone::data::em_data::EmCrossStopLine;
using ::zone::helper::math::DiagStatus;

using ::zone::data::em_data::kMaxCrossStopLineNum;
using ::zone::data::em_data::kMaxTrafficLightNum;
using ::zone::data::em_data::EmCrossStopLineType;

static const bc::uint8_t kIdxBoundPtLowerLeft = 0;
static const bc::uint8_t kIdxBoundPtLowerRight = 1;
static const bc::uint8_t kIdxBoundPtUpperRight = 2;
static const bc::uint8_t kIdxBoundPtUpperLeft = 3;

static const bc::uint8_t kIdxBoundEntry = 0;
static const bc::uint8_t kIdxBoundStraight = 1;
static const bc::uint8_t kIdxBoundLeft = 2;
static const bc::uint8_t kIdxBoundRight = 3;

enum class IntersectionManagerState : bc::uint8_t
{
  kNoIts = 0,
  kBuildingIts = 1,
  kPassingIts = 2,
  kExitingIts = 3,
};

class IntersectionRefPoint
{
 public:
  IntersectionRefPoint() : ref_point_cs_(Point3D()), valid_b_(bc::false_v) {}
  Point3D ref_point_cs_;
  bc::bool_t valid_b_;
};
class IntersectionManager
{
 public:
  void reset() { state_en_ = IntersectionManagerState::kNoIts; }
  IntersectionManagerState state_en_ = IntersectionManagerState::kNoIts;
};

class IntersectionBoundPoint
{
 public:
  IntersectionBoundPoint() : point_cs_(Point3D()), life_time_f_(0.0), valid_b_(bc::false_v) {}

  Point3D point_cs_;
  bc::float32_t life_time_f_;
  bc::bool_t valid_b_;
};

class IntersectionBound
{
 public:
  IntersectionBound() : center_point_cs_(Point3D()), length_f_(0.0), heading_norm_f_(0.0), valid_b_(bc::false_v) {}

  Point3D center_point_cs_;
  bc::float32_t length_f_;
  bc::float32_t heading_norm_f_;
  bc::bool_t valid_b_;
};

class IntersectionRecognition
{
  IntersectionRecognition(const IntersectionRecognition&) = delete;
  void operator=(const IntersectionRecognition&) = delete;

 public:
  IntersectionRecognition(EmData& em_collection);
  ~IntersectionRecognition() = default;

  void Run(const InputDiagInfo& static_obj_diag_cs, const EgoPoseCollection& ego_motion_cs,
           const StaticObjProcess& static_obj_process_cs);

  void SetInput(const StaticObjProcess& static_obj_process_cs, const EgoPoseCollection& ego_motion_cs);

  void TrackingIntersectionBound();

  void UpdateIntersectionManager(const StaticObjProcess& static_obj_process_cs);
  void BuildingIntersection();
  void ExploringIntersection();
  void CrosslineBoundMatching(const CrossStopLineInfo* iter_ptr, IntersectionBound& its_bound_cs);
  void UpdateBoundByBoundPoints(const bc::uint8_t bound_type_u8);
  void UpdateBoundPointsByBound(const bc::uint8_t point_type_u8);
  void UpdateEntryExitPoints(const bc::uint8_t point_type_u8);
  void SetTimeCycle(const bc::float32_t time_cycle_f) { time_cycle_f_ = time_cycle_f; }

  void SetOutput();

  void ResetIntersection();

  /** define in public for debug purpose, should move to private later.*/
  IntersectionManager its_manager_cs_;
  bc::TCArray<IntersectionBoundPoint, 4> its_bound_pts_ar_;
  bc::TCArray<IntersectionBoundPoint, 4> its_bound_pts_lst1_ar_;
  bc::TCArray<IntersectionRefPoint, 4> its_entry_exit_pts_ar_;
  // IntersectionRefPoint its_entry_cs_;
  // IntersectionRefPoint its_exit_straight_cs_;
  // IntersectionRefPoint its_exit_left_cs_;
  // IntersectionRefPoint its_exit_right_cs_;
  bc::bool_t its_build_success_b_ = false;

 private:
  bc::bool_t static CrossStopLineDxCompare(const CrossStopLineInfo& a, const CrossStopLineInfo& b)
  {
    return (a.dx_f_ <= b.dx_f_);
  };

  void HeadingRangeNormalize(bc::float32_t& heading_f);

  EmData* em_collection_ptr_;
  bc::TCArray<IntersectionBound, 4> its_bound_ar_;
  bc::TCArray<IntersectionBound, 4> its_bound_lst1_ar_;

  bc::TFixedVector<CrossStopLineInfo, kMaxStaticObjNum> cross_stop_line_vt_;
  bc::TFixedVector<TrafficLightInfo, kMaxStaticObjNum> traffic_light_vt_;
  TransForm transform_st;
  bc::float32_t time_cycle_f_;
  bc::float32_t life_time_f_ = 0.0f;
  bc::float32_t v_ego_f_ = 0.0f;
};

}  // namespace environment_model
}  // namespace zone
#endif
