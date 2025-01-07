#pragma once

#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
#include "ehradapterhelper.h"
#include "environment_model_internal.h"
#include "math/bitwise_operation.h"

using ::zone::common::EhrToEmData;
using zone::data::em_data::kMaxElemNumInOneLane;
using zone::data::em_data::kMaxLanePropertySegsInOneLaneElement;
using zone::data::em_data::kMaxLaneTransitionSegsInOneLaneElement;
using ::zone::data::em_data::LaneBoundary;
using ::zone::data::em_data::LaneElement;
using ::zone::data::em_data::LaneTransitionDirection;
using ::zone::data::em_data::RoadEdge;
using ::zone::helper::math::GetBitU16;
using ::zone::helper::math::SetBitU16;

static const bc::int32_t kValidMapPointOffset = -5000;    // 50m
static const bc::int32_t kValidMapSegmentOffset = -4000;  // 40m
static const bc::float32_t kMinDistanceFromEgo = 200.f;
static const bc::float32_t kLongDistanceBehindEgo = -10.f;
static const bc::float32_t kLongDistanceAheadEgo = 10.f;
static const bc::float32_t kPointMinDistance = 4.f;
static const bc::float32_t kEdgeMinDistance = 2.f;

class EhrDataHandle
{
  void operator=(const EhrDataHandle&) = delete;
  EhrDataHandle(const EhrDataHandle&) = delete;

 private:
  const EhrToEmData* ehr_to_emdata_ptr_;

 public:
  EhrDataHandle() = default;

  void setEhrToEmData(const ::zone::common::EhrToEmData& ehr_to_emdata);

  void getEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data);
  void getLeftMergeEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data);
  void getRightMergeEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data);

 private:
  void getLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range,
                       bc::TCArray<LaneElement, kMaxElemNumInOneLane>& em_lane_elements,
                       zone::data::em_data::RefLineSegLaneTransitionDir& split_scenario, int8_t lane_flag = 0);

  void getLaneBoundarys(const zone::common::EhrToEmLaneBoundary& lane_boundary, LaneBoundary& em_lane_boundary,
                        const bc::int8_t boundary_flag, bc::float32_t terminated_offset);

  void getRoadEdges(const zone::common::EhrToEmRoadEdge& lane_roadedge, RoadEdge& em_lane_roadedge);

  void getSurfaces(zone::data::em_data::EmAdapterHDmapData& map_em_data);

  void getLaneMarkings(zone::data::em_data::EmAdapterHDmapData& map_em_data);

  void getCenterLines(const zone::common::EhrToEmLaneElement& lane_element, LaneElement& em_lane_element,
                      bc::float32_t terminated_offset);

  void getLaneBoundarysBySegment(const zone::common::EhrToEmLaneElement& lane_element, LaneElement& em_lane_element,
                                 bc::float32_t terminated_offset);

  // void getRoadEdgesBySegment(const zone::common::EhrToEmLaneElement& lane_element, LaneElement& em_lane_element);

  bc::bool_t getDestLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range);

  bc::bool_t getDestLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range, bc::int32_t& dest_idx);

  bc::bool_t findRelatedOnRouteElement(const zone::common::EhrToEmLaneElement& on_route_element,
                                       const zone::common::EhrToEmLaneElement& continue_element,
                                       zone::data::em_data::RefLineSegLaneTransitionDir& split_scenario,
                                       LaneTransitionDirection& target_lane_transition);

  bc::uint16_t last_cycle_on_route_id = 0;
};