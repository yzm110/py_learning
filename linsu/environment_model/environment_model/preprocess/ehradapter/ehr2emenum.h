#pragma once
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
#include "ehradapterhelper.h"

using namespace zone;
#define EHR2EM_LFET_LANEBOUNDARY (1)
#define EHR2EM_RIGTH_LANEBOUNDARY (-1)

class Ehr2EmEnum
{
 private:
 public:
  Ehr2EmEnum(/* args */);
  ~Ehr2EmEnum();
  // EEhrToEmLaneBoundaryType -> LaneBoundaryType
  static void LaneBoundaryTypeConvert(const common::EEhrToEmLaneBoundaryType& ehr_type,
                                      data::em_data::LaneBoundaryType& em_type);

  static void LaneRoadedgeTypeConvert(const common::EEhrToEmRoadEdgeType& ehr_type,
                                      data::em_data::RoadEdgeType& em_type);

  //   // EEhrToEmLaneType -> LaneType, using by bit
  //   static void LaneTypeConvert(const uint32_t& ehr_type,
  //                               data::em_data::LaneType& em_type);

  // EEhrToEmLaneTransitionDirection -> LaneTransitionDirection
  static void LaneTransitionDirectionConvert(const common::EEhrToEmLaneTransitionDirection& ehr_type,
                                             const bc::uint16_t ehr_transition_id,
                                             data::em_data::LaneTransitionDirection& em_type,
                                             bc::uint16_t& em_transition_id);
//   // EEhrToEmLaneMarkingType -> LaneArrowType
//   static void LaneMarkingTypeConvert(const common::EEhrToEmLaneMarkingType& ehr_type,
//                                      data::em_data::LaneArrowType& em_type);

  // EEhrToEmGuidePointType -> MapGuidePointType
  static void GuidePointTypeConvert(const common::EEhrToEmGuidePointType& ehr_type,
                                    data::em_data::MapGuidePointType& em_type);

  // EEhrToEmSuperElevationClass -> SuperElevationType
  static void SuperElevationConvert(const common::EEhrToEmSuperElevationClass& ehr_type,
                                    data::em_data::SuperElevationType& em_type);
  // EEhrToEmSpecialSituationType -> SpecialSituation
  static void SpecialSituationTypeConvert(const common::EEhrToEmSpecialSituationType& ehr_type,
                                          data::em_data::SpecialSituation& em_type);

  // EEhrToEmEhrToEmColor -> LaneBoundaryColor
  static void LaneBoundaryColorConvert(const common::EEhrToEmColor& ehr_type,
                                       data::em_data::LaneBoundaryColor& em_type);
};