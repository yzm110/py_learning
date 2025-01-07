#include "ehr2emenum.h"
#include "ehradapterhelper.h"

Ehr2EmEnum::Ehr2EmEnum(/* args */) {}

Ehr2EmEnum::~Ehr2EmEnum() {}

void Ehr2EmEnum::LaneBoundaryTypeConvert(const common::EEhrToEmLaneBoundaryType& ehr_type,
                                         data::em_data::LaneBoundaryType& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_UNKNOWN:
      em_type = data::em_data::LaneBoundaryType::kSolid;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_NONE:
      em_type = data::em_data::LaneBoundaryType::kVirtual;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_SOLID_LINE:
      em_type = data::em_data::LaneBoundaryType::kSolid;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_DASHED_LINE:
      em_type = data::em_data::LaneBoundaryType::kDash;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_DOUBLE_SOLID_LINE:
      em_type = data::em_data::LaneBoundaryType::kDoubleSolid;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_DOUBLE_DASHED_LINE:
      em_type = data::em_data::LaneBoundaryType::kDoubleDash;
      break;
    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_LEFT_SOLID_RIGHT_DASHED:
      em_type = data::em_data::LaneBoundaryType::kLeftSolidRightDash;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_RIGHT_SOLID_LEFT_DASHED:
      em_type = data::em_data::LaneBoundaryType::kRightSolidLeftDash;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_DASHED_BLOCKS:
      em_type = data::em_data::LaneBoundaryType::kDashedBlocks;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_SHADED_AREA:  // 角岛-> 实线
      em_type = data::em_data::LaneBoundaryType::kShadedArea;
      break;

    case common::EEhrToEmLaneBoundaryType::EHRTOEM_LANE_BOUNDARY_TYPE_PHYSICAL_DIVIDER:
      em_type = data::em_data::LaneBoundaryType::kPhysicalDivider;
      break;

    default:
      break;
  }
}

void Ehr2EmEnum::LaneRoadedgeTypeConvert(const common::EEhrToEmRoadEdgeType& ehr_type,
                                         data::em_data::RoadEdgeType& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmRoadEdgeType::EHRTOEM_ROAD_EDGE_TYPE_NONE:
      em_type = data::em_data::RoadEdgeType::kUnknown;
      break;

    case common::EEhrToEmRoadEdgeType::EHRTOEM_ROAD_EDGE_TYPE_CURB:
      em_type = data::em_data::RoadEdgeType::kCurb;
      break;

    case common::EEhrToEmRoadEdgeType::EHRTOEM_ROAD_EDGE_TYPE_BARRIER:
      em_type = data::em_data::RoadEdgeType::kFence;
      break;

    case common::EEhrToEmRoadEdgeType::EHRTOEM_ROAD_EDGE_TYPE_WALL:
      em_type = data::em_data::RoadEdgeType::kWall;
      break;

    case common::EEhrToEmRoadEdgeType::EHRTOEM_ROAD_EDGE_TYPE_PAVEMENT_EDGE:
      em_type = data::em_data::RoadEdgeType::kPavementEdge;
      break;

    default:
      break;
  }
}

// // 用位来表达车道类型
// void Ehr2EmEnum::LaneTypeConvert(const uint32_t & ehr_type,
//                                  data::em_data::LaneType& em_type) {
// #if 0
//   switch (ehr_type) {
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_NONE:
//       em_type = data::em_data::LaneType::kUnknown;
//       break;
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_REGULAR_LANE:
//       em_type = data::em_data::LaneType::kRegular;
//       break;
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_BUS_LANE:
//       em_type = data::em_data::LaneType::kBus;
//       break;
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_EMERGENCY_LANE:
//       em_type = data::em_data::LaneType::kEmergency;
//       break;
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_ACCELERATION_LANE:
//       em_type = data::em_data::LaneType::kAcceleration;
//       break;
//     case common::EEhrToEmLaneType::EHRTOEM_LANE_TYPE_DECELERATION_LANE:
//       em_type = data::em_data::LaneType::kDeceleration;
//       break;

//     default:
//       break;
//   }
// #endif
//   if (ehr_type && 0x1) {
//     em_type = data::em_data::LaneType::kRegular;
//   }
//   if (ehr_type && 0x40) {
//     em_type = data::em_data::LaneType::kBus;
//   }
//   if (ehr_type && 0x10) {
//     em_type = data::em_data::LaneType::kEmergency;
//   }
//   if (ehr_type && 0x100) {
//     em_type = data::em_data::LaneType::kAcceleration;
//   }
//   if (ehr_type && 0x200) {
//     em_type = data::em_data::LaneType::kDeceleration;
//   }
//   if (ehr_type && 0x0) {
//     em_type = data::em_data::LaneType::kUnknown;
//   }
// }

// 车道变更过渡类型
void Ehr2EmEnum::LaneTransitionDirectionConvert(const common::EEhrToEmLaneTransitionDirection& ehr_type,
                                                const bc::uint16_t ehr_transition_id,
                                                data::em_data::LaneTransitionDirection& em_type,
                                                bc::uint16_t& em_transition_id)
{
  switch (ehr_type)
  {
    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_NORMAL:
      em_type = data::em_data::LaneTransitionDirection::kNormal;
      em_transition_id = ehr_transition_id;
      break;
    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kSplitToLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kSplitToRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_LEFT_AND_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kSplitToLeftAndRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_MERGE_FROM_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kMergeFromLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_MERGE_FROM_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kMergeFromRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_OPEN_TO_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kOpenToLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_OPEN_TO_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kOpenToRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_CLOSE_FROM_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kCloseFromLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_CLOSE_FROM_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kCloseFromRight;
      em_transition_id = ehr_transition_id;
      break;

    /// TODO: Temperory use as normal
    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_FROM_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kSplitFromLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_FROM_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kSplitFromRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_MERGE_TO_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kMergeToLeft;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_MERGE_TO_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kMergeToRight;
      em_transition_id = ehr_transition_id;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_OPEN_FROM_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kNormal;
      em_transition_id = 0;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_OPEN_FROM_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kNormal;
      em_transition_id = 0;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_CLOSE_TO_LEFT:
      em_type = data::em_data::LaneTransitionDirection::kNormal;
      em_transition_id = 0;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_CLOSE_TO_RIGHT:
      em_type = data::em_data::LaneTransitionDirection::kNormal;
      em_transition_id = 0;
      break;

    case common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_UNKNOWN:
      em_type = data::em_data::LaneTransitionDirection::kUnknown;
      em_transition_id = ehr_transition_id;
      break;
    default:
      break;
  }
}
// // 车道标记类型 LaneArrowType
// void Ehr2EmEnum::LaneMarkingTypeConvert(const common::EEhrToEmLaneMarkingType& ehr_type,
//                                         data::em_data::LaneArrowType& em_type)
// {
//   switch (ehr_type)
//   {
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_NONE:
//       em_type = data::em_data::LaneArrowType::kNone;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_LEFT:
//       em_type = data::em_data::LaneArrowType::kLeft;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_RIGHT:
//       em_type = data::em_data::LaneArrowType::kRight;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_STRAIGHT_AND_LEFT:
//       em_type = data::em_data::LaneArrowType::kStraightAndLeft;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_STRAIGHT_AND_RIGHT:
//       em_type = data::em_data::LaneArrowType::kStraightAndRight;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_LEFT_AND_RIGHT:
//       em_type = data::em_data::LaneArrowType::kLeftAndRight;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_UTURN:
//       em_type = data::em_data::LaneArrowType::kUturn;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_STRAIGHT_AND_UTURN:
//       em_type = data::em_data::LaneArrowType::kStaightAndUturn;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_LEFT_AND_UTURN:
//       em_type = data::em_data::LaneArrowType::kLeftAndUturn;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_LEFT_CONFLUENCE:
//       em_type = data::em_data::LaneArrowType::kLeftConfluence;
//       break;
//     case common::EEhrToEmLaneMarkingType::EHRTOEM_LANE_MARKING_TYPE_RIGHT_CONFLUENCE:
//       em_type = data::em_data::LaneArrowType::kRightConfluence;
//       break;
//     default:
//       break;
//   }
// }

// EEhrToEmGuidePointType -> MapGuidePointType
void Ehr2EmEnum::GuidePointTypeConvert(const common::EEhrToEmGuidePointType& ehr_type,
                                       data::em_data::MapGuidePointType& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NONE:
      em_type = data::em_data::MapGuidePointType::kNone;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_DESTINATION:
      em_type = data::em_data::MapGuidePointType::kDestination;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_INTERSECTION:
      em_type = data::em_data::MapGuidePointType::kIntersection;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_TUNNEL:
      em_type = data::em_data::MapGuidePointType::kTunnel;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_BRIDGE:
      em_type = data::em_data::MapGuidePointType::kBridge;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ENTRANCE_RAMP:
      em_type = data::em_data::MapGuidePointType::kEntranceRamp;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_EXIT_RAMP:
      em_type = data::em_data::MapGuidePointType::kExitRamp;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_REST_AREA:
      em_type = data::em_data::MapGuidePointType::kRestArea;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_TOLL_AREA:
      em_type = data::em_data::MapGuidePointType::kTollArea;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ROUNDABOUT:
      em_type = data::em_data::MapGuidePointType::kRoundabout;
      break;
    case common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_SUBPATH:
      em_type = data::em_data::MapGuidePointType::kSubPath;
      break;
    default:
      break;
  }
}

// EEhrToEmSuperElevationClass -> SuperElevationType
void Ehr2EmEnum::SuperElevationConvert(const common::EEhrToEmSuperElevationClass& ehr_type,
                                       data::em_data::SuperElevationType& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_0_TOWARDS_CURB:
      em_type = data::em_data::SuperElevationType::k0om2TowardsCurb;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_2_TOWARDS_CURB:
      em_type = data::em_data::SuperElevationType::k2om4TowardsCurb;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_4_TOWARDS_CURB:
      em_type = data::em_data::SuperElevationType::k4To6TowardsCurb;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_6_TOWARDS_CURB:
      em_type = data::em_data::SuperElevationType::k6To8TowardsCurb;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_8_TOWARDS_CURB:
      em_type = data::em_data::SuperElevationType::kMoreThan8TowardsCurb;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_0_TOWARDS_MIDDLE:
      em_type = data::em_data::SuperElevationType::k0To2TowardsMiddle;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_2_TOWARDS_MIDDLE:
      em_type = data::em_data::SuperElevationType::k2To4TowardsMiddle;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_4_TOWARDS_MIDDLE:
      em_type = data::em_data::SuperElevationType::k4To6TowardsMiddle;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_6_TOWARDS_MIDDLE:
      em_type = data::em_data::SuperElevationType::k6To8TowardsMiddle;
      break;
    case common::EEhrToEmSuperElevationClass::EHRTOEM_SUPER_ELEVATION_CLASS_MORE_THAN_8_TOWARDS_MIDDLE:
      em_type = data::em_data::SuperElevationType::kMoreThan8TowardsMiddle;
      break;

    default:
      break;
  }
}

void Ehr2EmEnum::SpecialSituationTypeConvert(const common::EEhrToEmSpecialSituationType& ehr_type,
                                             data::em_data::SpecialSituation& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_NONE:
      em_type = data::em_data::SpecialSituation::kNone;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_TOLL_BOOTH:
      em_type = data::em_data::SpecialSituation::kTollBooth;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_PEDESTRIAN_CROSSING:
      em_type = data::em_data::SpecialSituation::kPedestrianCorssing;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_SPEED_BUMP:
      em_type = data::em_data::SpecialSituation::kSpeedBump;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_STOPPING_LOCATION:
      em_type = data::em_data::SpecialSituation::kStoppingLocation;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_NOPARKING_AREA:
      em_type = data::em_data::SpecialSituation::kNoParkingArea;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_TRAFFIC_LIGHT:
      em_type = data::em_data::SpecialSituation::kTrafficLight;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANEMARKING_BROKEN:
      em_type = data::em_data::SpecialSituation::kLanemarkingBroken;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LINK_SLOPE_AVG_OVERRUN:
      em_type = data::em_data::SpecialSituation::kLinkSlopeAvgOverrun;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LINK_SAPA_ENTER:
      em_type = data::em_data::SpecialSituation::kLinkSapaEnter;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANE_CURVATURE_OVERRUN:
      em_type = data::em_data::SpecialSituation::kLaneCurvatureOverrun;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANE_EMGENCY_PARKING:
      em_type = data::em_data::SpecialSituation::kLaneEmergencyParking;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANE_EMGENCY:
      em_type = data::em_data::SpecialSituation::kLaneEmergency;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANE_TOO_NARROW:
      em_type = data::em_data::SpecialSituation::kLaneTooNarrow;
      break;
    case common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_LANE_TOO_WIDE:
      em_type = data::em_data::SpecialSituation::kLaneTooWide;
      break;
    default:
      break;
  }
}

void Ehr2EmEnum::LaneBoundaryColorConvert(const common::EEhrToEmColor& ehr_type,
                                          data::em_data::LaneBoundaryColor& em_type)
{
  switch (ehr_type)
  {
    case common::EEhrToEmColor::EHRTOEM_COLOR_NONE:
      em_type = data::em_data::LaneBoundaryColor::kUnknown;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_OTHER:
      em_type = data::em_data::LaneBoundaryColor::kOther;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_WHITE:
      em_type = data::em_data::LaneBoundaryColor::kWhite;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_YELLOW:
      em_type = data::em_data::LaneBoundaryColor::kYellow;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_ORANGE:
      em_type = data::em_data::LaneBoundaryColor::kOrange;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_RED:
      em_type = data::em_data::LaneBoundaryColor::kRed;
      break;
    case common::EEhrToEmColor::EHRTOEM_COLOR_BLUE:
      em_type = data::em_data::LaneBoundaryColor::kBlue;
      break;
    default:
      break;
  }
}