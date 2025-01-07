#include "ehrmapguidepointextract.h"
#include "common/data/ehr_to_em_data.h"
using ::zone::common::EEhrToEmGuidePointType;
using ::zone::data::em_data::MapGuidePointType;

void EhrMapGuidePointExtract::GuidePointInfo(
    bc::TCArray<zone::data::em_data::MapGuidePoint, kMaxMapGuidePtNum>& em_map_guide_pts,
    const zone::common::EhrToEmData& ehr_to_emdata)
{
  const uint32_t& guide_point_num = ehr_to_emdata.guide_point_num_;
  bc::uint8_t guide_idx = 0;
  for (uint32_t i = 0; i < guide_point_num && guide_idx < kMaxMapGuidePtNum; i++)
  {
    if (ehr_to_emdata.guide_point_[i].end_offset_ < -5000)  // -50m
    {
      continue;
    }
    em_map_guide_pts[guide_idx].start_s_ = ehr_to_emdata.guide_point_[i].start_offset_ / (100.f);
    em_map_guide_pts[guide_idx].end_s_ = ehr_to_emdata.guide_point_[i].end_offset_ / (100.f);
    em_map_guide_pts[guide_idx].is_on_route_ = ehr_to_emdata.guide_point_[i].is_on_route_;
    em_map_guide_pts[guide_idx].turn_angle_ = ehr_to_emdata.guide_point_[i].turn_angle_;
    em_map_guide_pts[guide_idx].target_lane_index_ = ehr_to_emdata.guide_point_[i].path_id_;
    if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NONE == ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kNone;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_DESTINATION ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kDestination;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_INTERSECTION ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kIntersection;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_TUNNEL == ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kTunnel;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_BRIDGE == ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kBridge;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ENTRANCE_RAMP ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kEntranceRamp;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_EXIT_RAMP ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kExitRamp;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_REST_AREA ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kRestArea;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_TOLL_AREA ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kTollArea;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ROUNDABOUT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kRoundabout;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_SUBPATH ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSubPath;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_JCT == ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kJCT;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_SPLIT_POINT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSplitPoint;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_MERGE_POINT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kMergePoint;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ROAD_SPLIT_POINT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kRoadSplit;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_ROAD_MERGE_POINT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kRoadMerge;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_IC_SPLIT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kICSplit;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_IC_MERGE ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kICMerge;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_JC_SPLIT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kJCSplit;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_JC_MERGE ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kJCMerge;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NOA_ENTRANCE_RAMP ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSwitchEntranceRamp;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NOA_EXIT_RAMP ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSwitchExitRamp;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NOA_ENTRANCE_JCT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSwitchEntranceJCT;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NOA_EXIT_JCT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSwitchExitJCT;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_OFFWAY == ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kSwitchOffway;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_REMAIN_DISTANCE ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kRemainDistance;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_NAVI_DISTANCE ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kNaviDistance;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_CONSTRUCTION ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kConstruction;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_TRAFFIC_LIGHT ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kTrafficLight;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_HIGHWAY_END ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kHighwayEnd;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_NATIONAL_HIGHWAY ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kNationalHighway;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_NO_GUARDRIAL ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kNoGuardrial;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LINK_SLOPE_OVERRUN ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLinkSlopeOverrun;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LINK_SLOPE_AVG_OVERRUN ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLinkSlopeAvgOverrun;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_UNPLAN_REGION ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kUnplanRegion;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LINK_SAPA_ENTER ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLinkSapaEnter;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LANE_CURVATURE_OVERRUN ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLaneCurvatureOverrun;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LINK_CURVATURE_OVERRUN ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLinkCurvatureOverrun;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LANE_EMGENCY_PARKING ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLaneEmergencyParking;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LANE_EMGENCY ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLaneEmergency;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LANE_TOO_NARROW ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLaneTooNarrow;
    }
    else if (EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_GEOFENCE_LANE_TOO_WIDE ==
             ehr_to_emdata.guide_point_[i].guide_point_type_)
    {
      em_map_guide_pts[guide_idx].type_ = MapGuidePointType::kLaneTooWide;
    }
    else
    {
      /** Do nothing*/
    }
    em_map_guide_pts[guide_idx].valid_ = bc::true_v;
    guide_idx++;
  }
}

EhrMapGuidePointExtract::EhrMapGuidePointExtract(/* args */) {}

EhrMapGuidePointExtract::~EhrMapGuidePointExtract() {}