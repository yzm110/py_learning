#include "ehrprofileextract.h"
#include <iostream>
#include "ehr2emenum.h"
#include "ehradapterhelper.h"

void RhrProfileExtract::getEhrLaneProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                          zone::data::em_data::ReferenceLine& referline_profile,
                                          const zone::common::EhrToEmData& ehr_to_emdata)
{
  // centerLine <-> reference_line other profiles
  // is_on_rout_range_ / is_on_route_segs_;
  getEhrLaneIsOnRouteProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // is_in_intersection_range_  / is_in_intersection_segs_;
  getEhrLaneIsInIntersectionProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // speed_limit_range_ / speed_limit_segs_;
  getEhrLaneSpeedLimitProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_type_range_ / lane_type_segs_;
  getEhrLaneLaneTypeProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_slop_range_ / slope_segs_;
  getEhrLaneLaneSlopeProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_transition_direction_range_ / lane_transition_dir_segs_;
  getEhrLaneLaneTransitionDirectionProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // super_elevation_range_ / super_elevation_segs_;
  getEhrLaneSuperElevationProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_curvature_range_ / curvature_segs_;
  getEhrLaneLaneCurvatureProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_heading_range_ / heading_segs_;
  getEhrLaneLaneHeadingProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_width_range_ / width_segs_;
  getEhrLaneLaneWidthProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // lane_marking_range_ / arrow_segs_;
  getEhrLaneLaneMarkingProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);

  // special_situation_range_ / special_segs_;
  getEhrLaneSpecialSituationProfile(ehr2em_centerline, referline_profile, ehr_to_emdata);
}

void RhrProfileExtract::getEhrLaneIsOnRouteProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                   zone::data::em_data::ReferenceLine& referline_profile,
                                                   const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_is_on_rout_num = 0;
  uint32_t is_on_rout_start_data_index = ehr2em_centerline.is_on_rout_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.is_on_rout_range_.total_number_ &&
                               total_is_on_rout_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange is_on_rout_range =
        ehr_to_emdata.lane_elements_data_idx_[is_on_rout_start_data_index + range_idx];
    uint32_t is_on_rout_start_index = is_on_rout_range.start_index_;
    uint32_t is_on_rout_total_number = is_on_rout_range.total_number_;
    for (uint32_t i = 0;
         i < is_on_rout_total_number && total_is_on_rout_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement; i++)
    {
      referline_profile.is_on_route_segs_[total_is_on_rout_num].start_s_ =
          ehr_to_emdata.is_on_rout_[is_on_rout_start_index + i].start_offset_ / (100.f);
      referline_profile.is_on_route_segs_[total_is_on_rout_num].end_s_ =
          ehr_to_emdata.is_on_rout_[is_on_rout_start_index + i].end_offset_ / (100.f);
      referline_profile.is_on_route_segs_[total_is_on_rout_num].is_on_route_ =
          ehr_to_emdata.is_on_rout_[is_on_rout_start_index + i].is_on_route_;
      referline_profile.is_on_route_segs_[total_is_on_rout_num].valid_ = bc::true_v;
      total_is_on_rout_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneIsInIntersectionProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                          zone::data::em_data::ReferenceLine& referline_profile,
                                                          const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_is_in_intersection_num = 0;
  uint32_t is_in_intersection_start_data_index = ehr2em_centerline.is_in_intersection_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.is_in_intersection_range_.total_number_ &&
                               total_is_in_intersection_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange is_in_intersection_range =
        ehr_to_emdata.lane_elements_data_idx_[is_in_intersection_start_data_index + range_idx];
    uint32_t is_in_intersection_start_index = is_in_intersection_range.start_index_;
    uint32_t is_in_intersection_total_number = is_in_intersection_range.total_number_;
    for (uint32_t i = 0; i < is_in_intersection_total_number &&
                         total_is_in_intersection_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
         i++)
    {
      referline_profile.is_in_intersection_segs_[total_is_in_intersection_num].start_s_ =
          ehr_to_emdata.is_in_intersection_[is_in_intersection_start_index + i].start_offset_ / (100.f);
      referline_profile.is_in_intersection_segs_[total_is_in_intersection_num].end_s_ =
          ehr_to_emdata.is_in_intersection_[is_in_intersection_start_index + i].end_offset_ / (100.f);
      referline_profile.is_in_intersection_segs_[total_is_in_intersection_num].is_in_intersection_ =
          ehr_to_emdata.is_in_intersection_[is_in_intersection_start_index + i].is_in_intersection_;
      referline_profile.is_in_intersection_segs_[total_is_in_intersection_num].valid_ = bc::true_v;
      total_is_in_intersection_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneSpeedLimitProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                    zone::data::em_data::ReferenceLine& referline_profile,
                                                    const zone::common::EhrToEmData& ehr_to_emdata)
{
  bc::TCArray<::zone::common::EhrToEmSpeedLimit, 100> speed_limit_segs;
  bc::uint32_t total_speed_limit_num = 0;
  int speed_limit_range_start_data_index = ehr2em_centerline.speed_limit_range_.start_index_;
  for (int i = 0; i < ehr2em_centerline.speed_limit_range_.total_number_ && total_speed_limit_num < 100; i++)
  {
    int speed_limit_range_start_index =
        ehr_to_emdata.lane_elements_data_idx_[speed_limit_range_start_data_index + i].start_index_;
    int speed_limit_range_total_number =
        ehr_to_emdata.lane_elements_data_idx_[speed_limit_range_start_data_index + i].total_number_;
    for (int j = speed_limit_range_start_index;
         j < speed_limit_range_start_index + speed_limit_range_total_number && total_speed_limit_num < 100; j++)
    {
      speed_limit_segs[total_speed_limit_num] = ehr_to_emdata.speed_limit_[j];
      total_speed_limit_num++;
    }
  }
  uint32_t t_speed_limit_ego_idx = 0;
  uint32_t t_speed_limit_start_idx = 0;
  for (uint32_t i = 0; i < total_speed_limit_num; i++)
  {
    if (t_speed_limit_start_idx == 0 && speed_limit_segs[i].end_offset_ / (100.f) > -40.f)
    {
      t_speed_limit_start_idx = i;
    }
    if (speed_limit_segs[i].start_offset_ / (100.f) <= 0.f && speed_limit_segs[i].end_offset_ / (100.f) >= 0.f)
    {
      t_speed_limit_ego_idx = i;
      break;
    }
  }
  uint32_t t_speed_limit_cur_idx = 0;
  if (t_speed_limit_ego_idx < t_speed_limit_start_idx + (uint32_t)kMaxLanePropertySegsInOneLaneElement)
  {
    for (uint32_t i = t_speed_limit_start_idx; i <= t_speed_limit_ego_idx; i++)
    {
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].start_s_ =
          speed_limit_segs[i].start_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].end_s_ =
          speed_limit_segs[i].end_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .min_spd_limit_ = speed_limit_segs[i].min_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .max_spd_limit_ = speed_limit_segs[i].max_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].valid_ =
          bc::true_v;
      t_speed_limit_cur_idx++;
    }
    for (uint32_t i = t_speed_limit_ego_idx + 1;
         i < std::min(total_speed_limit_num, (uint32_t)kMaxLanePropertySegsInOneLaneElement); i++)
    {
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].start_s_ =
          speed_limit_segs[i].start_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].end_s_ =
          speed_limit_segs[i].end_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .min_spd_limit_ = speed_limit_segs[i].min_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .max_spd_limit_ = speed_limit_segs[i].max_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].valid_ =
          bc::true_v;
      t_speed_limit_cur_idx++;
    }
  }
  else
  {
    for (uint32_t i = t_speed_limit_ego_idx - (uint32_t)kMaxLanePropertySegsInOneLaneElement + 1;
         i <= t_speed_limit_ego_idx; i++)
    {
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].start_s_ =
          speed_limit_segs[i].start_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].end_s_ =
          speed_limit_segs[i].end_offset_ / (100.f);
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .min_spd_limit_ = speed_limit_segs[i].min_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
          .speed_limit_segs_[t_speed_limit_cur_idx]
          .max_spd_limit_ = speed_limit_segs[i].max_speed_limit_;
      referline_profile.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_[t_speed_limit_cur_idx].valid_ =
          bc::true_v;
      t_speed_limit_cur_idx++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneTypeProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                  zone::data::em_data::ReferenceLine& referline_profile,
                                                  const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_type_num = 0;
  uint32_t lane_type_range_start_data_index = ehr2em_centerline.lane_type_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_type_range_.total_number_ &&
                               total_lane_type_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_type_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_type_range_start_data_index + range_idx];
    uint32_t lane_type_range_start_index = lane_type_range.start_index_;
    uint32_t lane_type_range_total_number = lane_type_range.total_number_;
    for (uint32_t i = 0;
         i < lane_type_range_total_number && total_lane_type_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement; i++)
    {
      referline_profile.lane_type_segs_[total_lane_type_num].start_s_ =
          ehr_to_emdata.lane_type_[lane_type_range_start_index + i].start_offset_ / (100.f);
      referline_profile.lane_type_segs_[total_lane_type_num].end_s_ =
          ehr_to_emdata.lane_type_[lane_type_range_start_index + i].end_offset_ / (100.f);
      // EEhrToEmLaneType use bit
      referline_profile.lane_type_segs_[total_lane_type_num].lane_type_ =
          ehr_to_emdata.lane_type_[lane_type_range_start_index + i].lane_type_;
      referline_profile.lane_type_segs_[total_lane_type_num].valid_ = bc::true_v;
      total_lane_type_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneSlopeProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                   zone::data::em_data::ReferenceLine& referline_profile,
                                                   const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_slop_num = 0;
  uint32_t lane_slop_range_start_data_index = ehr2em_centerline.lane_slop_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_slop_range_.total_number_ &&
                               total_lane_slop_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_slop_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_slop_range_start_data_index + range_idx];
    uint32_t lane_slop_range_start_index = lane_slop_range.start_index_;
    uint32_t lane_slop_range_total_number = lane_slop_range.total_number_;
    for (uint32_t i = 0;
         i < lane_slop_range_total_number && total_lane_slop_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement; i++)
    {
      referline_profile.slope_segs_[total_lane_slop_num].start_s_ =
          ehr_to_emdata.lane_slop_[lane_slop_range_start_index + i].start_offset_ / (100.f);
      referline_profile.slope_segs_[total_lane_slop_num].end_s_ =
          ehr_to_emdata.lane_slop_[lane_slop_range_start_index + i].end_offset_ / (100.f);
      referline_profile.slope_segs_[total_lane_slop_num].start_slope_ =
          ehr_to_emdata.lane_slop_[lane_slop_range_start_index + i].start_slop_;
      referline_profile.slope_segs_[total_lane_slop_num].end_slope_ =
          ehr_to_emdata.lane_slop_[lane_slop_range_start_index + i].end_slop_;
      referline_profile.slope_segs_[total_lane_slop_num].valid_ = bc::true_v;
      total_lane_slop_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneTransitionDirectionProfile(
    const zone::common::EhrToEmCenterLine ehr2em_centerline, zone::data::em_data::ReferenceLine& referline_profile,
    const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_transition_direction_num = 0;
  uint32_t lane_transition_direction_start_data_index = ehr2em_centerline.lane_transition_direction_range_.start_index_;
  // zone::common::EEhrToEmLaneTransitionDirection last_lane_transition =
  //     zone::common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_UNKNOWN;
  zone::data::em_data::LaneTransitionDirection last_lane_transition =
      zone::data::em_data::LaneTransitionDirection::kUnknown;
  uint16_t last_lane_transition_id = 0;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_transition_direction_range_.total_number_ &&
                               total_lane_transition_direction_num < (uint32_t)kMaxLaneTransitionSegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_transition_direction_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_transition_direction_start_data_index + range_idx];
    uint32_t lane_transition_direction_start_index = lane_transition_direction_range.start_index_;
    uint32_t lane_transition_direction_total_number = lane_transition_direction_range.total_number_;
    for (uint32_t i = 0; i < lane_transition_direction_total_number &&
                         total_lane_transition_direction_num <= (uint32_t)kMaxLaneTransitionSegsInOneLaneElement;
         i++)
    {
      if (ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].end_offset_ / (100.f) >
          -40.f)
      {
        zone::data::em_data::LaneTransitionDirection cur_lane_transition =
            zone::data::em_data::LaneTransitionDirection::kNormal;
        bc::uint16_t cur_lane_transition_id = 0;
        Ehr2EmEnum::LaneTransitionDirectionConvert(
            ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i]
                .lane_transition_direction_,
            ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].related_element_id_,
            cur_lane_transition, cur_lane_transition_id);
        if ((last_lane_transition != cur_lane_transition || last_lane_transition_id != cur_lane_transition_id) &&
            total_lane_transition_direction_num < (uint32_t)kMaxLaneTransitionSegsInOneLaneElement)
        {
          referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].start_s_ =
              ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].start_offset_ /
              (100.f);
          referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].end_s_ =
              ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].end_offset_ / (100.f);
          // Ehr2EmEnum::LaneTransitionDirectionConvert(
          //     ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i]
          //         .lane_transition_direction_,
          //     referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].lane_trans_dir_);
          referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].lane_trans_dir_ =
              cur_lane_transition;
          referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].related_element_id_ =
              ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].related_element_id_;
          referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num].valid_ = bc::true_v;
          last_lane_transition = cur_lane_transition;
          last_lane_transition_id =
              ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].related_element_id_;
          total_lane_transition_direction_num++;
        }
        else
        {
          if (total_lane_transition_direction_num == 0)
          {
#ifdef STD_COUT_ENABLE
            std::cout
                << "Error occurs in RhrProfileExtract::getEhrLaneLaneTransitionDirectionProfile: Unknown transition!!!"
                << std::endl;
#endif
            continue;
          }
          else if (last_lane_transition == cur_lane_transition && last_lane_transition_id == cur_lane_transition_id)
          {
            referline_profile.lane_transition_dir_segs_[total_lane_transition_direction_num - 1].end_s_ =
                ehr_to_emdata.lane_transition_direction_[lane_transition_direction_start_index + i].end_offset_ /
                (100.f);
          }
          else
          {
            total_lane_transition_direction_num++;
          }
        }
      }
    }
  }
}

void RhrProfileExtract::getEhrLaneSuperElevationProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                        zone::data::em_data::ReferenceLine& referline_profile,
                                                        const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_super_elevation_num = 0;
  uint32_t super_elevation_start_data_index = ehr2em_centerline.super_elevation_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.super_elevation_range_.total_number_ &&
                               total_super_elevation_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange super_elevation_range =
        ehr_to_emdata.lane_elements_data_idx_[super_elevation_start_data_index + range_idx];
    uint32_t super_elevation_start_index = super_elevation_range.start_index_;
    uint32_t super_elevation_total_number = super_elevation_range.total_number_;
    for (uint32_t i = 0;
         i < super_elevation_total_number && total_super_elevation_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
         i++)
    {
      referline_profile.super_elevation_segs_[total_super_elevation_num].start_s_ =
          ehr_to_emdata.super_elevation_[super_elevation_start_index + i].start_offset_ / (100.f);
      referline_profile.super_elevation_segs_[total_super_elevation_num].end_s_ =
          ehr_to_emdata.super_elevation_[super_elevation_start_index + i].end_offset_ / (100.f);
      Ehr2EmEnum::SuperElevationConvert(
          ehr_to_emdata.super_elevation_[super_elevation_start_index + i].super_elevation_class_,
          referline_profile.super_elevation_segs_[total_super_elevation_num].elevation_);
      referline_profile.super_elevation_segs_[total_super_elevation_num].valid_ = bc::true_v;
      total_super_elevation_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneCurvatureProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                       zone::data::em_data::ReferenceLine& referline_profile,
                                                       const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_curvature_num = 0;
  uint32_t lane_curvature_start_data_index = ehr2em_centerline.lane_curvature_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_curvature_range_.total_number_ &&
                               total_lane_curvature_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_curvature_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_curvature_start_data_index + range_idx];
    uint32_t lane_curvature_index = lane_curvature_range.start_index_;
    uint32_t lane_curvature_total_number = lane_curvature_range.total_number_;
    for (uint32_t i = 0;
         i < lane_curvature_total_number && total_lane_curvature_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
         i++)
    {
      referline_profile.curvature_segs_[total_lane_curvature_num].start_s_ =
          ehr_to_emdata.lane_curvature_[lane_curvature_index + i].start_offset_ / (100.f);
      referline_profile.curvature_segs_[total_lane_curvature_num].end_s_ =
          ehr_to_emdata.lane_curvature_[lane_curvature_index + i].end_offset_ / (100.f);
      referline_profile.curvature_segs_[total_lane_curvature_num].start_curvature_ =
          ehr_to_emdata.lane_curvature_[lane_curvature_index + i].start_curvature_;
      referline_profile.curvature_segs_[total_lane_curvature_num].end_curvature_ =
          ehr_to_emdata.lane_curvature_[lane_curvature_index + i].end_curvature_;
      referline_profile.curvature_segs_[total_lane_curvature_num].valid_ = bc::true_v;
      total_lane_curvature_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneHeadingProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                     zone::data::em_data::ReferenceLine& referline_profile,
                                                     const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_heading_num = 0;
  uint32_t lane_heading_start_data_index = ehr2em_centerline.lane_heading_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_heading_range_.total_number_ &&
                               total_lane_heading_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_heading_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_heading_start_data_index + range_idx];
    uint32_t lane_heading_start_index = lane_heading_range.start_index_;
    uint32_t lane_heading_total_number = lane_heading_range.total_number_;
    for (uint32_t i = 0;
         i < lane_heading_total_number && total_lane_heading_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement; i++)
    {
      referline_profile.heading_segs_[total_lane_heading_num].start_s_ =
          ehr_to_emdata.lane_heading_[lane_heading_start_index + i].start_offset_ / (100.f);
      referline_profile.heading_segs_[total_lane_heading_num].end_s_ =
          ehr_to_emdata.lane_heading_[lane_heading_start_index + i].end_offset_ / (100.f);
      referline_profile.heading_segs_[total_lane_heading_num].start_heading_ =
          ehr_to_emdata.lane_heading_[lane_heading_start_index + i].start_heading_;
      referline_profile.heading_segs_[total_lane_heading_num].end_heading_ =
          ehr_to_emdata.lane_heading_[lane_heading_start_index + i].end_heading_;
      referline_profile.heading_segs_[total_lane_heading_num].valid_ = bc::true_v;
      total_lane_heading_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneLaneWidthProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                   zone::data::em_data::ReferenceLine& referline_profile,
                                                   const zone::common::EhrToEmData& ehr_to_emdata)
{
  bc::TCArray<::zone::common::EhrToEmLaneWidth, 100> lane_width_segs;
  bc::uint32_t total_lane_width_num = 0;
  int lane_width_range_start_data_index = ehr2em_centerline.lane_width_range_.start_index_;
  for (int i = 0; i < ehr2em_centerline.lane_width_range_.total_number_ && total_lane_width_num < 100; i++)
  {
    int lane_width_range_start_index =
        ehr_to_emdata.lane_elements_data_idx_[lane_width_range_start_data_index + i].start_index_;
    int lane_width_range_total_number =
        ehr_to_emdata.lane_elements_data_idx_[lane_width_range_start_data_index + i].total_number_;
    for (int j = lane_width_range_start_index;
         j < lane_width_range_start_index + lane_width_range_total_number && total_lane_width_num < 100; j++)
    {
      lane_width_segs[total_lane_width_num] = ehr_to_emdata.lane_width_[j];
      total_lane_width_num++;
    }
  }
  uint32_t t_width_ego_idx = 0;
  uint32_t t_width_start_idx = 0;
  for (uint32_t i = 0; i < total_lane_width_num; i++)
  {
    if (t_width_start_idx == 0 && lane_width_segs[i].end_offset_ / (100.f) > -40.f)
    {
      t_width_start_idx = i;
    }
    if (lane_width_segs[i].start_offset_ / (100.f) <= 0.f && lane_width_segs[i].end_offset_ / (100.f) >= 0.f)
    {
      t_width_ego_idx = i;
      break;
    }
  }
  uint32_t t_width_cur_idx = 0;
  if (t_width_ego_idx < t_width_start_idx + (uint32_t)kMaxLanePropertySegsInOneLaneElement)
  {
    for (uint32_t i = t_width_start_idx; i <= t_width_ego_idx; i++)
    {
      referline_profile.width_segs_[t_width_cur_idx].start_s_ = lane_width_segs[i].start_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_s_ = lane_width_segs[i].end_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].start_width_ = lane_width_segs[i].start_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_width_ = lane_width_segs[i].end_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].valid_ = bc::true_v;
      t_width_cur_idx++;
    }
    for (uint32_t i = t_width_ego_idx + 1;
         i < std::min(total_lane_width_num, (uint32_t)kMaxLanePropertySegsInOneLaneElement); i++)
    {
      referline_profile.width_segs_[t_width_cur_idx].start_s_ = lane_width_segs[i].start_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_s_ = lane_width_segs[i].end_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].start_width_ = lane_width_segs[i].start_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_width_ = lane_width_segs[i].end_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].valid_ = bc::true_v;
      t_width_cur_idx++;
    }
  }
  else
  {
    for (uint32_t i = t_width_ego_idx - (uint32_t)kMaxLanePropertySegsInOneLaneElement + 1; i <= t_width_ego_idx; i++)
    {
      referline_profile.width_segs_[t_width_cur_idx].start_s_ = lane_width_segs[i].start_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_s_ = lane_width_segs[i].end_offset_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].start_width_ = lane_width_segs[i].start_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].end_width_ = lane_width_segs[i].end_width_ / (100.f);
      referline_profile.width_segs_[t_width_cur_idx].valid_ = bc::true_v;
      t_width_cur_idx++;
    }
  }

  // // update lane width with offset
  // for (uint32_t i = 0; i < r_point_offsets.size() && i < (uint32_t)kMaxRefLinePtsNum; i++)
  // {
  //   for (uint32_t j = 0; j < total_lane_width_num; j++)
  //   {
  //     auto start_s = lane_width_segs[j].start_offset_ / (100.f);
  //     auto end_s = lane_width_segs[j].end_offset_ / (100.f);
  //     if (r_point_offsets[i] >= start_s && r_point_offsets[i] <= end_s)
  //     {
  //       auto start_width = lane_width_segs[j].start_width_ / (100.f);
  //       auto end_width = lane_width_segs[j].end_width_ / (100.f);
  //       ;
  //       if ((end_s - start_s) > 0.0)
  //       {
  //         referline_profile.ref_line_pts_[i].lane_width_ =
  //             ((end_s - start_s) * (end_width + start_width) - end_width * (end_s - r_point_offsets[i]) -
  //              start_width * (r_point_offsets[i] - start_s)) /
  //             (end_s - start_s);
  //         break;
  //       }
  //       else
  //       {
  //         std::cout << "ehr to em offset error: " << "s: " << start_s << " e: " << end_s << std::endl;
  //       }
  //     }
  //   }
  // }
}

void RhrProfileExtract::getEhrLaneLaneMarkingProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                     zone::data::em_data::ReferenceLine& referline_profile,
                                                     const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_lane_marking_num = 0;
  uint32_t lane_marking_start_data_index = ehr2em_centerline.lane_marking_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.lane_marking_range_.total_number_ &&
                               total_lane_marking_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange lane_marking_range =
        ehr_to_emdata.lane_elements_data_idx_[lane_marking_start_data_index + range_idx];
    uint32_t lane_marking_start_index = lane_marking_range.start_index_;
    uint32_t lane_marking_total_number = lane_marking_range.total_number_;
    for (uint32_t i = 0;
         i < lane_marking_total_number && total_lane_marking_num < (uint32_t)kMaxLanePropertySegsInOneLaneElement; i++)
    {
      referline_profile.arrow_segs_[total_lane_marking_num].start_s_ =
          ehr_to_emdata.lane_marking_[lane_marking_start_index + i].offset_ / (100.f);
      referline_profile.arrow_segs_[total_lane_marking_num].end_s_ =
          ehr_to_emdata.lane_marking_[lane_marking_start_index + i].offset_ / (100.f);
      referline_profile.arrow_segs_[total_lane_marking_num].arrow_type_ =
          ehr_to_emdata.lane_marking_[lane_marking_start_index + i].lane_marking_type_;
      referline_profile.arrow_segs_[total_lane_marking_num].id_ =
          ehr_to_emdata.lane_marking_[lane_marking_start_index + i].id_;
      referline_profile.arrow_segs_[total_lane_marking_num].valid_ = bc::true_v;
      total_lane_marking_num++;
    }
  }
}

void RhrProfileExtract::getEhrLaneSpecialSituationProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                          zone::data::em_data::ReferenceLine& referline_profile,
                                                          const zone::common::EhrToEmData& ehr_to_emdata)
{
  uint32_t total_special_situation_num = 0;
  uint32_t special_situation_start_data_index = ehr2em_centerline.special_situation_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr2em_centerline.special_situation_range_.total_number_ &&
                               total_special_situation_num < (uint32_t)kMaxSpecialSituationSegsInOneLaneElement;
       range_idx++)
  {
    zone::common::EhrToEmDataRange special_situation_range =
        ehr_to_emdata.lane_elements_data_idx_[special_situation_start_data_index + range_idx];
    uint32_t special_situation_start_index = special_situation_range.start_index_;
    uint32_t special_situation_total_number = special_situation_range.total_number_;
    for (uint32_t i = 0; i < special_situation_total_number &&
                         total_special_situation_num < (uint32_t)kMaxSpecialSituationSegsInOneLaneElement;
         i++)
    {
      if (ehr_to_emdata.special_situation_[special_situation_start_index + i].special_situation_type_ ==
          common::EEhrToEmSpecialSituationType::EHRTOEM_SPECIAL_SITUATION_TYPE_TRAFFIC_LIGHT)
      {
        continue;
      }
      referline_profile.special_segs_[total_special_situation_num].start_s_ =
          ehr_to_emdata.special_situation_[special_situation_start_index + i].start_offset_ / (100.f);
      referline_profile.special_segs_[total_special_situation_num].end_s_ =
          ehr_to_emdata.special_situation_[special_situation_start_index + i].end_offset_ / (100.f);
      Ehr2EmEnum::SpecialSituationTypeConvert(
          ehr_to_emdata.special_situation_[special_situation_start_index + i].special_situation_type_,
          referline_profile.special_segs_[total_special_situation_num].special_situation_);
      referline_profile.special_segs_[total_special_situation_num].valid_ = bc::true_v;
      total_special_situation_num++;
    }
  }
}

RhrProfileExtract::RhrProfileExtract(/* args */) {}

RhrProfileExtract::~RhrProfileExtract() {}