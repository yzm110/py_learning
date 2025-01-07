#include "ehrdatahandle.h"
#include <iostream>
#include <vector>
#include "ehr2emenum.h"
#include "ehrmapgeofenceextract.h"
#include "ehrmapguidepointextract.h"
#include "ehrprofileextract.h"
#include "geographic_transform/geographic_geometry.h"

static void Point2Point3D(const zone::common::Point point, zone::common::Point3D& point3d)
{
  point3d.x = (float32_t)point.x;
  point3d.y = (float32_t)point.y;
  point3d.z = (float32_t)point.z;
}

void EhrDataHandle::setEhrToEmData(const ::zone::common::EhrToEmData& ehr_to_emdata)
{
  ehr_to_emdata_ptr_ = &ehr_to_emdata;
}

void EhrDataHandle::getEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data)
{
  // lane_data_,EHR(right -> left),（EM left->right）
  bc::bool_t has_dest_lane = bc::false_v;
  bc::bool_t ego_is_dest = bc::false_v;
  bc::int32_t dest_idx = -1;
  bc::uint8_t main_lane_start_idx = 0;
  if (-1 != ehr_to_emdata_ptr_->host_lane_idx_)
  {
    for (int32_t i = 0; i < static_cast<int>(ehr_to_emdata_ptr_->lane_data_num_); i++)
    {
      const zone::common::EhrToEmDataRange& lane_elements_range =
          ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_;
      if (((2 + (int32_t)(ehr_to_emdata_ptr_->host_lane_idx_ - i)) > -1) &&
          ((int32_t)(2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)) < 5))
      {
        if (lane_elements_range.total_number_ > 0)
        {
          map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].lane_valid_ = bc::true_v;
        }
        else
        {
          map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].lane_valid_ = bc::false_v;
        }
        auto& em_laneElements = map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].lane_elements_;
        getLaneElements(lane_elements_range, em_laneElements, map_em_data.nearst_split_scenario_,
                        (int32_t)(ehr_to_emdata_ptr_->host_lane_idx_ - i));
        auto& t_lanes = map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)];
        if (t_lanes.lane_elements_[0].lane_type_ == 1)
        {
          t_lanes.main_lane_idx_ = main_lane_start_idx;
          main_lane_start_idx++;
        }
        t_lanes.global_lane_idx_ = static_cast<uint32_t>(i);
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].reference_line_.current_point_idx_ > (kMaxRefLinePtsNum - 2))
            t_lanes.lane_elements_[k].element_valid_ = bc::false_v;
        }
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].element_valid_)
          {
            map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].valid_elements_cnts_++;
          }
        }
        if (map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].valid_elements_cnts_ == 0)
        {
          map_em_data.lanes_[2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)].lane_valid_ = bc::false_v;
        }
        if (0 == (ehr_to_emdata_ptr_->host_lane_idx_ - i))
          ego_is_dest = getDestLaneElements(lane_elements_range, dest_idx);
      }
      else
      {
        if (lane_elements_range.total_number_ > 0)
        {
          int32_t start_index = lane_elements_range.start_index_;
          uint32_t total_lane_type_num = 0;
          uint32_t lane_type_range_start_data_index =
              ehr_to_emdata_ptr_->lane_elements_[start_index].center_line_.lane_type_range_.start_index_;
          bc::bool_t find_ego_segment = bc::false_v;
          for (uint32_t range_idx = 0; range_idx < (uint32_t)ehr_to_emdata_ptr_->lane_elements_[start_index]
                                                       .center_line_.lane_type_range_.total_number_ &&
                                       !find_ego_segment;
               range_idx++)
          {
            zone::common::EhrToEmDataRange lane_type_range =
                ehr_to_emdata_ptr_->lane_elements_data_idx_[lane_type_range_start_data_index + range_idx];
            uint32_t lane_type_range_start_index = lane_type_range.start_index_;
            uint32_t lane_type_range_total_number = lane_type_range.total_number_;
            for (uint32_t i = 0; i < lane_type_range_total_number && !find_ego_segment; i++)
            {
              if (ehr_to_emdata_ptr_->lane_type_[lane_type_range_start_index + i].start_offset_ <= 0 &&
                  ehr_to_emdata_ptr_->lane_type_[lane_type_range_start_index + i].end_offset_ >= 0)
              {
                if (ehr_to_emdata_ptr_->lane_type_[lane_type_range_start_index + i].lane_type_ == 1)
                {
                  main_lane_start_idx++;
                }
                find_ego_segment = bc::true_v;
              }
            }
          }
        }
      }

      if (getDestLaneElements(lane_elements_range) && (!ego_is_dest))
      {
        if (map_em_data.relat_dest_lane_ == 0)
        {
          map_em_data.relat_dest_lane_ = i - ehr_to_emdata_ptr_->host_lane_idx_;
        }
        else if ((i - ehr_to_emdata_ptr_->host_lane_idx_) * map_em_data.relat_dest_lane_ > 0)
        {
          if ((map_em_data.relat_dest_lane_ > 0 &&
               i - ehr_to_emdata_ptr_->host_lane_idx_ < map_em_data.relat_dest_lane_) ||
              (map_em_data.relat_dest_lane_ < 0 &&
               i - ehr_to_emdata_ptr_->host_lane_idx_ > map_em_data.relat_dest_lane_))
          {
            map_em_data.relat_dest_lane_ = i - ehr_to_emdata_ptr_->host_lane_idx_;
          }
        }
        has_dest_lane = bc::true_v;
      }
    };
    if (ego_is_dest)
    {
      if (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].element_valid_ &&
          map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[1].element_valid_)
      {
        if (!map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].is_dest_lane_ele_ &&
            !map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[1].is_dest_lane_ele_)
        {
          map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[1].is_dest_lane_ele_ = bc::true_v;
          map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].is_dest_lane_ele_ = bc::false_v;
        }
      }
      else
      {
        map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].is_dest_lane_ele_ = bc::true_v;
      }
      map_em_data.relat_dest_lane_ = 0;
    }
    else if (!has_dest_lane)
    {
      bc::uint8_t dest_lane = kMaxLaneNum;
      bc::uint8_t dest_element = kMaxElemNumInOneLane;
      bc::float32_t max_route_distance = 0.f;
      // if (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].element_valid_ &&
      //     map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_ >
      //     max_route_distance)
      // {
      //   max_route_distance = map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_;
      //   dest_lane = ::zone::data::em_data::kHostLane;
      //   dest_element = 0;
      // }
      // for (bc::uint8_t i = 0; i < kMaxLaneNum; ++i)
      // {
      //   for (bc::uint8_t j = 0; j < kMaxElemNumInOneLane; ++j)
      //   {
      //     if (map_em_data.lanes_[i].lane_elements_[j].element_valid_)
      //     {
      //       if (map_em_data.lanes_[i].lane_elements_[j].route_left_dis_ > max_route_distance)
      //       {
      //         max_route_distance = map_em_data.lanes_[i].lane_elements_[j].route_left_dis_;
      //         dest_lane = i;
      //         dest_element = j;
      //       }
      //     }
      //   }
      // }
      // bc::bool_t out_of_lane = bc::false_v;
      // for (bc::uint32_t i = 0; i < ehr_to_emdata_ptr_->lane_data_num_; ++i)
      // {
      //   if ((2 + (int32_t)(ehr_to_emdata_ptr_->host_lane_idx_ - i) <= -1) ||
      //       ((int32_t)(2 + (ehr_to_emdata_ptr_->host_lane_idx_ - i)) >= 5))
      //   {
      //     bc::uint32_t ele_start_idx = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.start_index_;
      //     bc::uint32_t total_num = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.total_number_;
      //     for (bc::uint32_t j = ele_start_idx; j < ele_start_idx + total_num; ++j)
      //     {
      //       if (ehr_to_emdata_ptr_->lane_elements_[j].route_left_dist_ / 100.f > max_route_distance)
      //       {
      //         max_route_distance = ehr_to_emdata_ptr_->lane_elements_[j].route_left_dist_ / 100.f;
      //         dest_lane = ::zone::data::em_data::kHostLane - i + ehr_to_emdata_ptr_->host_lane_idx_;
      //         dest_element = 0;
      //         out_of_lane = bc::true_v;
      //       }
      //     }
      //   }
      // }
      bc::bool_t out_of_lane = bc::false_v;
      bc::int32_t host_relat_dest_lane_ = 0;
      for (bc::uint32_t i = 0; i < ehr_to_emdata_ptr_->lane_data_num_; ++i)
      {
        bc::uint32_t ele_start_idx = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.start_index_;
        bc::uint32_t total_num = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.total_number_;
        for (bc::uint32_t j = ele_start_idx; j < ele_start_idx + total_num; ++j)
        {
          if (ehr_to_emdata_ptr_->lane_elements_[j].route_left_dist_ / 100.f > max_route_distance)
          {
            max_route_distance = ehr_to_emdata_ptr_->lane_elements_[j].route_left_dist_ / 100.f;
            dest_lane = ::zone::data::em_data::kHostLane - i + ehr_to_emdata_ptr_->host_lane_idx_;
            int32_t current_lane_idx = 2 + (int32_t)(ehr_to_emdata_ptr_->host_lane_idx_ - i);
            int32_t current_element_relative_idx = j - ele_start_idx;
            if (current_lane_idx > -1 && current_lane_idx < 5)
            {
              if (map_em_data.lanes_[current_lane_idx].lane_elements_[0].element_valid_ &&
                  current_element_relative_idx >= map_em_data.lanes_[current_lane_idx].lane_elements_[0].relative_idx_)
              {
                dest_element = 0;
                out_of_lane = bc::false_v;
                if (dest_lane == ::zone::data::em_data::kHostLane &&
                    current_element_relative_idx > map_em_data.lanes_[current_lane_idx].lane_elements_[0].relative_idx_)
                {
                  host_relat_dest_lane_ = 1;
                }
              }
              else if (map_em_data.lanes_[current_lane_idx].lane_elements_[1].element_valid_ &&
                       current_element_relative_idx <=
                           map_em_data.lanes_[current_lane_idx].lane_elements_[1].relative_idx_)
              {
                dest_element = 1;
                out_of_lane = bc::false_v;
                if (dest_lane == ::zone::data::em_data::kHostLane &&
                    current_element_relative_idx < map_em_data.lanes_[current_lane_idx].lane_elements_[1].relative_idx_)
                {
                  host_relat_dest_lane_ = -1;
                }
              }
            }
            else
            {
              dest_element = 0;
              out_of_lane = bc::true_v;
            }
          }
        }
      }
      if (dest_lane != kMaxLaneNum && dest_element != kMaxElemNumInOneLane && !out_of_lane)
      {
        map_em_data.lanes_[dest_lane].lane_elements_[dest_element].is_dest_lane_ele_ = bc::true_v;
        map_em_data.relat_dest_lane_ = ::zone::data::em_data::kHostLane - dest_lane;
        if (dest_lane == ::zone::data::em_data::kHostLane &&
            map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].element_valid_ &&
            map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[1].element_valid_ &&
            fabsf(map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_ -
                  map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[1].route_left_dis_) < 0.1f)
        {
          map_em_data.relat_dest_lane_ += host_relat_dest_lane_;
        }
      }
      else if (out_of_lane)
      {
        map_em_data.relat_dest_lane_ = ::zone::data::em_data::kHostLane - dest_lane;
      }
    }
    // // Calculate lanes num that has same route_left_distance with host lane
    // if (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_ > 0.f)
    // {
    //   for (bc::uint32_t i = 0; i < ehr_to_emdata_ptr_->lane_data_num_; ++i)
    //   {
    //     bc::int32_t ele_start_idx = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.start_index_;
    //     bc::int32_t total_num = ehr_to_emdata_ptr_->lane_data_[i].lane_elements_range_.total_number_;
    //     for (bc::uint32_t j = ele_start_idx; j < ele_start_idx + total_num; ++j)
    //     {
    //       if (fabsf((ehr_to_emdata_ptr_->lane_elements_[j].route_left_dist_ / 100.f) -
    //                 map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_) < 0.1f)
    //       {
    //         map_em_data.main_lane_num_++;
    //         break;
    //       }
    //     }
    //   }
    // }
    map_em_data.main_lane_num_ = ehr_to_emdata_ptr_->ego_section_regular_lane_num_;

    // 输出当前自车位置最左和最右的roadedge
    // 默认使用最左车道的最左element的左roadedge作为最左roadedge，最右车道的最右element作为最右roadedge
    int32_t left_most_roadedge_idx =
        ehr_to_emdata_ptr_->lane_data_[ehr_to_emdata_ptr_->lane_data_num_ - 1].lane_elements_range_.start_index_ +
        ehr_to_emdata_ptr_->lane_data_[ehr_to_emdata_ptr_->lane_data_num_ - 1].lane_elements_range_.total_number_ - 1;
    getRoadEdges(ehr_to_emdata_ptr_->lane_elements_[left_most_roadedge_idx].left_road_edge_,
                 map_em_data.left_road_edge_);
    getRoadEdges(ehr_to_emdata_ptr_->lane_elements_[0].right_road_edge_, map_em_data.right_road_edge_);

    // 输出静态障碍物
    getSurfaces(map_em_data);

    // 输出地面箭头
    getLaneMarkings(map_em_data);
  }
  else
  {
    ;
  }
  map_em_data.map_car_position_.timestamp_ = ehr_to_emdata_ptr_->car_position_.timestamp_;
  map_em_data.map_car_position_.confidence_ = ehr_to_emdata_ptr_->car_position_.confidence_;
  map_em_data.map_car_position_.heading_ = ehr_to_emdata_ptr_->car_position_.heading_;
  // now use loc timestamp !
  map_em_data.timestamp_ = ehr_to_emdata_ptr_->car_position_.timestamp_;
  // get MapGuidePoint
  EhrMapGuidePointExtract::GuidePointInfo(map_em_data.map_guide_pts_, *ehr_to_emdata_ptr_);
  // get MapGeoFence
  EhrMapGeofenceExtract::getMapGeofence(map_em_data.map_geofence_, *ehr_to_emdata_ptr_);
}

void EhrDataHandle::getLeftMergeEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data)
{
  /// left
  if (0 != ehr_to_emdata_ptr_->lane_data_left_merge_num_)
  {
    for (int32_t i = 0; i < static_cast<int>(ehr_to_emdata_ptr_->lane_data_left_merge_num_); i++)
    {
      const zone::common::EhrToEmDataRange& lane_elements_range =
          ehr_to_emdata_ptr_->lane_data_left_merge_[i].lane_elements_range_;
      if (lane_elements_range.total_number_ > 0)
      {
        bc::uint8_t lane_idx_u8 = 5;
        for (bc::int8_t t_idx_s8 = 1; t_idx_s8 >= 0; t_idx_s8--)
        {
          if (!map_em_data.lanes_[t_idx_s8].lane_valid_)
          {
            lane_idx_u8 = t_idx_s8;
            break;
          }
        }
        if (lane_idx_u8 > 1)
        {
          return;
        }
        map_em_data.lanes_[lane_idx_u8].lane_valid_ = bc::true_v;
        auto& em_laneElements = map_em_data.lanes_[lane_idx_u8].lane_elements_;
        getLaneElements(lane_elements_range, em_laneElements, map_em_data.nearst_split_scenario_, 1);
        auto& t_lanes = map_em_data.lanes_[lane_idx_u8];
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].reference_line_.current_point_idx_ > (kMaxRefLinePtsNum - 2))
            t_lanes.lane_elements_[k].element_valid_ = bc::false_v;
        }
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].element_valid_)
          {
            t_lanes.valid_elements_cnts_++;
            em_laneElements[k].merge_flag_ = bc::true_v;
          }
        }
        if (map_em_data.lanes_[lane_idx_u8].valid_elements_cnts_ == 0)
        {
          map_em_data.lanes_[lane_idx_u8].lane_valid_ = bc::false_v;
        }
      }
    }
  }
}
void EhrDataHandle::getRightMergeEhrToEmData(zone::data::em_data::EmAdapterHDmapData& map_em_data)
{
  /// right
  if (0 != ehr_to_emdata_ptr_->lane_data_right_merge_num_)
  {

    for (int32_t i = static_cast<int>(ehr_to_emdata_ptr_->lane_data_right_merge_num_); i > 0; i--)
    {
      const zone::common::EhrToEmDataRange& lane_elements_range =
          ehr_to_emdata_ptr_->lane_data_right_merge_[i - 1].lane_elements_range_;
      if (lane_elements_range.total_number_ > 0)
      {
        bc::uint8_t lane_idx_u8 = 5;
        for (bc::uint8_t t_idx_u8 = 3; t_idx_u8 < 5; t_idx_u8++)
        {
          if (!map_em_data.lanes_[t_idx_u8].lane_valid_)
          {
            lane_idx_u8 = t_idx_u8;
            break;
          }
        }
        if (lane_idx_u8 < 3 || lane_idx_u8 > 4)
        {
          return;
        }
        map_em_data.lanes_[lane_idx_u8].lane_valid_ = bc::true_v;
        auto& em_laneElements = map_em_data.lanes_[lane_idx_u8].lane_elements_;
        getLaneElements(lane_elements_range, em_laneElements, map_em_data.nearst_split_scenario_, 1);
        auto& t_lanes = map_em_data.lanes_[lane_idx_u8];
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].reference_line_.current_point_idx_ > (kMaxRefLinePtsNum - 2))
            t_lanes.lane_elements_[k].element_valid_ = bc::false_v;
        }
        for (int k = 0; k < kMaxElemNumInOneLane; k++)
        {
          if (t_lanes.lane_elements_[k].element_valid_)
          {
            t_lanes.valid_elements_cnts_++;
            em_laneElements[k].merge_flag_ = bc::true_v;
          }
        }
        if (map_em_data.lanes_[lane_idx_u8].valid_elements_cnts_ == 0)
        {
          map_em_data.lanes_[lane_idx_u8].lane_valid_ = bc::false_v;
        }
      }
    }
  }
}

bc::bool_t EhrDataHandle::getDestLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range)
{
  int32_t start_index = lane_elements_range.start_index_;
  zone::common::EhrToEmLaneElement lane_elements[lane_elements_range.total_number_];
  for (int32_t i = 0; i < lane_elements_range.total_number_; i++)
  {
    lane_elements[i] = ehr_to_emdata_ptr_->lane_elements_[start_index + i];
    if (lane_elements[i].is_dest_lane_element_) return bc::true_v;
  }
  return bc::false_v;
}

bc::bool_t EhrDataHandle::getDestLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range,
                                              bc::int32_t& dest_idx)
{
  int32_t start_index = lane_elements_range.start_index_;
  zone::common::EhrToEmLaneElement lane_elements[lane_elements_range.total_number_];
  for (int32_t i = 0; i < lane_elements_range.total_number_; i++)
  {
    lane_elements[i] = ehr_to_emdata_ptr_->lane_elements_[start_index + i];
    if (lane_elements[i].is_dest_lane_element_)
    {
      dest_idx = i;
      return bc::true_v;
    }
  }
  return bc::false_v;
}

void EhrDataHandle::getLaneElements(const zone::common::EhrToEmDataRange& lane_elements_range,
                                    bc::TCArray<LaneElement, kMaxElemNumInOneLane>& em_lane_elements,
                                    zone::data::em_data::RefLineSegLaneTransitionDir& split_scenario, int8_t lane_flag)
{
  int32_t start_index = lane_elements_range.start_index_;
  // int32_t total_number = lane_elements_range.total_number_;
  zone::common::EhrToEmLaneElement lane_elements[lane_elements_range.total_number_];

  LaneElement tmp_em_lane_elements[lane_elements_range.total_number_];
  bc::float32_t terminated_offset_ar[lane_elements_range.total_number_];
  for (int32_t i = 0; i < lane_elements_range.total_number_; i++)
  {
    lane_elements[i] = ehr_to_emdata_ptr_->lane_elements_[start_index + i];
    // centerLine <-> reference_line other profiles @ ehrprofileextratract class
    RhrProfileExtract::getEhrLaneProfile(lane_elements[i].center_line_, tmp_em_lane_elements[i].reference_line_,
                                         *ehr_to_emdata_ptr_);
    // Find tollbooth or service zone start_offset
    terminated_offset_ar[i] = 500.f;
    for (bc::uint8_t seg_idx = 0; seg_idx < kMaxSpecialSituationSegsInOneLaneElement; seg_idx++)
    {
      if (tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].valid_ &&
          (tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].special_situation_ ==
               zone::data::em_data::SpecialSituation::kTollBooth ||
           tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].special_situation_ ==
               zone::data::em_data::SpecialSituation::kLinkSapaEnter) &&
          tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].start_s_ > 0.f &&
          tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].start_s_ < 200.f)
      {
        terminated_offset_ar[i] = tmp_em_lane_elements[i].reference_line_.special_segs_[seg_idx].start_s_;
        break;
      }
    }
    getCenterLines(lane_elements[i], tmp_em_lane_elements[i], terminated_offset_ar[i]);
    // others lane element profile
    tmp_em_lane_elements[i].relative_idx_ = i;
    tmp_em_lane_elements[i].mark_direction_ =
        static_cast<bc::uint8_t>(tmp_em_lane_elements[i].reference_line_.arrow_segs_[0].arrow_type_);
    tmp_em_lane_elements[i].lane_type_ = tmp_em_lane_elements[i].reference_line_.lane_type_segs_[0].lane_type_;
    for (uint32_t j = 0; j < kMaxLanePropertySegsInOneLaneElement; ++j)
    {
      if (tmp_em_lane_elements[i].reference_line_.lane_type_segs_[j].valid_ &&
          tmp_em_lane_elements[i].reference_line_.lane_type_segs_[j].start_s_ <= 0.f &&
          tmp_em_lane_elements[i].reference_line_.lane_type_segs_[j].end_s_ >= 0.f)
      {
        tmp_em_lane_elements[i].lane_type_ = tmp_em_lane_elements[i].reference_line_.lane_type_segs_[j].lane_type_;
        break;
      }
    }
    tmp_em_lane_elements[i].route_left_dis_ = lane_elements[i].route_left_dist_ / 100.f;
    tmp_em_lane_elements[i].is_dest_lane_ele_ = lane_elements[i].is_dest_lane_element_;
    tmp_em_lane_elements[i].element_valid_ = bc::true_v;
    tmp_em_lane_elements[i].element_id_ = lane_elements[i].element_id_;
    tmp_em_lane_elements[i].recommend_level_ = lane_elements[i].preference_degree_;
    tmp_em_lane_elements[i].direction_to_dest_ = lane_elements[i].direction_to_dest_;
  }
  /// TODO: Set relative left element to index 0 and right to index 1 if not using SplitMapLaneData anymore
  // Choose appropriate elements to the selected lane distinguished by the lane position
  if (0 == lane_flag)
  {
    if (lane_elements_range.total_number_ >= kMaxElemNumInOneLane)
    {
      // Always find the longest route_left_dis element to use as the target element
      // If tie elements exist, use relative right element or last cycle same id element as the target element
      // Always find the min element id as current default continue element
      bc::int32_t t_on_route_idx = 0;
      bc::int32_t t_continue_idx = 0;
      bc::uint16_t t_min_id = 0;
      bc::uint8_t t_total_on_route_num = 0;
      bc::uint8_t t_total_continue_num = 0;
      for (bc::int32_t i = 0; i < lane_elements_range.total_number_; i++)
      {
        if (tmp_em_lane_elements[i].is_dest_lane_ele_)
        {
          t_on_route_idx = i;
          t_total_on_route_num++;
        }
        // if (tmp_em_lane_elements[i].element_id_ > 0 &&
        //     (t_min_id == 0 || t_min_id > tmp_em_lane_elements[i].element_id_))
        // {
        //   t_min_id = tmp_em_lane_elements[i].element_id_;
        //   t_continue_idx = i;
        // }
        if (lane_elements[i].is_all_continue_)
        {
          if (t_total_continue_num > 0)
          {
            if (tmp_em_lane_elements[i].element_id_ > 0 &&
                (t_min_id == 0 || t_min_id > tmp_em_lane_elements[i].element_id_))
            {
              t_min_id = tmp_em_lane_elements[i].element_id_;
              t_continue_idx = i;
            }
          }
          else
          {
            t_min_id = tmp_em_lane_elements[i].element_id_;
            t_continue_idx = i;
          }
          t_total_continue_num++;
        }
      }
      if (t_total_on_route_num != 1)
      {
        bc::uint8_t t_longest_on_route_idx = 0;
        bc::float32_t t_longest_on_route = 0.f;
        for (bc::int32_t i = 0; i < lane_elements_range.total_number_; i++)
        {
          if (tmp_em_lane_elements[i].route_left_dis_ > t_longest_on_route)
          {
            t_longest_on_route = tmp_em_lane_elements[i].route_left_dis_;
            t_longest_on_route_idx = i;
          }
          else if (fabsf(tmp_em_lane_elements[i].route_left_dis_ - t_longest_on_route) < 0.1f &&
                   tmp_em_lane_elements[i].element_id_ == last_cycle_on_route_id)
          {
            t_longest_on_route_idx = i;
          }
        }
        t_on_route_idx = t_longest_on_route_idx;
      }
      last_cycle_on_route_id = tmp_em_lane_elements[t_on_route_idx].element_id_;
      // Special case that no element is continue element
      if (t_total_continue_num == 0)
      {
        t_continue_idx = t_on_route_idx;
      }

      // Split point related with the on route element
      if (t_on_route_idx == t_continue_idx)
      {
        // Find the nearest lane split point ahead of the ego car, as well as the related element
        bc::float32_t t_split_start_offset = 0.f;
        LaneTransitionDirection t_lane_transition_type = LaneTransitionDirection::kUnknown;
        bc::uint8_t t_split_element_idx = 0;
        bc::bool_t t_find_split_point = bc::false_v;
        for (int32_t i = 0; i < lane_elements_range.total_number_; i++)
        {
          for (bc::uint8_t j = 0; j < kMaxLaneTransitionSegsInOneLaneElement; j++)
          {
            bc::float32_t& t_current_start_offset =
                tmp_em_lane_elements[i].reference_line_.lane_transition_dir_segs_[j].start_s_;
            LaneTransitionDirection& t_current_transition =
                tmp_em_lane_elements[i].reference_line_.lane_transition_dir_segs_[j].lane_trans_dir_;
            if (tmp_em_lane_elements[i].reference_line_.lane_transition_dir_segs_[j].valid_ &&

                t_current_start_offset > 0.f && (t_current_transition == LaneTransitionDirection::kSplitToLeft ||
                                                 t_current_transition == LaneTransitionDirection::kSplitToRight))

            {
              if (!t_find_split_point || (t_find_split_point && t_split_start_offset > t_current_start_offset))
              {
                t_split_start_offset = t_current_start_offset;
                t_lane_transition_type = t_current_transition;
                t_split_element_idx = i;
                t_find_split_point = bc::true_v;
              }
              break;
            }
          }
        }
        if (t_lane_transition_type == LaneTransitionDirection::kSplitToLeft)
        {
          getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
                                    tmp_em_lane_elements[t_on_route_idx], terminated_offset_ar[t_on_route_idx]);
          // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
          //                       tmp_em_lane_elements[t_on_route_idx]);
          em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
          // Imposible case indicates that the most left element has a split point to the left
          if (t_on_route_idx == lane_elements_range.total_number_ - 1)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx - 1],
                                      tmp_em_lane_elements[t_on_route_idx - 1],
                                      terminated_offset_ar[t_on_route_idx - 1]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx - 1],
            //                       tmp_em_lane_elements[t_on_route_idx - 1]);
            em_lane_elements[1] = tmp_em_lane_elements[t_on_route_idx - 1];
          }
          // Reasonable and ordinary case
          else
          {
            getLaneBoundarysBySegment(
                ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 1],
                tmp_em_lane_elements[lane_elements_range.total_number_ - 1],
                terminated_offset_ar[lane_elements_range.total_number_ - 1]);
            // getRoadEdgesBySegment(
            //     ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 1],
            //     tmp_em_lane_elements[lane_elements_range.total_number_ - 1]);
            em_lane_elements[1] = tmp_em_lane_elements[lane_elements_range.total_number_ - 1];
          }
        }
        else if (t_lane_transition_type == LaneTransitionDirection::kSplitToRight)
        {
          getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
                                    tmp_em_lane_elements[t_on_route_idx], terminated_offset_ar[t_on_route_idx]);
          // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
          //                       tmp_em_lane_elements[t_on_route_idx]);
          em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
          // Imposible case indicates that the most right element has a split point to the right
          if (t_on_route_idx == 0)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx + 1],
                                      tmp_em_lane_elements[t_on_route_idx + 1],
                                      terminated_offset_ar[t_on_route_idx + 1]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx + 1],
            //                       tmp_em_lane_elements[t_on_route_idx + 1]);
            em_lane_elements[1] = tmp_em_lane_elements[t_on_route_idx + 1];
          }
          // Reasonable and ordinary case
          else
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                      terminated_offset_ar[0]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
            em_lane_elements[1] = tmp_em_lane_elements[0];
          }
        }
        // Imposible case indicates that muti-elements lane does not has a split point ahead of the ego car
        else
        {
          getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
                                    tmp_em_lane_elements[t_on_route_idx], terminated_offset_ar[t_on_route_idx]);
          // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
          //                       tmp_em_lane_elements[t_on_route_idx]);
          em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
          if (t_on_route_idx != 0)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                      terminated_offset_ar[0]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
            em_lane_elements[1] = tmp_em_lane_elements[0];
          }
          else
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1],
                                      terminated_offset_ar[1]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1]);
            em_lane_elements[1] = tmp_em_lane_elements[1];
          }
        }
      }
      else
      {
        // // Find the closest element between continue and on route element
        // if (t_continue_idx >= 0 && t_on_route_idx > t_continue_idx + 1 &&
        //     lane_elements[t_on_route_idx].element_id_ > lane_elements[t_continue_idx + 1].element_id_)
        // {
        //   t_on_route_idx = t_continue_idx + 1;
        // }
        // else if (t_continue_idx >= 1 && t_on_route_idx < t_continue_idx - 1 &&
        //          lane_elements[t_on_route_idx].element_id_ > lane_elements[t_continue_idx - 1].element_id_)
        // {
        //   t_on_route_idx = t_continue_idx - 1;
        // }
        // // Find the corelated transition element id
        // bc::bool_t find_related_on_route_element = bc::false_v;
        // uint32_t continue_cl_start_data_index =
        //     lane_elements[t_continue_idx].center_line_.lane_transition_direction_range_.start_index_;
        // uint32_t continue_cl_total_data_number =
        //     lane_elements[t_continue_idx].center_line_.lane_transition_direction_range_.total_number_;
        // LaneTransitionDirection target_lane_transition = LaneTransitionDirection::kUnknown;
        // for (uint32_t range_idx = 0; range_idx < continue_cl_total_data_number; range_idx++)
        // {
        //   zone::common::EhrToEmDataRange lane_transition_direction_range =
        //       ehr_to_emdata_ptr_->lane_elements_data_idx_[continue_cl_start_data_index + range_idx];
        //   uint32_t lane_transition_direction_start_index = lane_transition_direction_range.start_index_;
        //   uint32_t lane_transition_direction_total_number = lane_transition_direction_range.total_number_;
        //   for (uint32_t i = 0; i < lane_transition_direction_total_number; i++)
        //   {
        //     if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //             .related_element_id_ == tmp_em_lane_elements[t_on_route_idx].element_id_)
        //     {
        //       if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //               .lane_transition_direction_ ==
        //           zone::common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_LEFT)
        //       {
        //         target_lane_transition = LaneTransitionDirection::kSplitToLeft;
        //         split_scenario.lane_trans_dir_ = target_lane_transition;
        //         split_scenario.start_s_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .start_offset_ /
        //             100.f;
        //         split_scenario.end_s_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .end_offset_ /
        //             100.f;
        //         split_scenario.related_element_id_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .related_element_id_;
        //         split_scenario.valid_ = bc::true_v;
        //       }
        //       else if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                    .lane_transition_direction_ ==
        //                zone::common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_RIGHT)
        //       {
        //         target_lane_transition = LaneTransitionDirection::kSplitToRight;
        //         split_scenario.lane_trans_dir_ = target_lane_transition;
        //         split_scenario.start_s_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .start_offset_ /
        //             100.f;
        //         split_scenario.end_s_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .end_offset_ /
        //             100.f;
        //         split_scenario.related_element_id_ =
        //             ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
        //                 .related_element_id_;
        //         split_scenario.valid_ = bc::true_v;
        //       }
        //       else
        //       {
        //         std::cout << "Error occurs in EhrDataHandle::getLaneElements: Not find corrcet transition!!!"
        //                   << std::endl;
        //       }
        //       find_related_on_route_element = bc::true_v;
        //       break;
        //     }
        //   }
        // }
        // if (!find_related_on_route_element)
        // {
        //   std::cout << "Error occurs in EhrDataHandle::getLaneElements: Not find related on route element!!!"
        //             << std::endl;
        // }
        LaneTransitionDirection target_lane_transition = LaneTransitionDirection::kUnknown;
        bc::int32_t t_target_idx = t_on_route_idx;
        if (!findRelatedOnRouteElement(lane_elements[t_on_route_idx], lane_elements[t_continue_idx], split_scenario,
                                       target_lane_transition))
        {
#ifdef STD_COUT_ENABLE
          std::cout << "Error occurs in EhrDataHandle::getLaneElements: Not find related on route element!!!"
                    << std::endl;
#endif
        }
        else
        {
          for (bc::int32_t element_idx = 0; element_idx < lane_elements_range.total_number_; ++element_idx)
          {
            if (tmp_em_lane_elements[element_idx].element_id_ == split_scenario.related_element_id_)
            {
              t_target_idx = element_idx;
              break;
            }
          }
        }
        if (target_lane_transition == LaneTransitionDirection::kSplitToLeft ||
            target_lane_transition == LaneTransitionDirection::kSplitToRight)
        {
          getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_continue_idx],
                                    tmp_em_lane_elements[t_continue_idx], terminated_offset_ar[t_continue_idx]);
          // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_continue_idx],
          //                       tmp_em_lane_elements[t_continue_idx]);
          em_lane_elements[0] = tmp_em_lane_elements[t_continue_idx];
          // Continue element is on the left of on route element
          if (t_continue_idx > t_on_route_idx + 1)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_target_idx],
                                      tmp_em_lane_elements[t_target_idx], terminated_offset_ar[t_target_idx]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_target_idx],
            //                       tmp_em_lane_elements[t_target_idx]);
            em_lane_elements[1] = tmp_em_lane_elements[t_target_idx];
          }
          // Continue element is on the right of on route element
          else if (t_on_route_idx > 1 && t_continue_idx < t_on_route_idx - 1)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_target_idx],
                                      tmp_em_lane_elements[t_target_idx], terminated_offset_ar[t_target_idx]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_target_idx],
            //                       tmp_em_lane_elements[t_target_idx]);
            em_lane_elements[1] = tmp_em_lane_elements[t_target_idx];
          }
          else
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
                                      tmp_em_lane_elements[t_on_route_idx], terminated_offset_ar[t_on_route_idx]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
            //                       tmp_em_lane_elements[t_on_route_idx]);
            em_lane_elements[1] = tmp_em_lane_elements[t_on_route_idx];
          }
        }
        // Imposible case indicates that muti-elements lane does not has a split point ahead of the ego car
        else
        {
          getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
                                    tmp_em_lane_elements[t_on_route_idx], terminated_offset_ar[t_on_route_idx]);
          // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
          //                       tmp_em_lane_elements[t_on_route_idx]);
          em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
          if (t_on_route_idx != 0)
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                      terminated_offset_ar[0]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
            em_lane_elements[1] = tmp_em_lane_elements[0];
          }
          else
          {
            getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1],
                                      terminated_offset_ar[1]);
            // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1]);
            em_lane_elements[1] = tmp_em_lane_elements[1];
          }
        }
      }
      // // Split element is on the left side of on route element
      // else if (t_on_route_idx < t_split_element_idx)
      // {
      //   // Reasonable and ordinary case
      //   if (t_lane_transition_type == LaneTransitionDirection::kSplitToRight)
      //   {
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_split_element_idx],
      //                               tmp_em_lane_elements[t_split_element_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_split_element_idx],
      //                           tmp_em_lane_elements[t_split_element_idx]);
      //     em_lane_elements[0] = tmp_em_lane_elements[t_split_element_idx];
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                               tmp_em_lane_elements[t_on_route_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                           tmp_em_lane_elements[t_on_route_idx]);
      //     em_lane_elements[1] = tmp_em_lane_elements[t_on_route_idx];
      //   }
      //   // Imposible case indicates that the on route element is not related with the split element
      //   else
      //   {
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                               tmp_em_lane_elements[t_on_route_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                           tmp_em_lane_elements[t_on_route_idx]);
      //     em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
      //   }
      // }
      // // Split element is on the right side of on route element
      // else
      // {
      //   // Reasonable and ordinary case
      //   if (t_lane_transition_type == LaneTransitionDirection::kSplitToLeft)
      //   {
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_split_element_idx],
      //                               tmp_em_lane_elements[t_split_element_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_split_element_idx],
      //                           tmp_em_lane_elements[t_split_element_idx]);
      //     em_lane_elements[0] = tmp_em_lane_elements[t_split_element_idx];
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                               tmp_em_lane_elements[t_on_route_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                           tmp_em_lane_elements[t_on_route_idx]);
      //     em_lane_elements[1] = tmp_em_lane_elements[t_on_route_idx];
      //   }
      //   // Imposible case indicates that the on route element is not related with the split element
      //   else
      //   {
      //     getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                               tmp_em_lane_elements[t_on_route_idx]);
      //     getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + t_on_route_idx],
      //                           tmp_em_lane_elements[t_on_route_idx]);
      //     em_lane_elements[0] = tmp_em_lane_elements[t_on_route_idx];
      //   }
      // }
    }
    else
    {
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                terminated_offset_ar[0]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
      em_lane_elements[0] = tmp_em_lane_elements[0];
      last_cycle_on_route_id = tmp_em_lane_elements[0].element_id_;
    }
  }
  /// TODO: Use left or right most element to set on left/right lane or
  /// take longest route_left_dis element into consideration
  else if (lane_flag > 0)
  {
    if (lane_elements_range.total_number_ >= kMaxElemNumInOneLane)
    {
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 1],
                                tmp_em_lane_elements[lane_elements_range.total_number_ - 1],
                                terminated_offset_ar[lane_elements_range.total_number_ - 1]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 1],
      //                       tmp_em_lane_elements[lane_elements_range.total_number_ - 1]);
      em_lane_elements[0] = tmp_em_lane_elements[lane_elements_range.total_number_ - 1];
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 2],
                                tmp_em_lane_elements[lane_elements_range.total_number_ - 2],
                                terminated_offset_ar[lane_elements_range.total_number_ - 2]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + lane_elements_range.total_number_ - 2],
      //                       tmp_em_lane_elements[lane_elements_range.total_number_ - 2]);
      em_lane_elements[1] = tmp_em_lane_elements[lane_elements_range.total_number_ - 2];
      bc::bool_t dest_lane_out_of_range = bc::false_v;
      for (bc::int32_t i = lane_elements_range.total_number_ - 1 - kMaxElemNumInOneLane; i >= 0; i--)
      {
        if (tmp_em_lane_elements[i].is_dest_lane_ele_)
        {
          dest_lane_out_of_range = bc::true_v;
          break;
        }
      }
      if (!em_lane_elements[0].is_dest_lane_ele_ && !em_lane_elements[1].is_dest_lane_ele_ && dest_lane_out_of_range)
      {
        em_lane_elements[1].is_dest_lane_ele_ = bc::true_v;
      }
    }
    else
    {
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                terminated_offset_ar[0]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
      em_lane_elements[0] = tmp_em_lane_elements[0];
    }
  }
  else if (lane_flag < 0)
  {
    if (lane_elements_range.total_number_ >= kMaxElemNumInOneLane)
    {
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1],
                                terminated_offset_ar[1]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index + 1], tmp_em_lane_elements[1]);
      em_lane_elements[1] = tmp_em_lane_elements[1];
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                terminated_offset_ar[0]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
      em_lane_elements[0] = tmp_em_lane_elements[0];
      bc::bool_t dest_lane_out_of_range = bc::false_v;
      for (bc::int32_t i = kMaxElemNumInOneLane; i < lane_elements_range.total_number_; i++)
      {
        if (tmp_em_lane_elements[i].is_dest_lane_ele_)
        {
          dest_lane_out_of_range = bc::true_v;
          break;
        }
      }
      if (!em_lane_elements[0].is_dest_lane_ele_ && !em_lane_elements[1].is_dest_lane_ele_ && dest_lane_out_of_range)
      {
        em_lane_elements[1].is_dest_lane_ele_ = bc::true_v;
      }
    }
    else
    {
      getLaneBoundarysBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0],
                                terminated_offset_ar[0]);
      // getRoadEdgesBySegment(ehr_to_emdata_ptr_->lane_elements_[start_index], tmp_em_lane_elements[0]);
      em_lane_elements[0] = tmp_em_lane_elements[0];
    }
  }
}

bc::bool_t EhrDataHandle::findRelatedOnRouteElement(const zone::common::EhrToEmLaneElement& on_route_element,
                                                    const zone::common::EhrToEmLaneElement& continue_element,
                                                    zone::data::em_data::RefLineSegLaneTransitionDir& split_scenario,
                                                    LaneTransitionDirection& target_lane_transition)
{
  int32_t continue_start_offset = -100000;
  uint32_t cl_start_data_index = on_route_element.center_line_.map_pts_range_.start_index_;
  uint32_t continue_cl_start_data_index = continue_element.center_line_.map_pts_range_.start_index_;
  uint32_t max_range_idx = std::min(on_route_element.center_line_.map_pts_range_.total_number_,
                                    continue_element.center_line_.map_pts_range_.total_number_);
  bc::bool_t find_continue_segment = bc::false_v;
  for (uint32_t range_idx = 0; range_idx < max_range_idx; range_idx++)
  {
    if (ehr_to_emdata_ptr_->lane_elements_data_idx_[cl_start_data_index + range_idx].start_index_ !=
            ehr_to_emdata_ptr_->lane_elements_data_idx_[continue_cl_start_data_index + range_idx].start_index_ ||
        ehr_to_emdata_ptr_->lane_elements_data_idx_[cl_start_data_index + range_idx].total_number_ !=
            ehr_to_emdata_ptr_->lane_elements_data_idx_[continue_cl_start_data_index + range_idx].total_number_)
    {
      continue_start_offset =
          ehr_to_emdata_ptr_
              ->pts_offset_[ehr_to_emdata_ptr_->lane_elements_data_idx_[cl_start_data_index + range_idx].start_index_];
      find_continue_segment = bc::true_v;
      break;
    }
  }
  if (!find_continue_segment)
  {
    if (on_route_element.center_line_.map_pts_range_.total_number_ >
        continue_element.center_line_.map_pts_range_.total_number_)
    {
      continue_start_offset =
          ehr_to_emdata_ptr_->pts_offset_
              [ehr_to_emdata_ptr_->lane_elements_data_idx_[cl_start_data_index + max_range_idx].start_index_];
    }
    else if (on_route_element.center_line_.map_pts_range_.total_number_ <
             continue_element.center_line_.map_pts_range_.total_number_)
    {
      continue_start_offset =
          ehr_to_emdata_ptr_->pts_offset_
              [ehr_to_emdata_ptr_->lane_elements_data_idx_[continue_cl_start_data_index + max_range_idx].start_index_];
    }
  }

  uint32_t continue_cl_lane_transition_start_data_index =
      continue_element.center_line_.lane_transition_direction_range_.start_index_;
  for (uint32_t continue_range_idx = 0;
       continue_range_idx < (uint32_t)continue_element.center_line_.lane_transition_direction_range_.total_number_;
       continue_range_idx++)
  {
    zone::common::EhrToEmDataRange lane_transition_direction_range =
        ehr_to_emdata_ptr_->lane_elements_data_idx_[continue_cl_lane_transition_start_data_index + continue_range_idx];
    uint32_t lane_transition_direction_start_index = lane_transition_direction_range.start_index_;
    uint32_t lane_transition_direction_total_number = lane_transition_direction_range.total_number_;
    for (uint32_t i = 0; i < lane_transition_direction_total_number; i++)
    {
      if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i].start_offset_ ==
          continue_start_offset)
      {
        if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
                .lane_transition_direction_ ==
            zone::common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_LEFT)
        {
          target_lane_transition = LaneTransitionDirection::kSplitToLeft;
          split_scenario.lane_trans_dir_ = target_lane_transition;
          split_scenario.start_s_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i].start_offset_ /
              100.f;
          split_scenario.end_s_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i].end_offset_ /
              100.f;
          split_scenario.related_element_id_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
                  .related_element_id_;
          split_scenario.valid_ = bc::true_v;
        }
        else if (ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
                     .lane_transition_direction_ ==
                 zone::common::EEhrToEmLaneTransitionDirection::EHRTOEM_LANE_TRANSITION_DIRECTION_SPLIT_TO_RIGHT)
        {
          target_lane_transition = LaneTransitionDirection::kSplitToRight;
          split_scenario.lane_trans_dir_ = target_lane_transition;
          split_scenario.start_s_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i].start_offset_ /
              100.f;
          split_scenario.end_s_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i].end_offset_ /
              100.f;
          split_scenario.related_element_id_ =
              ehr_to_emdata_ptr_->lane_transition_direction_[lane_transition_direction_start_index + i]
                  .related_element_id_;
          split_scenario.valid_ = bc::true_v;
        }
        else
        {
#ifdef STD_COUT_ENABLE
          std::cout << "Error occurs in EhrDataHandle::findRelatedOnRouteElement: Not find corrcet transition!!!"
                    << std::endl;
#endif
        }
        return bc::true_v;
      }
    }
  }
  return bc::false_v;
}

void EhrDataHandle::getCenterLines(const zone::common::EhrToEmLaneElement& lane_element, LaneElement& em_lane_element,
                                   bc::float32_t terminated_offset)
{
  const bc::int32_t kCenterLinePtsNum = 1000;
  // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_vx;
  // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_vy;
  bc::TCArray<bc::float32_t, kCenterLinePtsNum> point_offsets{};
  bc::TCArray<::zone::common::Point, kCenterLinePtsNum> map_pts;
  bc::uint32_t valid_start_pts = 0;
  bc::uint16_t total_map_pts = 0;
  uint32_t cl_start_data_index = lane_element.center_line_.map_pts_range_.start_index_;
  for (uint32_t range_idx = 0; range_idx < (uint32_t)lane_element.center_line_.map_pts_range_.total_number_;
       range_idx++)
  {
    zone::common::EhrToEmDataRange center_line_pts_range =
        ehr_to_emdata_ptr_->lane_elements_data_idx_[cl_start_data_index + range_idx];
    uint32_t cl_start_index = center_line_pts_range.start_index_;
    uint32_t cl_total_number = center_line_pts_range.total_number_;
    // zone::common::Point3D center_line_pts[cl_total_number];
    for (uint32_t i = 0; i < cl_total_number && i < kCenterLinePtsNum &&
                         (ehr_to_emdata_ptr_->pts_offset_[cl_start_index + i] / 100.f) < terminated_offset;
         i++)
    {
      // Point2Point3D(ehr_to_emdata_ptr_->map_pts_[cl_start_index + i], center_line_pts[i]);
      // center_line_pts_vx.push_back(center_line_pts[i].x);
      // center_line_pts_vy.push_back(center_line_pts[i].y);
      point_offsets[total_map_pts] = ehr_to_emdata_ptr_->pts_offset_[cl_start_index + i] / 100.f;
      map_pts[total_map_pts] = ehr_to_emdata_ptr_->map_pts_[cl_start_index + i];
      if (ehr_to_emdata_ptr_->pts_offset_[cl_start_index + i] <= kValidMapPointOffset)
      {
        valid_start_pts = total_map_pts;
      }
      total_map_pts++;
    }
  }

  // Always add the first index to output as init point
  bc::uint32_t total_size = 0;
  bc::uint32_t clidx = UINT32_MAX;
  bc::float32_t t_x = map_pts[valid_start_pts].x;
  bc::float32_t t_y = map_pts[valid_start_pts].y;
  em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.x = t_x;
  em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.y = t_y;
  em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.z = point_offsets[total_size];
  em_lane_element.reference_line_.ref_line_pts_[total_size].valid_ = bc::true_v;
  total_size++;
  // Judge whether the first point is related with the ego current position
  float32_t min_dis = kMinDistanceFromEgo;
  if ((hypot(t_x, t_y) < min_dis) && (t_x > kLongDistanceBehindEgo) && t_x < kLongDistanceAheadEgo)
  {
    min_dis = hypot(t_x, t_y);
    clidx = valid_start_pts;
  }
  // Choose useful index to generate points
  for (uint32_t i = valid_start_pts + 1; i < total_map_pts && total_size < (uint32_t)kMaxRefLinePtsNum; i++)
  {
    bc::float32_t cur_x = map_pts[i].x;
    bc::float32_t cur_y = map_pts[i].y;
    if ((hypot(t_x - cur_x, t_y - cur_y) < kPointMinDistance))
    {
      continue;
    }
    if ((hypot(cur_x, cur_y) < min_dis) && (cur_x > kLongDistanceBehindEgo) && cur_x < kLongDistanceAheadEgo)
    {
      min_dis = hypot(cur_x, cur_y);
      clidx = total_size;
    }
    t_x = cur_x;
    t_y = cur_y;

    em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.x = cur_x;
    em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.y = cur_y;
    em_lane_element.reference_line_.ref_line_pts_[total_size].pos_.z = point_offsets[i];
    em_lane_element.reference_line_.ref_line_pts_[total_size].valid_ = bc::true_v;
    total_size++;
  }

  // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_rvx;
  // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_rvy;
  // // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_heading;
  // // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> center_line_pts_curvature;
  // // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> clrs;
  // // bc::float64_t clds = 0.0;
  // bc::uint32_t clidx = UINT32_MAX;
  // bc::uint32_t current_idx = 0;
  // bc::TFixedVector<bc::float64_t, kCenterLinePtsNum> r_point_offsets;
  // // EhrAdapterHelper::BSpineInterpolationCenterline(
  // //     point_offsets, r_point_offsets, center_line_pts_vx, center_line_pts_vy, center_line_pts_rvx,
  // //     center_line_pts_rvy,
  // //     clrs, clds, center_line_pts_heading, center_line_pts_curvature, clidx, current_idx);
  // EhrAdapterHelper::BSpineInterpolationCenterline(point_offsets, r_point_offsets, center_line_pts_vx,
  //                                                 center_line_pts_vy, center_line_pts_rvx, center_line_pts_rvy,
  //                                                 clidx,
  //                                                 current_idx);
  // em_lane_element.reference_line_.available_ = bc::true_v;
  // for (uint32_t i = 0; i < center_line_pts_rvy.size() && i < (uint32_t)kMaxRefLinePtsNum; i++)
  // {
  //   em_lane_element.reference_line_.ref_line_pts_[i].pos_.x = center_line_pts_rvx[i];
  //   em_lane_element.reference_line_.ref_line_pts_[i].pos_.y = center_line_pts_rvy[i];
  //   // em_lane_element.reference_line_.ref_line_pts_[i].heading_ = center_line_pts_heading[i];
  //   // em_lane_element.reference_line_.ref_line_pts_[i].curvature_ = center_line_pts_curvature[i];
  //   em_lane_element.reference_line_.ref_line_pts_[i].valid_ = bc::true_v;
  //   // em_lane_element.reference_line_.ref_line_pts_[i].curvature_ / heading_ /
  //   // s_ /
  // }
  // em_lane_element.reference_line_.ref_line_valid_pts_ =
  //     std::min((bc::uint32_t)center_line_pts_rvy.size(), (bc::uint32_t)kMaxRefLinePtsNum);
  em_lane_element.reference_line_.ref_line_valid_pts_ = total_size;
  if (UINT32_MAX != clidx)
  {
    em_lane_element.reference_line_.current_point_idx_ = clidx;
  }
  // The first point is ahead of ego current position
  else if (em_lane_element.reference_line_.ref_line_pts_[0].pos_.x > 0.f)
  {
    em_lane_element.reference_line_.current_point_idx_ = kMaxRefLinePtsNum;
  }
  // The last point is behind of ego current position
  else
  {
    em_lane_element.reference_line_.current_point_idx_ = em_lane_element.reference_line_.ref_line_valid_pts_ - 1;
  }
  if (em_lane_element.reference_line_.ref_line_valid_pts_ > 0)
  {
    em_lane_element.reference_line_.available_ = bc::true_v;
  }
}

void EhrDataHandle::getLaneBoundarys(const zone::common::EhrToEmLaneBoundary& lane_boundary,
                                     LaneBoundary& em_lane_boundary, const bc::int8_t boundary_flag,
                                     bc::float32_t terminated_offset)
{
  const bc::int32_t kMaxBoundarySegNum = 100;
  const bc::int32_t kBoundaryPtsNum = 1000;
  bc::TCArray<::zone::common::EhrToEmLaneBoundarySegment, kMaxBoundarySegNum> boundary_segs;
  bc::TCArray<int32_t, kBoundaryPtsNum> point_offsets;
  bc::TCArray<::zone::common::Point, kBoundaryPtsNum> map_pts;
  if (lane_boundary.existence_)
  {
    // Reorgnize map lane boundary data
    bc::uint8_t total_segs = 0;
    bc::uint16_t total_map_pts = 0;
    int lanbousegstart_data_index = lane_boundary.lane_boundary_segment_range_.start_index_;
    for (int i = 0; i < lane_boundary.lane_boundary_segment_range_.total_number_ && total_segs < kMaxBoundarySegNum;
         i++)
    {
      int lanbousegstart_index =
          ehr_to_emdata_ptr_->lane_elements_data_idx_[lanbousegstart_data_index + i].start_index_;
      int lanbousegtotal_number =
          ehr_to_emdata_ptr_->lane_elements_data_idx_[lanbousegstart_data_index + i].total_number_;
      for (int j = lanbousegstart_index;
           j < lanbousegstart_index + lanbousegtotal_number && total_segs < kMaxBoundarySegNum; j++)
      {
        boundary_segs[total_segs] = ehr_to_emdata_ptr_->lane_boundary_segment_[j];
        total_segs++;
      }
    }
    bc::uint32_t valid_start_pts = 0;
    int ptsstart_data_index = lane_boundary.map_pts_range_.start_index_;
    for (int i = 0; i < lane_boundary.map_pts_range_.total_number_ && total_map_pts < kBoundaryPtsNum; i++)
    {
      int ptsstart_index = ehr_to_emdata_ptr_->lane_elements_data_idx_[ptsstart_data_index + i].start_index_;
      int ptstotal_number = ehr_to_emdata_ptr_->lane_elements_data_idx_[ptsstart_data_index + i].total_number_;
      for (int j = ptsstart_index; j < ptsstart_index + ptstotal_number && total_map_pts < kBoundaryPtsNum &&
                                   (ehr_to_emdata_ptr_->pts_offset_[j] / 100.f) < terminated_offset;
           j++)
      {
        point_offsets[total_map_pts] = ehr_to_emdata_ptr_->pts_offset_[j];
        map_pts[total_map_pts] = ehr_to_emdata_ptr_->map_pts_[j];
        if (point_offsets[total_map_pts] <= kValidMapPointOffset)
        {
          valid_start_pts = total_map_pts;
        }
        total_map_pts++;
      }
    }
    int valid_start_seg = 0;
    for (int i = 0; i < total_segs && boundary_segs[i].end_offset_ <= kValidMapSegmentOffset; i++)
    {
      valid_start_seg = i;
    }
    // Select valid boundary segments
    bc::uint8_t seg_idx = 0;
    ::zone::data::em_data::LaneBoundaryType last_boundary_type = ::zone::data::em_data::LaneBoundaryType::kUnknown;
    ::zone::data::em_data::LaneBoundaryType current_boundary_type = ::zone::data::em_data::LaneBoundaryType::kUnknown;
    for (int i = valid_start_seg; i < total_segs && seg_idx < kMaxBoundarySegment; i++)
    {
      Ehr2EmEnum::LaneBoundaryTypeConvert(boundary_segs[i].lane_boudary_type_, current_boundary_type);
      if (seg_idx == 0 || last_boundary_type != current_boundary_type ||
          fabsf(em_lane_boundary.segs_[seg_idx - 1].end_s_ - boundary_segs[i].start_offset_ / (100.f)) > 1.f)
      {
        em_lane_boundary.segs_[seg_idx].start_s_ = boundary_segs[i].start_offset_ / (100.f);
        em_lane_boundary.segs_[seg_idx].end_s_ = boundary_segs[i].end_offset_ / (100.f);
        Ehr2EmEnum::LaneBoundaryColorConvert(boundary_segs[i].lane_boudary_color_,
                                             em_lane_boundary.segs_[seg_idx].color_type_);
        em_lane_boundary.segs_[seg_idx].boundary_type_ = current_boundary_type;
        em_lane_boundary.segs_[seg_idx].valid_ = bc::true_v;
        last_boundary_type = current_boundary_type;
        seg_idx++;
      }
      else
      {
        em_lane_boundary.segs_[seg_idx - 1].end_s_ = boundary_segs[i].end_offset_ / (100.f);
      }
    }

    // Always add the first index to output as init point
    bc::uint32_t total_size = 0;
    bc::uint32_t idx = UINT32_MAX;
    bc::float32_t t_x = map_pts[valid_start_pts].x;
    bc::float32_t t_y = map_pts[valid_start_pts].y;
    em_lane_boundary.pts_[total_size].x = t_x;
    em_lane_boundary.pts_[total_size].y = t_y;
    total_size++;
    // Judge whether the first point is related with the ego current position
    float32_t min_dis = kMinDistanceFromEgo;
    if ((hypot(t_x, t_y) < min_dis) && (t_x > kLongDistanceBehindEgo) && t_x < kLongDistanceAheadEgo)
    {
      min_dis = hypot(t_x, t_y);
      idx = valid_start_pts;
    }
    // Choose useful index to generate points
    for (uint32_t i = valid_start_pts + 1; i < total_map_pts && total_size < (uint32_t)kMaxBoundaryPoint; i++)
    {
      bc::float32_t cur_x = map_pts[i].x;
      bc::float32_t cur_y = map_pts[i].y;
      if ((hypot(t_x - cur_x, t_y - cur_y) < kPointMinDistance))
      {
        continue;
      }
      if ((hypot(cur_x, cur_y) < min_dis) && (cur_x > kLongDistanceBehindEgo) && cur_x < kLongDistanceAheadEgo)
      {
        min_dis = hypot(cur_x, cur_y);
        idx = total_size;
      }
      t_x = cur_x;
      t_y = cur_y;

      em_lane_boundary.pts_[total_size].x = cur_x;
      em_lane_boundary.pts_[total_size].y = cur_y;
      total_size++;
    }
    // // Choose useful segment index to generate points
    // int total_size = 0;
    // bc::uint32_t idx = UINT32_MAX;
    // bc::uint32_t current_idx = 0;
    // for (int i = valid_start_seg; i < total_segs; i++)
    // {
    //   std::vector<zone::common::Point> point_listsegment;
    //   geographic_transform::CGeographicGeometry::SplitVehiclePoints(
    //       &(point_offsets[0]), &(map_pts[0]), static_cast<uint32_t>(total_map_pts), boundary_segs[i].start_offset_,
    //       boundary_segs[i].end_offset_, point_listsegment);

    //   bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> boundary_pts_vx;
    //   bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> boundary_pts_vy;
    //   for (uint32_t i = 0; i < point_listsegment.size() && i < kBoundaryPtsNum; i++)
    //   {
    //     boundary_pts_vx.push_back(point_listsegment[i].x);
    //     boundary_pts_vy.push_back(point_listsegment[i].y);
    //   }
    //   bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> boundary_pts_rvx;
    //   bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> boundary_pts_rvy;
    //   // bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> line_pts_heading;
    //   // bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> line_pts_curvature;
    //   // bc::TFixedVector<bc::float64_t, kBoundaryPtsNum> rrs;
    //   // bc::float64_t rds = 0.0;
    //   // EhrAdapterHelper::BSpineInterpolation(boundary_pts_vx, boundary_pts_vy, boundary_pts_rvx,
    //   //                                       boundary_pts_rvy, rrs, rds, line_pts_heading, line_pts_curvature,
    //   //                                       idx, current_idx);
    //   EhrAdapterHelper::BSpineInterpolation(boundary_pts_vx, boundary_pts_vy, boundary_pts_rvx, boundary_pts_rvy,
    //   idx,
    //                                         current_idx);
    //   for (uint32_t i = 0; i < boundary_pts_rvy.size() && i < (uint32_t)kMaxBoundaryPoint; i++)
    //   {
    //     em_lane_boundary.pts_[total_size].x = boundary_pts_rvx[i];
    //     em_lane_boundary.pts_[total_size].y = boundary_pts_rvy[i];
    //     total_size++;
    //     if (total_size >= kMaxBoundaryPoint) break;
    //   }
    //   if (total_size >= kMaxBoundaryPoint) break;
    //   // if (boundary_pts_rvx[boundary_pts_rvx.size() - 1] > 0.f && idx == UINT32_MAX) {
    //   //   std::cout << "Error here" << std::endl;
    //   // }
    // }
    if (idx != UINT32_MAX)
    {
      em_lane_boundary.ego_point_idx_ = idx;
    }
    // The first point is ahead of ego current position
    else if (em_lane_boundary.pts_[0].x > 0.f)
    {
      em_lane_boundary.ego_point_idx_ = kMaxBoundaryPoint;
    }
    // The last point is behind of ego current position
    else
    {
      em_lane_boundary.ego_point_idx_ = total_size - 1;
    }
    em_lane_boundary.total_valid_number_ = total_size;
    if (em_lane_boundary.total_valid_number_ > 0)
    {
      em_lane_boundary.existence_ = bc::true_v;
    }
  }
}

void EhrDataHandle::getRoadEdges(const zone::common::EhrToEmRoadEdge& lane_roadedge, RoadEdge& em_lane_roadedge)
{
  const bc::int32_t kMaxRoadedgeSegNum = 100;
  const bc::int32_t kRoadedgePtsNum = 1000;
  bc::TCArray<::zone::common::EhrToEmRoadEdgeSegment, kMaxRoadedgeSegNum> roadedge_segs;
  bc::TCArray<int32_t, kRoadedgePtsNum> point_offsets;
  bc::TCArray<::zone::common::Point, kRoadedgePtsNum> map_pts;
  if (lane_roadedge.existence_)
  {
    // Reorgnize map road edge data
    bc::uint8_t total_segs = 0;
    bc::uint16_t total_map_pts = 0;
    int lanbousegstart_data_index = lane_roadedge.road_edge_segment_range_.start_index_;
    for (int i = 0; i < lane_roadedge.road_edge_segment_range_.total_number_ && total_segs < kMaxRoadedgeSegNum; i++)
    {
      int lanbousegstart_index =
          ehr_to_emdata_ptr_->lane_elements_data_idx_[lanbousegstart_data_index + i].start_index_;
      int lanbousegtotal_number =
          ehr_to_emdata_ptr_->lane_elements_data_idx_[lanbousegstart_data_index + i].total_number_;
      for (int j = lanbousegstart_index;
           j < lanbousegstart_index + lanbousegtotal_number && total_segs < kMaxRoadedgeSegNum; j++)
      {
        roadedge_segs[total_segs] = ehr_to_emdata_ptr_->road_edge_segment_[j];
        total_segs++;
      }
    }
    bc::uint32_t valid_start_pts = 0;
    int ptsstart_data_index = lane_roadedge.map_pts_range_.start_index_;
    for (int i = 0; i < lane_roadedge.map_pts_range_.total_number_ && total_map_pts < kRoadedgePtsNum; i++)
    {
      int ptsstart_index = ehr_to_emdata_ptr_->lane_elements_data_idx_[ptsstart_data_index + i].start_index_;
      int ptstotal_number = ehr_to_emdata_ptr_->lane_elements_data_idx_[ptsstart_data_index + i].total_number_;
      for (int j = ptsstart_index; j < ptsstart_index + ptstotal_number && total_map_pts < kRoadedgePtsNum; j++)
      {
        point_offsets[total_map_pts] = ehr_to_emdata_ptr_->pts_offset_[j];
        map_pts[total_map_pts] = ehr_to_emdata_ptr_->map_pts_[j];
        if (point_offsets[total_map_pts] <= kValidMapPointOffset)
        {
          valid_start_pts = total_map_pts;
        }
        total_map_pts++;
      }
    }
    int valid_start_seg = -1;
    for (int i = 0; i < total_segs && valid_start_seg == -1; i++)
    {
      if (roadedge_segs[i].end_offset_ >= kValidMapSegmentOffset)
      {
        valid_start_seg = i;
      }
    }
    // No valid roadedge segments
    if (valid_start_seg == -1)
    {
      return;
    }
    // Select valid roadedge segments
    bc::TCArray<int32_t, kMaxBoundarySegment> roadedge_segs_start_offset = {0};
    bc::TCArray<int32_t, kMaxBoundarySegment> roadedge_segs_end_offset = {0};
    bc::TCArray<::zone::data::em_data::RoadEdgeType, kMaxBoundarySegment> roadedge_segs_type = {
        ::zone::data::em_data::RoadEdgeType::kUnknown};
    bc::uint8_t seg_idx = 0;
    ::zone::data::em_data::RoadEdgeType last_roadedge_type = ::zone::data::em_data::RoadEdgeType::kUnknown;
    ::zone::data::em_data::RoadEdgeType current_roadedge_type = ::zone::data::em_data::RoadEdgeType::kUnknown;
    for (int i = valid_start_seg; i < total_segs && seg_idx < kMaxBoundarySegment; i++)
    {
      Ehr2EmEnum::LaneRoadedgeTypeConvert(roadedge_segs[i].road_edge_type_, current_roadedge_type);
      if (seg_idx == 0 || last_roadedge_type != current_roadedge_type ||
          (roadedge_segs[i].start_offset_ - roadedge_segs_end_offset[seg_idx - 1]) > 100)
      {
        roadedge_segs_start_offset[seg_idx] = roadedge_segs[i].start_offset_;
        roadedge_segs_end_offset[seg_idx] = roadedge_segs[i].end_offset_;
        roadedge_segs_type[seg_idx] = current_roadedge_type;
        last_roadedge_type = current_roadedge_type;
        seg_idx++;
      }
      else
      {
        roadedge_segs_end_offset[seg_idx - 1] = roadedge_segs[i].end_offset_;
      }
    }
    // Always add the first index to output as init point
    bc::uint32_t total_size = 0;
    bc::uint32_t idx = UINT32_MAX;
    bc::float32_t t_x = map_pts[valid_start_pts].x;
    bc::float32_t t_y = map_pts[valid_start_pts].y;
    em_lane_roadedge.pts_[total_size].x = t_x;
    em_lane_roadedge.pts_[total_size].y = t_y;
    total_size++;
    bc::uint8_t start_seg_idx = 0;
    bc::bool_t find_start_seg_idx = bc::false_v;
    for (bc::uint8_t i = 0; i < seg_idx; ++i)
    {
      if (!find_start_seg_idx && point_offsets[valid_start_pts] <= roadedge_segs_end_offset[i])
      {
        start_seg_idx = i;
        find_start_seg_idx = bc::true_v;
      }
      if (find_start_seg_idx)
      {
        em_lane_roadedge.segs_[i - start_seg_idx].type_ = roadedge_segs_type[i];
      }
    }
    // Choose useful index to generate points
    bc::uint8_t current_seg_idx = 0;
    for (uint32_t i = valid_start_pts + 1; i < total_map_pts && total_size < (uint32_t)kMaxBoundaryPoint; i++)
    {
      bc::float32_t cur_x = map_pts[i].x;
      bc::float32_t cur_y = map_pts[i].y;
      if ((hypot(t_x - cur_x, t_y - cur_y) < kEdgeMinDistance))
      {
        continue;
      }
      t_x = cur_x;
      t_y = cur_y;

      em_lane_roadedge.pts_[total_size].x = cur_x;
      em_lane_roadedge.pts_[total_size].y = cur_y;
      // Find seg start&end index
      if (start_seg_idx < seg_idx && point_offsets[i] <= roadedge_segs_end_offset[start_seg_idx])
      {
        em_lane_roadedge.segs_[current_seg_idx].end_idx_ = total_size;
        em_lane_roadedge.segs_[current_seg_idx].valid_ = bc::true_v;
      }
      else if (start_seg_idx < seg_idx - 1)
      {
        current_seg_idx++;
        start_seg_idx++;
        if (em_lane_roadedge.segs_[current_seg_idx - 1].end_idx_ != 0)
        {
          em_lane_roadedge.segs_[current_seg_idx].start_idx_ = total_size;
          em_lane_roadedge.segs_[current_seg_idx].end_idx_ = total_size;
          em_lane_roadedge.segs_[current_seg_idx].valid_ = bc::true_v;
        }
      }
      total_size++;
    }
    if (total_size > 0)
    {
      em_lane_roadedge.existence_ = bc::true_v;
    }
  }
}

void EhrDataHandle::getLaneBoundarysBySegment(const zone::common::EhrToEmLaneElement& lane_element,
                                              LaneElement& em_lane_element, bc::float32_t terminated_offset)
{
  getLaneBoundarys(lane_element.left_boundary_, em_lane_element.left_boundary_, EHR2EM_LFET_LANEBOUNDARY,
                   terminated_offset);
  getLaneBoundarys(lane_element.right_boundary_, em_lane_element.right_boundary_, EHR2EM_RIGTH_LANEBOUNDARY,
                   terminated_offset);
}

void EhrDataHandle::getSurfaces(zone::data::em_data::EmAdapterHDmapData& map_em_data)
{
  bc::uint8_t total_cross_stop_line_num = 0;
  bc::uint8_t total_traffic_light_num = 0;
  bc::bool_t has_intersection = bc::false_v;
  for (uint32_t surface_idx = 0; surface_idx < ehr_to_emdata_ptr_->surface_features_num_; surface_idx++)
  {
    zone::common::EhrToEmSurfaceFeaturesData current_surface = ehr_to_emdata_ptr_->surface_features_[surface_idx];
    bc::float32_t t_delta_x_0 = 0.f;
    bc::float32_t t_delta_y_0 = 0.f;
    bc::float32_t t_delta_x_1 = 0.f;
    bc::float32_t t_delta_y_1 = 0.f;
    bc::float32_t t_length_0 = 0.f;
    bc::float32_t t_length_1 = 0.f;
    bc::float32_t t_delta_x = 0.f;
    bc::float32_t t_delta_y = 0.f;
    switch (ehr_to_emdata_ptr_->surface_features_[surface_idx].surface_features_type_)
    {
      case common::EEhrToEmSurfaceFeaturesType::EHRTOEM_SURFACE_FEATURES_TYPE_PEDESTRIAN_CROSSING:
        if (total_cross_stop_line_num == zone::data::em_data::kMaxCrossStopLineNum ||
            0.5 * (current_surface.surface_point_[0].x + current_surface.surface_point_[2].x) < -20.f)
        {
          continue;
        }
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].id_ = current_surface.surface_id_;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].type_ =
            zone::data::em_data::EmCrossStopLineType::kCrossLine;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].position_.x =
            0.5 * (current_surface.surface_point_[0].x + current_surface.surface_point_[2].x);
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].position_.y =
            0.5 * (current_surface.surface_point_[0].y + current_surface.surface_point_[2].y);
        t_delta_x_0 = current_surface.surface_point_[0].x - current_surface.surface_point_[1].x;
        t_delta_y_0 = current_surface.surface_point_[0].y - current_surface.surface_point_[1].y;
        t_delta_x_1 = current_surface.surface_point_[0].x - current_surface.surface_point_[3].x;
        t_delta_y_1 = current_surface.surface_point_[0].y - current_surface.surface_point_[3].y;
        t_length_0 = sqrt(t_delta_x_0 * t_delta_x_0 + t_delta_y_0 * t_delta_y_0);
        t_length_1 = sqrt(t_delta_x_1 * t_delta_x_1 + t_delta_y_1 * t_delta_y_1);
        if (t_length_0 > t_length_1)
        {
          map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].length_ = t_length_0;
          map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].width_ = t_length_1;
        }
        else
        {
          map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].length_ = t_length_1;
          map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].width_ = t_length_0;
        }
        // map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].length_ =
        //     sqrt(t_delta_x_0 * t_delta_x_0 + t_delta_y_0 * t_delta_y_0);
        // map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].width_ =
        //     sqrt(t_delta_x_1 * t_delta_x_1 + t_delta_y_1 * t_delta_y_1);
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].heading_ = current_surface.surface_heading_;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].valid_ = bc::true_v;
        total_cross_stop_line_num++;
        break;

      case common::EEhrToEmSurfaceFeaturesType::EHRTOEM_SURFACE_FEATURES_TYPE_STOPPING_LOCATION:
        if (total_cross_stop_line_num == zone::data::em_data::kMaxCrossStopLineNum ||
            0.5 * (current_surface.surface_point_[0].x + current_surface.surface_point_[1].x) < -20.f)
        {
          continue;
        }
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].id_ = current_surface.surface_id_;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].type_ =
            zone::data::em_data::EmCrossStopLineType::kStopLine;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].position_.x =
            0.5 * (current_surface.surface_point_[0].x + current_surface.surface_point_[1].x);
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].position_.y =
            0.5 * (current_surface.surface_point_[0].y + current_surface.surface_point_[1].y);
        t_delta_x = current_surface.surface_point_[0].x - current_surface.surface_point_[1].x;
        t_delta_y = current_surface.surface_point_[0].y - current_surface.surface_point_[1].y;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].length_ =
            sqrt(t_delta_x * t_delta_x + t_delta_y * t_delta_y);
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].width_ = 0.2f;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].heading_ = current_surface.surface_heading_;
        map_em_data.cross_stop_line_ar_[total_cross_stop_line_num].valid_ = bc::true_v;
        total_cross_stop_line_num++;
        break;

      case common::EEhrToEmSurfaceFeaturesType::EHRTOEM_SURFACE_FEATURES_TYPE_TRAFFIC_LIGHT:
        if (total_traffic_light_num == zone::data::em_data::kMaxTrafficLightNum ||
            current_surface.surface_point_[0].x < -10.f)
        {
          continue;
        }
        for (bc::uint8_t light_idx = 0; light_idx < current_surface.surface_features_num_; light_idx++)
        {
          bc::uint16_t t_traffic_light_type_u16 = 0;
          if (GetBitU16(current_surface.surface_features_lamp_type_[light_idx],
                        static_cast<bc::uint8_t>(zone::data::em_data::EmLaneMarkingTypeMsk::kForwardMsk)))
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kCircleMask), bc::true_v,
                      t_traffic_light_type_u16);
          }
          if (GetBitU16(current_surface.surface_features_lamp_type_[light_idx],
                        static_cast<bc::uint8_t>(zone::data::em_data::EmLaneMarkingTypeMsk::kTurnLeftMsk)))
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kLeftArrowMask), bc::true_v,
                      t_traffic_light_type_u16);
          }
          if (GetBitU16(current_surface.surface_features_lamp_type_[light_idx],
                        static_cast<bc::uint8_t>(zone::data::em_data::EmLaneMarkingTypeMsk::kTurnRightMsk)))
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kRightArrowMask), bc::true_v,
                      t_traffic_light_type_u16);
          }
          if (GetBitU16(current_surface.surface_features_lamp_type_[light_idx],
                        static_cast<bc::uint8_t>(zone::data::em_data::EmLaneMarkingTypeMsk::kUTurnMsk)))
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kUturnMask), bc::true_v,
                      t_traffic_light_type_u16);
          }
          if (GetBitU16(current_surface.surface_features_lamp_type_[light_idx],
                        static_cast<bc::uint8_t>(zone::data::em_data::EmLaneMarkingTypeMsk::kProhibitMsk)))
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kProhibitMsk), bc::true_v,
                      t_traffic_light_type_u16);
          }
          if (current_surface.surface_features_lamp_countdown_[light_idx])
          {
            SetBitU16(static_cast<bc::uint8_t>(zone::data::em_data::EmTrafficLightTypeMsk::kTimeCountdownMsk),
                      bc::true_v, t_traffic_light_type_u16);
          }
          map_em_data.traffic_light_ar_[total_traffic_light_num].id_ = current_surface.surface_id_;
          map_em_data.traffic_light_ar_[total_traffic_light_num].type_ = t_traffic_light_type_u16;
          map_em_data.traffic_light_ar_[total_traffic_light_num].position_.x = current_surface.surface_point_[0].x;
          map_em_data.traffic_light_ar_[total_traffic_light_num].position_.y = current_surface.surface_point_[0].y;
          map_em_data.traffic_light_ar_[total_traffic_light_num].heading_ = current_surface.surface_heading_;
          map_em_data.traffic_light_ar_[total_traffic_light_num].mode_ =
              zone::data::em_data::EmTrafficLightMode::kUnknown;
          map_em_data.traffic_light_ar_[total_traffic_light_num].color_ =
              zone::data::em_data::EmTrafficLightColor::kUnknown;
          map_em_data.traffic_light_ar_[total_traffic_light_num].countdown_ = 0;
          map_em_data.traffic_light_ar_[total_traffic_light_num].countdown_valid_ = bc::false_v;
          map_em_data.traffic_light_ar_[total_traffic_light_num].valid_ = bc::true_v;
          total_traffic_light_num++;
        }
        break;

      case common::EEhrToEmSurfaceFeaturesType::EHRTOEM_SURFACE_FEATURES_TYPE_INTERSECTION_AREA:
        // 限制十字路口的end必须要在自车前方，且必须为离自车最近的
        if (has_intersection &&
            (0.5 * (current_surface.surface_point_[2].x + current_surface.surface_point_[3].x) < 0.f ||
             current_surface.surface_point_[0].x + current_surface.surface_point_[1].x >
                 map_em_data.intersection_data_.its_bound_pts_ar_[0].x +
                     map_em_data.intersection_data_.its_bound_pts_ar_[1].x))
        {
          continue;
        }
        map_em_data.intersection_data_.its_bound_pts_ar_[0].x = current_surface.surface_point_[0].x;
        map_em_data.intersection_data_.its_bound_pts_ar_[0].y = current_surface.surface_point_[0].y;
        map_em_data.intersection_data_.its_bound_pts_ar_[1].x = current_surface.surface_point_[1].x;
        map_em_data.intersection_data_.its_bound_pts_ar_[1].y = current_surface.surface_point_[1].y;
        map_em_data.intersection_data_.its_bound_pts_ar_[2].x = current_surface.surface_point_[2].x;
        map_em_data.intersection_data_.its_bound_pts_ar_[2].y = current_surface.surface_point_[2].y;
        map_em_data.intersection_data_.its_bound_pts_ar_[3].x = current_surface.surface_point_[3].x;
        map_em_data.intersection_data_.its_bound_pts_ar_[3].y = current_surface.surface_point_[3].y;
        map_em_data.intersection_data_.valid_b_ = bc::true_v;
        has_intersection = bc::true_v;
        break;

      default:
        break;
    }
  }
}

void EhrDataHandle::getLaneMarkings(zone::data::em_data::EmAdapterHDmapData& map_em_data)
{
  bc::uint8_t total_lane_marking_num = 0;
  for (bc::uint8_t lane_idx = 0;
       lane_idx < kMaxLaneNum && total_lane_marking_num < zone::data::em_data::kMaxLaneMarkingNum; lane_idx++)
  {
    if (map_em_data.lanes_[lane_idx].lane_valid_)
    {
      for (bc::uint8_t ele_idx = 0;
           ele_idx < kMaxElemNumInOneLane && total_lane_marking_num < zone::data::em_data::kMaxLaneMarkingNum;
           ele_idx++)
      {
        if (map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].element_valid_)
        {
          zone::data::em_data::ReferenceLine& cur_refline =
              map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].reference_line_;
          if (cur_refline.available_)
          {
            bc::uint8_t ref_start_idx = 0;
            for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLanePropertySegsInOneLaneElement &&
                                          total_lane_marking_num < zone::data::em_data::kMaxLaneMarkingNum;
                 seg_idx++)
            {
              // Pass dulipcated ID lane marking
              bc::bool_t dupliate_lane_marking = bc::false_v;
              for (bc::uint8_t marking_idx = 0; marking_idx < total_lane_marking_num; marking_idx++)
              {
                if (map_em_data.lane_marking_ar_[marking_idx].id_ == cur_refline.arrow_segs_[seg_idx].id_ + 1)
                {
                  dupliate_lane_marking = bc::true_v;
                  break;
                }
              }
              if (dupliate_lane_marking)
              {
                continue;
              }
              // Set lane marking and use offset to find position
              if (cur_refline.arrow_segs_[seg_idx].valid_ && cur_refline.arrow_segs_[seg_idx].start_s_ > -10.f &&
                  cur_refline.arrow_segs_[seg_idx].start_s_ <
                      cur_refline.ref_line_pts_[cur_refline.ref_start_idx_ + cur_refline.ref_line_valid_pts_ - 1]
                          .pos_.z)
              {
                map_em_data.lane_marking_ar_[total_lane_marking_num].id_ = cur_refline.arrow_segs_[seg_idx].id_ + 1;
                map_em_data.lane_marking_ar_[total_lane_marking_num].type_ =
                    cur_refline.arrow_segs_[seg_idx].arrow_type_;
                map_em_data.lane_marking_ar_[total_lane_marking_num].lane_assigned_ = lane_idx;
                for (bc::uint8_t ref_idx = ref_start_idx; ref_idx < kMaxRefLinePtsNum; ref_idx++)
                {
                  if (cur_refline.arrow_segs_[seg_idx].start_s_ < cur_refline.ref_line_pts_[ref_idx].pos_.z)
                  {
                    if (ref_idx == 0)
                    {
                      map_em_data.lane_marking_ar_[total_lane_marking_num].position_.x =
                          cur_refline.ref_line_pts_[ref_idx].pos_.x;
                      map_em_data.lane_marking_ar_[total_lane_marking_num].position_.y =
                          cur_refline.ref_line_pts_[ref_idx].pos_.y;
                    }
                    else
                    {
                      map_em_data.lane_marking_ar_[total_lane_marking_num].position_.x =
                          cur_refline.ref_line_pts_[ref_idx - 1].pos_.x;
                      map_em_data.lane_marking_ar_[total_lane_marking_num].position_.y =
                          cur_refline.ref_line_pts_[ref_idx - 1].pos_.y;
                      ref_start_idx = ref_idx;
                    }
                    break;
                  }
                }
                map_em_data.lane_marking_ar_[total_lane_marking_num].heading_ = 0.f;
                map_em_data.lane_marking_ar_[total_lane_marking_num].valid_ = bc::true_v;
                map_em_data.lane_marking_element_id_ar_[total_lane_marking_num] =
                    map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].element_id_;
                total_lane_marking_num++;
              }
            }
          }
        }
      }
    }
  }
}

// void EhrDataHandle::getRoadEdgesBySegment(const zone::common::EhrToEmLaneElement& lane_element,
//                                           LaneElement& em_lane_element)
// {
//   getRoadEdges(lane_element.left_road_edge_, em_lane_element.left_road_edge_);
//   getRoadEdges(lane_element.right_road_edge_, em_lane_element.right_road_edge_);
// }