#include "olr.h"
#include "coordinate_angle_converter.h"
#include "math/bitwise_operation.h"
#include "math/low_pass_filter.h"

namespace zone {
namespace environment_model {

using ::zone::environment_model::CoordinateAngleConverter;
using ::zone::helper::math::LowPass;
using ::zone::helper::math::GetBitU8;

Olr::Olr(EmData& em_collection, EmData& em_collection_lst_cycle)
    : em_collection_(&em_collection), em_collection_lst_cycle_(&em_collection_lst_cycle), object_prob_collection{}
{
}

void Olr::Run()
{
  // Set default value for road_struct_change_info_
  for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < ::zone::data::em_data::kMaxLaneNum; t_laneidx_u8++)
  {
    for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < ::zone::data::em_data::kMaxElemNumInOneLane;
         t_elementidx_u8++)
    {
      road_struct_change_info_[t_laneidx_u8][t_elementidx_u8] = LaneElementMapping();
    }
  }
  // Match element with the one which has the same element id in last cycle
  ElementMappingWithLstCycle();
  obj_container.clear();
  obj_index.clear();
  if (CollectValidObjects())
  {
    // Reset prob_max_ar_ every cycle before use
    for (bc::uint8_t idx = 0; idx < ::zone::data::em_data::kFusMaxObjNum; idx++)
    {
      prob_max_ar_[idx] = 0.f;
    }
    ObjectAssignment();
    // UpdateObjectAssociation();
    // If the highest prob in a certain lane is lower than FLOAT32_EPSILON,
    // set idx_assigned_lane_ as default value 5
    for (bc::uint8_t idx = 0; idx < obj_container.size(); idx++)
    {
      bc::uint8_t& t_idx_assigned_lane_u8 = (*em_collection_).agents_[obj_index[idx]].idx_assigned_lane_;
      if (t_idx_assigned_lane_u8 != ::zone::data::em_data::kUnknownLane)
      {
        if ((*em_collection_).agents_[obj_index[idx]].lane_association_[t_idx_assigned_lane_u8].probability_ <
            FLOAT32_EPSILON)
        {
          t_idx_assigned_lane_u8 = ::zone::data::em_data::kUnknownLane;
        }
      }
    }
  }
  last_cycle_ego_lc_cnts_ = (*em_collection_).ego_lc_cnts_;
  // Store all the object ids from last cycle
  for (bc::uint8_t i = 0; i < ::zone::data::em_data::kFusMaxObjNum; ++i)
  {
    last_cycle_obj_ids[i] = (*em_collection_).agents_[i].id_;
  }
}

void Olr::ElementMappingWithLstCycle()
{
  for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < ::zone::data::em_data::kMaxLaneNum; t_laneidx_u8++)
  {
    if ((*em_collection_).lanes_[t_laneidx_u8].lane_valid_)
    {
      for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < ::zone::data::em_data::kMaxElemNumInOneLane;
           t_elementidx_u8++)
      {
        LaneElement& t_element_cs = (*em_collection_).lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
        LaneElement& t_lst_element_cs =
            (*em_collection_lst_cycle_).lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
        if (t_element_cs.element_id_ != 0)
        {
          // If element id is valid in current cycle, set road_struct_change_info_ valid
          road_struct_change_info_[t_laneidx_u8][t_elementidx_u8].valid_ = bc::true_v;
          if (t_element_cs.element_id_ != t_lst_element_cs.element_id_)
          {
            road_struct_change_info_[t_laneidx_u8][t_elementidx_u8].element_id_change_b_ = bc::true_v;
            ElementMapping(t_element_cs, road_struct_change_info_[t_laneidx_u8][t_elementidx_u8].is_old_element_b_,
                           road_struct_change_info_[t_laneidx_u8][t_elementidx_u8].lst_lane_index_u8_,
                           road_struct_change_info_[t_laneidx_u8][t_elementidx_u8].lst_element_index_u8_);
          }
          else
          {
            // keep default value
          }
        }
        else
        {
          // keep default value
          road_struct_change_info_[t_laneidx_u8][t_elementidx_u8] = LaneElementMapping();
        }
      }
    }
  }
}

void Olr::ElementMapping(const LaneElement& current_element, bc::bool_t& is_old_element, bc::uint8_t& lst_lane_idx,
                         bc::uint8_t& lst_element_idx)
{
  for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < ::zone::data::em_data::kMaxLaneNum && !is_old_element;
       t_laneidx_u8++)
  {
    for (bc::uint8_t t_elementidx_u8 = 0;
         t_elementidx_u8 < ::zone::data::em_data::kMaxElemNumInOneLane && !is_old_element; t_elementidx_u8++)
    {
      LaneElement& t_lst_element_cs = (*em_collection_lst_cycle_).lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
      if (current_element.element_id_ == t_lst_element_cs.element_id_)
      {
        is_old_element = bc::true_v;
        lst_lane_idx = t_laneidx_u8;
        lst_element_idx = t_elementidx_u8;
      }
      else
      {
        // keep default value
      }
    }
  }
}

bc::bool_t Olr::AgentDistanceToBorder(const bc::float32_t& refline_point_x, const bc::float32_t& refline_point_y,
                                      const bc::float32_t& refline_point_heading, const bc::float32_t& obj_x,
                                      const bc::float32_t& obj_y, const bc::float32_t& obj_width,
                                      const bc::float32_t& obj_length, const bc::float32_t& obj_heading,
                                      const bc::float32_t& lane_width, bc::float32_t& distance_of_left_to_left_border,
                                      bc::float32_t& distance_of_right_to_left_border,
                                      bc::float32_t& distance_of_left_to_right_border,
                                      bc::float32_t& distance_of_right_to_right_border, bc::float32_t& prob)
{
  // Stop calculating if the attribute of object and lane are invalid
  if (obj_width <= FLOAT32_EPSILON || obj_length <= FLOAT32_EPSILON || lane_width <= FLOAT32_EPSILON)
  {
    return bc::false_v;
  }
  // Calculate object four corner points without considering object heading
  Point2D temp_right_top_point(obj_x + obj_length / 2.f, obj_y + obj_width / 2.f);
  Point2D temp_right_bottom_point(obj_x + obj_length / 2.f, obj_y - obj_width / 2.f);
  Point2D temp_left_top_point(obj_x - obj_length / 2.f, obj_y + obj_width / 2.f);
  Point2D temp_left_bottom_point(obj_x - obj_length / 2.f, obj_y - obj_width / 2.f);
  // Recalculate object four corner points taking object heading into consideration
  Point2D right_top_point(
      (temp_right_top_point.x - obj_x) * cos(obj_heading) - (temp_right_top_point.y - obj_y) * sin(obj_heading) + obj_x,
      (temp_right_top_point.x - obj_x) * sin(obj_heading) + (temp_right_top_point.y - obj_y) * cos(obj_heading) +
          obj_y);
  Point2D right_bottom_point((temp_right_bottom_point.x - obj_x) * cos(obj_heading) -
                                 (temp_right_bottom_point.y - obj_y) * sin(obj_heading) + obj_x,
                             (temp_right_bottom_point.x - obj_x) * sin(obj_heading) +
                                 (temp_right_bottom_point.y - obj_y) * cos(obj_heading) + obj_y);
  Point2D left_top_point(
      (temp_left_top_point.x - obj_x) * cos(obj_heading) - (temp_left_top_point.y - obj_y) * sin(obj_heading) + obj_x,
      (temp_left_top_point.x - obj_x) * sin(obj_heading) + (temp_left_top_point.y - obj_y) * cos(obj_heading) + obj_y);
  Point2D left_bottom_point((temp_left_bottom_point.x - obj_x) * cos(obj_heading) -
                                (temp_left_bottom_point.y - obj_y) * sin(obj_heading) + obj_x,
                            (temp_left_bottom_point.x - obj_x) * sin(obj_heading) +
                                (temp_left_bottom_point.y - obj_y) * cos(obj_heading) + obj_y);
  Point2D right_most_point(0.f, 0.f);
  Point2D left_most_point(0.f, 0.f);
  bc::float32_t distance_from_right_most_to_refline = 0.f;
  bc::float32_t distance_from_left_most_to_refline = 0.f;
  bc::float32_t delta_theta = refline_point_heading - obj_heading;
  // Uint point on the given lane point
  Point2D unit_point(cos(refline_point_heading) + refline_point_x, sin(refline_point_heading) + refline_point_y);
  // Find the left and right most point in object four corner points
  bc::float32_t delta_theta_valid;
  if (delta_theta >= 0.f)
  {
    delta_theta_valid = fmod(delta_theta, (2 * G_PI));
  }
  else
  {
    delta_theta_valid = -fmod(-delta_theta, (2 * G_PI));
  }
  if ((delta_theta_valid >= 0.f && delta_theta_valid <= 0.5 * G_PI) ||
      (delta_theta_valid >= -2 * G_PI && delta_theta_valid <= -1.5 * G_PI))
  {
    left_most_point = left_top_point;
    right_most_point = right_bottom_point;
  }
  else if ((delta_theta_valid >= 0.5 * G_PI && delta_theta_valid <= G_PI) ||
           (delta_theta_valid >= -1.5 * G_PI && delta_theta_valid <= -G_PI))
  {
    left_most_point = left_bottom_point;
    right_most_point = right_top_point;
  }
  else if ((delta_theta_valid >= G_PI && delta_theta_valid <= 1.5 * G_PI) ||
           (delta_theta_valid >= -G_PI && delta_theta_valid <= -0.5 * G_PI))
  {
    left_most_point = right_bottom_point;
    right_most_point = left_top_point;
  }
  else if ((delta_theta_valid >= 1.5 * G_PI && delta_theta_valid <= 2 * G_PI) ||
           (delta_theta_valid >= -0.5 * G_PI && delta_theta_valid <= 0.f))
  {
    left_most_point = right_top_point;
    right_most_point = left_bottom_point;
  }
  // Calculate the distance from left and right most point of object
  // to the given refline point
  distance_from_right_most_to_refline = fabs((refline_point_y - unit_point.y) * right_most_point.x +
                                             (unit_point.x - refline_point_x) * right_most_point.y +
                                             (refline_point_x * unit_point.y - unit_point.x * refline_point_y));
  distance_from_left_most_to_refline =
      fabs((refline_point_y - unit_point.y) * left_most_point.x + (unit_point.x - refline_point_x) * left_most_point.y +
           (refline_point_x * unit_point.y - unit_point.x * refline_point_y));
  // Judge the symbol of distance
  // Positive if the corner point on the refline point left and negative if on the right
  Point2D vector_of_direction(cos(refline_point_heading), sin(refline_point_heading));
  Point2D vector_left_to_refline(left_most_point.x - refline_point_x, left_most_point.y - refline_point_y);
  Point2D vector_right_to_refline(right_most_point.x - refline_point_x, right_most_point.y - refline_point_y);
  bc::float32_t cross_product_left =
      vector_of_direction.x * vector_left_to_refline.y - vector_left_to_refline.x * vector_of_direction.y;
  bc::float32_t cross_product_right =
      vector_of_direction.x * vector_right_to_refline.y - vector_right_to_refline.x * vector_of_direction.y;
  if (cross_product_left < 0.f)
  {
    distance_from_left_most_to_refline = -distance_from_left_most_to_refline;
  }
  if (cross_product_right < 0.f)
  {
    distance_from_right_most_to_refline = -distance_from_right_most_to_refline;
  }
  // Judge the symbol of the point to border
  // If the left obj point is on the left side of the left border,
  // distance_of_left_to_border should be positive;
  // If the right obj point is on the left side of the right border,
  // distance_of_right_to_border should be positive.
  distance_of_left_to_left_border = distance_from_left_most_to_refline - lane_width / 2.f;
  distance_of_right_to_left_border = distance_from_right_most_to_refline - lane_width / 2.f;
  distance_of_left_to_right_border = distance_from_left_most_to_refline + lane_width / 2.f;
  distance_of_right_to_right_border = distance_from_right_most_to_refline + lane_width / 2.f;
  // Calculate the probability of the object in lane by:
  //   part of object in lane / whole object
  if (distance_of_left_to_left_border * distance_of_right_to_right_border <= 0.f)
  {
    // Whole object is within lane or object occupy the entire lane
    prob = 1.0f;
  }
  else if (distance_of_left_to_left_border > 0.f && distance_of_right_to_right_border > 0.f)
  {
    bc::float32_t temp_in = lane_width - distance_of_right_to_right_border;
    bc::float32_t temp_out = distance_of_left_to_left_border;
    prob = temp_in / (temp_in + temp_out);
  }
  else
  {
    bc::float32_t temp_in = lane_width + distance_of_left_to_left_border;
    bc::float32_t temp_out = -distance_of_right_to_right_border;
    prob = temp_in / (temp_in + temp_out);
  }
  // Using temp_in to calculate prob may result in negative value,
  // which means out of the lane, so we can just set the prob to 0
  if (prob < 0.f)
  {
    prob = 0.f;
  }
  return bc::true_v;
}

bc::bool_t Olr::CollectValidObjects()
{
  // Store the valid objects and their related indices in container
  for (bc::uint8_t obj_idx = 0; obj_idx < ::zone::data::em_data::kFusMaxObjNum; ++obj_idx)
  {
    if ((*em_collection_).agents_[obj_idx].valid_)
    {
      obj_container.push_back((*em_collection_).agents_[obj_idx]);
      obj_index.push_back(obj_idx);
    }
  }
  // Check the number of valid objects
  if (obj_container.size() < 1)
  {
    return bc::false_v;
  }
  else
  {
    return bc::true_v;
  }
}

void Olr::UpdateObjectAssociation()
{
  for (bc::uint8_t i = 0; i < ::zone::data::em_data::kFusMaxObjNum; ++i)
  {
    for (bc::uint8_t j = 0; j < ::zone::data::em_data::kMaxLaneAssociationNum; ++j)
    {
      if ((*em_collection_).agents_[i].lane_association_[j].probability_ < FLOAT32_EPSILON)
      {
        (*em_collection_).agents_[i].lane_association_[j] = LaneAssociation();
        break;
      }
      else if ((*em_collection_).agents_[i].lane_association_[j].probability_ <= kProbDeadzone)
      {
        (*em_collection_).agents_[i].lane_association_[j] = LaneAssociation();
      }
    }
  }
}

void Olr::ObjectAssignment()
{
  // -1: lane change left; 0: lane keeping; 1: lane change right;
  bc::int32_t lane_change_status = (*em_collection_).ego_lc_cnts_ - last_cycle_ego_lc_cnts_;
  if (lane_change_status > 1 || lane_change_status < -1)
  {
    /// TODO: Log invalid lane change count error and modify return value
    return;
  }
  // Reset lane association
  for (bc::uint8_t i = 0; i < kFusMaxObjNum; ++i)
  {
    if ((*em_collection_).agents_[i].valid_)
    {
      for (bc::uint8_t j = 0; j < ::zone::data::em_data::kMaxLaneAssociationNum; ++j)
      {
        (*em_collection_).agents_[i].lane_association_[j] = LaneAssociation();
      }
    }
  }
  // bc::TCArray<bc::float32_t, ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum> prob_accumulator;
  // Initiate all values in probability container to 0
  // for (bc::uint8_t prob_i = 0; prob_i < ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum; ++prob_i) {
  //   prob_accumulator[prob_i] = 0.f;
  // }
  // Create a temporary prob collection for each object in each lane and element
  bc::TCArray<bc::TCArray<bc::TCArray<bc::float32_t, ::zone::data::em_data::kMaxElemNumInOneLane>,
                          ::zone::data::em_data::kMaxLaneNum>,
              ::zone::data::em_data::kFusMaxObjNum>
      temp_object_prob_collection = object_prob_collection;
  for (bc::uint8_t i = 0; i < ::zone::data::em_data::kMaxLaneNum; ++i)
  {
    LaneData& current_lane = (*em_collection_).lanes_[i];
    if (current_lane.lane_valid_)
    {
      for (bc::uint8_t j = 0; j < ::zone::data::em_data::kMaxElemNumInOneLane; ++j)
      {
        if (current_lane.lane_elements_[j].element_valid_)
        {
          bc::TFixedVector<RefLinePoint, ::zone::data::em_data::kMaxRefLinePtsNum> lane_points;
          bc::TFixedVector<SLTPoint, ::zone::data::em_data::kFusMaxObjNum> sl_points;
          bc::TFixedVector<RefLinePoint, ::zone::data::em_data::kFusMaxObjNum> refline_points;
          bc::TFixedVector<bc::uint8_t, ::zone::data::em_data::kFusMaxObjNum> refline_idx;
          ReferenceLine& current_refline = current_lane.lane_elements_[j].reference_line_;
          // Collect all valid reference points into lane_points
          for (bc::uint8_t k = 0; k < ::zone::data::em_data::kMaxRefLinePtsNum; ++k)
          {
            RefLinePoint current_refline_point = current_refline.ref_line_pts_[k];
            lane_points.push_back(current_refline_point);
          }
          // Find proper refline in last cycle to claculate SLTPoint and filter the prob
          ReferenceLine t_ref_refline = (*em_collection_lst_cycle_).lanes_[i].lane_elements_[j].reference_line_;
          LaneElementMapping t_current_element_mapping_info = road_struct_change_info_[i][j];
          bc::TCArray<bc::float32_t, ::zone::data::em_data::kFusMaxObjNum> t_object_prob_collection_f;
          // Set default value
          for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
          {
            t_object_prob_collection_f[t_obj_idx] = 0.f;
          }
          if (t_current_element_mapping_info.valid_)
          {
            if (t_current_element_mapping_info.element_id_change_b_ && t_current_element_mapping_info.is_old_element_b_)
            {
              t_ref_refline = (*em_collection_lst_cycle_)
                                  .lanes_[t_current_element_mapping_info.lst_lane_index_u8_]
                                  .lane_elements_[t_current_element_mapping_info.lst_element_index_u8_]
                                  .reference_line_;
              for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
              {
                t_object_prob_collection_f[t_obj_idx] =
                    object_prob_collection[t_obj_idx][t_current_element_mapping_info.lst_lane_index_u8_]
                                          [t_current_element_mapping_info.lst_element_index_u8_];
              }
            }
            else if (t_current_element_mapping_info.element_id_change_b_ &&
                     !t_current_element_mapping_info.is_old_element_b_)
            {
              /**Do nothing*/
            }
            else
            {
              for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
              {
                t_object_prob_collection_f[t_obj_idx] = object_prob_collection[t_obj_idx][i][j];
              }
            }
          }
          else
          {
            if (lane_change_status == -1)
            {
              if (i > 0 && ((*em_collection_lst_cycle_).lanes_[i - 1].lane_valid_) &&
                  ((*em_collection_lst_cycle_).lanes_[i - 1].lane_elements_[j].element_valid_))
              {
                t_ref_refline = (*em_collection_lst_cycle_).lanes_[i - 1].lane_elements_[j].reference_line_;
                for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
                {
                  t_object_prob_collection_f[t_obj_idx] = object_prob_collection[t_obj_idx][i - 1][j];
                }
              }
              else
              {
                t_ref_refline = ReferenceLine();
              }
            }
            else if (lane_change_status == 1)
            {
              if (i < ::zone::data::em_data::kMaxLaneNum - 1 &&
                  ((*em_collection_lst_cycle_).lanes_[i + 1].lane_valid_) &&
                  ((*em_collection_lst_cycle_).lanes_[i + 1].lane_elements_[j].element_valid_))
              {
                t_ref_refline = (*em_collection_lst_cycle_).lanes_[i + 1].lane_elements_[j].reference_line_;
                for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
                {
                  t_object_prob_collection_f[t_obj_idx] = object_prob_collection[t_obj_idx][i + 1][j];
                }
              }
              else
              {
                t_ref_refline = ReferenceLine();
              }
            }
            else
            {
              for (bc::uint8_t t_obj_idx = 0; t_obj_idx < ::zone::data::em_data::kFusMaxObjNum; t_obj_idx++)
              {
                t_object_prob_collection_f[t_obj_idx] = object_prob_collection[t_obj_idx][i][j];
              }
            }
          }
          // Convert object from cartesian to frenet in sl coordinator
          if (CoordinateAngleConverter::CartesianToFrenetAnglebasedMutiObjects(
                  t_ref_refline, cycle_time_, obj_index, kSlLLowpass, kSlVlLowpass, lane_points, obj_container,
                  t_current_element_mapping_info, sl_points, refline_points, refline_idx))
          {
            bc::uint8_t idx = 0;
            for (bc::uint8_t point_idx = 0; point_idx < ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum;
                 ++point_idx)
            {
              AgentCurrentPosProjOnToRefLine& current_agent_projection =
                  current_refline.agents_projected_traj_[point_idx];
              if (idx < obj_container.size())
              {
                // Check the validity of the object and related SL point
                if (obj_index[idx] == point_idx && sl_points[idx].valid_)
                {
                  // if (obj_container[idx].pos_.x >=
                  // current_refline.ref_line_pts_[(uint8_t)current_refline.reserve_[0]].pos_.x)
                  // {
                  bc::float32_t t_distance_of_left_to_left_border_f = 0.f;
                  bc::float32_t t_distance_of_right_to_left_border_f = 0.f;
                  bc::float32_t t_distance_of_left_to_right_border_f = 0.f;
                  bc::float32_t t_distance_of_right_to_right_border_f = 0.f;
                  bc::float32_t prob = 0.f;
                  if (AgentDistanceToBorder(
                          refline_points[idx].pos_.x, refline_points[idx].pos_.y, refline_points[idx].heading_,
                          obj_container[idx].pos_.x, obj_container[idx].pos_.y, obj_container[idx].width_,
                          obj_container[idx].length_, obj_container[idx].heading_, refline_points[idx].lane_width_,
                          t_distance_of_left_to_left_border_f, t_distance_of_right_to_left_border_f,
                          t_distance_of_left_to_right_border_f, t_distance_of_right_to_right_border_f, prob))
                  {
                    // Calculate filtered probability with ego lane change status
                    bc::float32_t filtered_prob = 0.f;
                    if (prob < FLOAT32_EPSILON &&
                        t_distance_of_left_to_left_border_f * t_distance_of_right_to_right_border_f > FLOAT32_EPSILON)
                    {
                      // Multiplication of distance to left and right border greater than 0 means the object is not on
                      // the given lane.
                      // Because of the low pass filter, it will not decrease to 0 immediately which will pass
                      // fault message to the downstream module.
                      // In this case, we will force the filtered_prob to remain 0.
                    }
                    else
                    {
                      // Current cycle Object ID doesn't match last cycle's,
                      // consider the object as a new one and filter the prob from 0.
                      if (last_cycle_obj_ids[point_idx] != (*em_collection_).agents_[point_idx].id_)
                      {
                        filtered_prob = LowPass(prob, 0.3f * prob, kProbLowpass, cycle_time_);
                      }
                      else
                      {
                        if (t_object_prob_collection_f[obj_index[idx]] <= FLOAT32_EPSILON)
                        {
                          filtered_prob = LowPass(prob, 0.3f * prob, kProbLowpass, cycle_time_);
                        }
                        else
                        {
                          if (t_current_element_mapping_info.element_id_change_b_ &&
                              !t_current_element_mapping_info.is_old_element_b_)
                          {
                            filtered_prob = LowPass(prob, 0.3f * prob, kProbLowpass, cycle_time_);
                          }
                          else
                          {
                            filtered_prob =
                                LowPass(prob, t_object_prob_collection_f[obj_index[idx]], kProbLowpass, cycle_time_);
                          }
                        }
                      }
                    }
                    // prob_accumulator[obj_index[idx]] += filtered_prob;
                    temp_object_prob_collection[obj_index[idx]][i][j] = filtered_prob;
                    // Update lane association from lane 0 to lane 4 with each valid prob been calculated
                    LaneAssociation& current_association =
                        (*em_collection_).agents_[obj_index[idx]].lane_association_[i];
                    if (j == 0)
                    {
                      current_association.probability_ = filtered_prob;
                      current_association.dist_left_point_to_left_border_ = t_distance_of_left_to_left_border_f;
                      current_association.dist_right_point_to_left_border_ = t_distance_of_right_to_left_border_f;
                      current_association.dist_left_point_to_right_border_ = t_distance_of_left_to_right_border_f;
                      current_association.dist_right_point_to_right_border_ = t_distance_of_right_to_right_border_f;
                      current_association.assigned_lane_idx_ = i;
                      current_association.assigned_element_idx_ = j;
                      current_association.nearest_ref_line_point_ = refline_points[idx];
                      current_association.valid_ = bc::true_v;
                    }
                    else
                    {
                      current_association = LaneAssociation();
                    }
                    // Calculate lane index with the highest prob and add into idx_assigned_lane_
                    bc::uint8_t& t_idx_assigned_lane_u8 = (*em_collection_).agents_[obj_index[idx]].idx_assigned_lane_;
                    if (t_idx_assigned_lane_u8 == ::zone::data::em_data::kUnknownLane)
                    {
                      t_idx_assigned_lane_u8 = i;
                      prob_max_ar_[obj_index[idx]] = filtered_prob;
                    }
                    else
                    {
                      if (filtered_prob <= prob_max_ar_[obj_index[idx]])
                      {
                        if (((prob_max_ar_[obj_index[idx]] > 0.5f && filtered_prob > 0.5f &&
                              fabsf(prob_max_ar_[obj_index[idx]] - filtered_prob) < 0.05f) ||
                             fabsf(prob_max_ar_[obj_index[idx]] - filtered_prob) <= FLOAT32_EPSILON) &&
                            (i == ::zone::data::em_data::kHostLane ||
                             ((i == ::zone::data::em_data::kLeftLane || i == ::zone::data::em_data::kRightLane) &&
                              (t_idx_assigned_lane_u8 == ::zone::data::em_data::kLLLane ||
                               t_idx_assigned_lane_u8 == ::zone::data::em_data::kRRLane))))
                        {
                          t_idx_assigned_lane_u8 = i;
                          prob_max_ar_[obj_index[idx]] = filtered_prob;
                        }
                        else
                        {
                          /**Do nothing*/
                        }
                      }
                      else
                      {
                        if (((prob_max_ar_[obj_index[idx]] > 0.5f && filtered_prob > 0.5f &&
                              fabsf(prob_max_ar_[obj_index[idx]] - filtered_prob) < 0.05f) ||
                             fabsf(prob_max_ar_[obj_index[idx]] - filtered_prob) <= FLOAT32_EPSILON) &&
                            (t_idx_assigned_lane_u8 == ::zone::data::em_data::kHostLane ||
                             ((t_idx_assigned_lane_u8 == ::zone::data::em_data::kLeftLane ||
                               t_idx_assigned_lane_u8 == ::zone::data::em_data::kRightLane) &&
                              (i == ::zone::data::em_data::kLLLane || i == ::zone::data::em_data::kRRLane))))
                        {
                          /**Do nothing*/
                        }
                        else
                        {
                          t_idx_assigned_lane_u8 = i;
                          prob_max_ar_[obj_index[idx]] = filtered_prob;
                        }
                      }
                    }

                    // if ((*em_collection_)
                    //         .agents_[obj_index[idx]]
                    //         .lane_association_[ ::zone::data::em_data::kMaxLaneAssociationNum - 1]
                    //         .probability_ < filtered_prob)
                    // {
                    //   for (bc::int8_t obj_lane = ::zone::data::em_data::kMaxLaneAssociationNum - 1; obj_lane >= 0;
                    //        --obj_lane)
                    //   {
                    //     // If new calculated filtered_prob is greater than current prob,
                    //     // move the current association data backward by one position and continue loop.
                    //     // If no other association data available or the current prob is greater,
                    //     // place the filtered_prob and related association data to current position
                    //     LaneAssociation& current_association =
                    //         (*em_collection_).agents_[obj_index[idx]].lane_association_[obj_lane];
                    //     if (current_association.probability_ < filtered_prob)
                    //     {
                    //       // Move the current association data backward by one position and continue loop
                    //       if (obj_lane < ::zone::data::em_data::kMaxLaneAssociationNum - 1)
                    //       {
                    //         (*em_collection_).agents_[obj_index[idx]].lane_association_[obj_lane + 1] =
                    //             current_association;
                    //       }
                    //       // No other association data available
                    //       if (obj_lane == 0)
                    //       {
                    //         current_association.probability_ = filtered_prob;
                    //         current_association.distance_to_left_border_ = distance_to_left_border;
                    //         current_association.distance_to_right_border_ = distance_to_right_border;
                    //         current_association.assigned_lane_idx_ = i;
                    //         current_association.assigned_element_idx_ = j;
                    //         current_association.nearest_ref_line_point_ = refline_points[idx];
                    //         current_association.valid_ = bc::true_v;
                    //       }
                    //     }
                    //     else
                    //     {
                    //       // If there is a tie(with same probability), always put the association data
                    //       // of the closest lane to the upper position(close to index 0)
                    //       if (fabsf(current_association.probability_ - filtered_prob) <= FLOAT32_EPSILON)
                    //       {
                    //         // Calculate the distance to given nearest reference line point from (0, 0)
                    //         bc::uint8_t current_point_idx_ = current_refline.current_point_idx_;
                    //         bc::float32_t ref_point_x = current_refline.ref_line_pts_[current_point_idx_].pos_.x;
                    //         bc::float32_t ref_point_y = current_refline.ref_line_pts_[current_point_idx_].pos_.y;
                    //         bc::float32_t distance_to_refline =
                    //             sqrt(ref_point_x * ref_point_x + ref_point_y * ref_point_y);
                    //         // Calculate the distance to current association reference line point from (0, 0)
                    //         bc::uint8_t obj_lane_current_point_idx_ =
                    //             (*em_collection_)
                    //                 .lanes_[current_association.assigned_lane_idx_]
                    //                 .lane_elements_[current_association.assigned_element_idx_]
                    //                 .reference_line_.current_point_idx_;
                    //         ReferenceLine& association_refline = (*em_collection_)
                    //                                                  .lanes_[obj_lane]
                    //                                                  .lane_elements_[(*em_collection_)
                    //                                                                      .agents_[obj_index[idx]]
                    //                                                                      .lane_association_[obj_lane]
                    //                                                                      .assigned_element_idx_]
                    //                                                  .reference_line_;
                    //         bc::float32_t obj_lane_ref_point_x =
                    //             association_refline.ref_line_pts_[obj_lane_current_point_idx_].pos_.x;
                    //         bc::float32_t obj_lane_ref_point_y =
                    //             association_refline.ref_line_pts_[obj_lane_current_point_idx_].pos_.y;
                    //         bc::float32_t obj_lane_distance_to_refline =
                    //             sqrt(obj_lane_ref_point_x * obj_lane_ref_point_x +
                    //                  obj_lane_ref_point_y * obj_lane_ref_point_y);
                    //         // Place filtered_prob to the correct position
                    //         if (distance_to_refline > obj_lane_distance_to_refline ||
                    //             fabsf(distance_to_refline - obj_lane_distance_to_refline) <= FLOAT32_EPSILON)
                    //         {
                    //           if (obj_lane < ::zone::data::em_data::kMaxLaneAssociationNum - 1)
                    //           {
                    //             (*em_collection_).agents_[obj_index[idx]].lane_association_[obj_lane + 1] =
                    //                 current_association;
                    //           }
                    //           if (obj_lane == 0)
                    //           {
                    //             current_association.probability_ = filtered_prob;
                    //             current_association.distance_to_left_border_ = distance_to_left_border;
                    //             current_association.distance_to_right_border_ = distance_to_right_border;
                    //             current_association.assigned_lane_idx_ = i;
                    //             current_association.assigned_element_idx_ = j;
                    //             current_association.nearest_ref_line_point_ = refline_points[idx];
                    //             current_association.valid_ = bc::true_v;
                    //           }
                    //         }
                    //         else
                    //         {
                    //           LaneAssociation& update_association =
                    //               (*em_collection_).agents_[obj_index[idx]].lane_association_[obj_lane + 1];
                    //           update_association.probability_ = filtered_prob;
                    //           update_association.distance_to_left_border_ = distance_to_left_border;
                    //           update_association.distance_to_right_border_ = distance_to_right_border;
                    //           update_association.assigned_lane_idx_ = i;
                    //           update_association.assigned_element_idx_ = j;
                    //           update_association.nearest_ref_line_point_ = refline_points[idx];
                    //           update_association.valid_ = bc::true_v;
                    //           break;
                    //         }
                    //       }
                    //       else
                    //       {
                    //         LaneAssociation& update_association =
                    //             (*em_collection_).agents_[obj_index[idx]].lane_association_[obj_lane + 1];
                    //         update_association.probability_ = filtered_prob;
                    //         update_association.distance_to_left_border_ = distance_to_left_border;
                    //         update_association.distance_to_right_border_ = distance_to_right_border;
                    //         update_association.assigned_lane_idx_ = i;
                    //         update_association.assigned_element_idx_ = j;
                    //         update_association.nearest_ref_line_point_ = refline_points[idx];
                    //         update_association.valid_ = bc::true_v;
                    //         break;
                    //       }
                    //     }
                    //   }
                    // }
                  }
                  else
                  {
                    /// TODO: Log invalid object length, object width or lane width error
                  }
                  // Set value to lane output
                  current_agent_projection.slt_pt_ = sl_points[idx];
                  current_agent_projection.idx_ = obj_index[idx];
                  current_agent_projection.id_ = (*em_collection_).agents_[current_agent_projection.idx_].id_;
                  current_agent_projection.valid_ = bc::true_v;
                  current_agent_projection.refline_pts_idx_ = refline_idx[idx];
                  idx++;
                }
                else
                {
                  current_agent_projection = AgentCurrentPosProjOnToRefLine();
                  temp_object_prob_collection[point_idx][i][j] = 0.f;
                }
              }
              else
              {
                current_agent_projection = AgentCurrentPosProjOnToRefLine();
                temp_object_prob_collection[point_idx][i][j] = 0.f;
              }
            }
          }
        }
        // Lane element is invalid, all related element prob should decrease to 0
        else
        {
          for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
          {
            bc::float32_t filtered_prob = 0.f;
            // In the case of lane change, all the probs should shift to left or right by one lane
            if ((lane_change_status == -1 && i > ::zone::data::em_data::kLLLane) ||
                (lane_change_status == 1 && i < ::zone::data::em_data::kRRLane))
            {
              filtered_prob =
                  LowPass(0.f, object_prob_collection[agent_idx][i + lane_change_status][j], kProbLowpass, cycle_time_);
            }
            else
            {
              filtered_prob = LowPass(0.f, object_prob_collection[agent_idx][i][j], kProbLowpass, cycle_time_);
            }
            temp_object_prob_collection[agent_idx][i][j] = filtered_prob;
          }
          for (bc::uint8_t t_agent_idx = 0; t_agent_idx < ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum;
               t_agent_idx++)
          {
            current_lane.lane_elements_[j].reference_line_.agents_projected_traj_[t_agent_idx] =
                AgentCurrentPosProjOnToRefLine();
          }
        }
      }
    }
    // Lane is invalid, all related lane and element prob should decrease to 0
    else
    {
      if (lane_change_status == -1)
      {
        if (i > 0)
        {
          for (bc::uint8_t element_idx = 0; element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++element_idx)
          {
            for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
            {
              bc::float32_t filtered_prob =
                  LowPass(0.f, object_prob_collection[agent_idx][i + lane_change_status][element_idx], kProbLowpass,
                          cycle_time_);
              temp_object_prob_collection[agent_idx][i][element_idx] = filtered_prob;
            }
          }
        }
        else
        {
          for (bc::uint8_t element_idx = 0; element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++element_idx)
          {
            for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
            {
              temp_object_prob_collection[agent_idx][i][element_idx] = 0.f;
            }
          }
        }
      }
      else if (lane_change_status == 1)
      {
        if (i < ::zone::data::em_data::kRRLane)
        {
          for (bc::uint8_t element_idx = 0; element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++element_idx)
          {
            for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
            {
              bc::float32_t filtered_prob =
                  LowPass(0.f, object_prob_collection[agent_idx][i + lane_change_status][element_idx], kProbLowpass,
                          cycle_time_);
              temp_object_prob_collection[agent_idx][i][element_idx] = filtered_prob;
            }
          }
        }
        else
        {
          for (bc::uint8_t element_idx = 0; element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++element_idx)
          {
            for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
            {
              temp_object_prob_collection[agent_idx][i][element_idx] = 0.f;
            }
          }
        }
      }
      else
      {
        for (bc::uint8_t element_idx = 0; element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++element_idx)
        {
          for (bc::uint8_t agent_idx = 0; agent_idx < ::zone::data::em_data::kFusMaxObjNum; ++agent_idx)
          {
            bc::float32_t filtered_prob =
                LowPass(0.f, object_prob_collection[agent_idx][i][element_idx], kProbLowpass, cycle_time_);
            temp_object_prob_collection[agent_idx][i][element_idx] = filtered_prob;
          }
        }
      }
      for (bc::uint8_t t_element_idx = 0; t_element_idx < ::zone::data::em_data::kMaxElemNumInOneLane; ++t_element_idx)
      {
        for (bc::uint8_t t_agent_idx = 0; t_agent_idx < ::zone::data::em_data::kAgentsProjectedToOneRefLineMaxNum;
             t_agent_idx++)
        {
          current_lane.lane_elements_[t_element_idx].reference_line_.agents_projected_traj_[t_agent_idx] =
              AgentCurrentPosProjOnToRefLine();
        }
      }
    }
  }
  // Normalize the probs of each single object
  // for (bc::uint8_t idx = 0; idx < kFusMaxObjNum; ++idx) {
  //   if ((*em_collection_).agents_[idx].valid_) {
  //     bc::float32_t sum_of_prob = prob_accumulator[idx];
  //     if (sum_of_prob > FLOAT32_EPSILON) {
  //       for (bc::uint8_t obj_lane = 0; obj_lane < ::zone::data::em_data::kMaxLaneAssociationNum; ++obj_lane) {
  //         (*em_collection_).agents_[idx].lane_association_[obj_lane].probability_ /= sum_of_prob;
  //       }
  //     }
  //   }
  // }
  // Update prob collection for using in the next cycle
  object_prob_collection = temp_object_prob_collection;
}
}  // namespace environment_model
}  // namespace zone
