#ifndef POINT_BASED_OLR_H_INCLUDED_
#define POINT_BASED_OLR_H_INCLUDED_

#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/basic_type.h"
#include "common/data/em_data.h"
#include "common/data/em_param.h"
#include "coordinate_angle_converter.h"
#include "environment_model_internal.h"

namespace zone {
namespace environment_model {

using ::zone::data::em_data::EmData;
using ::zone::data::em_data::LaneData;
using ::zone::data::em_data::ReferenceLine;
using ::zone::data::em_data::LaneElement;
using ::zone::data::em_data::AgentCurrentPosProjOnToRefLine;
using ::zone::data::em_data::LaneAssociation;
using ::zone::data::em_data::TrafficAgentData;
using zone::common::OlrParam;
using ::bc::G_PI;

class Olr
{
  Olr(const Olr&) = delete;
  void operator=(const Olr&) = delete;

 public:
  Olr(EmData& em_collection, EmData& em_collection_lst_cycle_cs);

  ~Olr() = default;

  void SetInput(bc::float64_t cycle_time) { cycle_time_ = cycle_time; }

  void Run();

  void SetParam(OlrParam& param)
  {
    kProbDeadzone = max(FLOAT32_EPSILON, param.agent_prob_deadzone);
    kProbLowpass = param.agent_prob_lowpass_factor;
    kSlLLowpass = param.agent_l_lowpass_factor;
    kSlVlLowpass = param.agent_vl_lowpass_factor;
  };

  void Reset(EmData& em_collection, EmData& em_collection_lst_cycle_cs)
  {
    em_collection_ = &em_collection;
    em_collection_lst_cycle_ = &em_collection_lst_cycle_cs;
    kProbDeadzone = 0.f;
    kProbLowpass = 0.f;
    kSlLLowpass = 0.f;
    kSlVlLowpass = 0.f;
    last_cycle_ego_lc_cnts_ = 0;
    cycle_time_ = 0.f;
    for (bc::uint8_t t_i_u8 = 0; t_i_u8 < ::zone::data::em_data::kFusMaxObjNum; t_i_u8++)
    {
      obj_container[t_i_u8] = TrafficAgentData();
      obj_index[t_i_u8] = 0;
      last_cycle_obj_ids[t_i_u8] = 0.f;
      for (bc::uint8_t t_j_u8 = 0; t_j_u8 < ::zone::data::em_data::kMaxLaneNum; t_j_u8++)
      {
        for (bc::uint8_t t_k_u8 = 0; t_k_u8 < ::zone::data::em_data::kMaxElemNumInOneLane; t_k_u8++)
        {
          object_prob_collection[t_i_u8][t_j_u8][t_k_u8] = 0.f;
        }
      }
    }
  }

 private:
  /** @brief Judge whether road structure has changed
   */
  void ElementMappingWithLstCycle();

  bc::bool_t AgentDistanceToBorder(const bc::float32_t& point_x, const bc::float32_t& point_y,
                                   const bc::float32_t& point_heading, const bc::float32_t& obj_x,
                                   const bc::float32_t& obj_y, const bc::float32_t& obj_width,
                                   const bc::float32_t& obj_length, const bc::float32_t& obj_heading,
                                   const bc::float32_t& lane_width, bc::float32_t& distance_of_left_to_left_border,
                                   bc::float32_t& distance_of_right_to_left_border,
                                   bc::float32_t& distance_of_left_to_right_border,
                                   bc::float32_t& distance_of_right_to_right_border, bc::float32_t& prob);

  /** @brief Store the valid objects to the local container
   *  @return boolean
   */
  bc::bool_t CollectValidObjects();

  /** @brief Reset the lane association data for the case that
   *  the highest probability to the object is around 0
   */
  void UpdateObjectAssociation();

  /** @brief Assign object to the given lane model;
   *  also calculate the probability of the object on each lane
   */
  void ObjectAssignment();

  /** @brief Map element with last cycle EM using element id
   */
  void ElementMapping(const LaneElement& current_element, bc::bool_t& is_old_element, bc::uint8_t& lst_lane_idx,
                      bc::uint8_t& lst_element_idx);

  EmData* em_collection_;
  EmData* em_collection_lst_cycle_;
  bc::float32_t kProbDeadzone = 0.0f;
  bc::float32_t kProbLowpass = 0.0f;
  bc::float32_t kSlLLowpass = 0.0f;
  bc::float32_t kSlVlLowpass = 0.0f;
  bc::int32_t last_cycle_ego_lc_cnts_ = 0;
  bc::TCArray<bc::TCArray<LaneElementMapping, ::zone::data::em_data::kMaxElemNumInOneLane>,
              ::zone::data::em_data::kMaxLaneNum>
      road_struct_change_info_;
  bc::TCArray<bc::float32_t, kFusMaxObjNum> prob_max_ar_;
  bc::float64_t cycle_time_ = 0.f;
  bc::TFixedVector<TrafficAgentData, ::zone::data::em_data::kFusMaxObjNum> obj_container;
  bc::TFixedVector<bc::uint8_t, ::zone::data::em_data::kFusMaxObjNum> obj_index;
  bc::TCArray<bc::uint16_t, ::zone::data::em_data::kFusMaxObjNum> last_cycle_obj_ids;
  bc::TCArray<bc::TCArray<bc::TCArray<bc::float32_t, ::zone::data::em_data::kMaxElemNumInOneLane>,
                          ::zone::data::em_data::kMaxLaneNum>,
              ::zone::data::em_data::kFusMaxObjNum>
      object_prob_collection;
};
}  // namespace environment_model
}  // namespace zone

#endif
