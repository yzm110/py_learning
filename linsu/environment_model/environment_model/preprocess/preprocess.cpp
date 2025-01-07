#include "preprocess.h"

namespace zone {
namespace environment_model {

Preprocess::Preprocess(EmData& em_collection) : em_collection_ptr_(&em_collection) {}

void Preprocess::Run(const ObjTracks& per_obj_cs, const LaneMarkings& per_lane_cs,
                     const StaticObjectData& per_static_obj_cs, const FreespaceList& per_freespace_cs,
                     const EgoMotionData& ego_motion_cs, const EhrToEmData& ehr2em_cs, const NavigationData& navi_cs,
                     const NavigationPlusData& navi_plus_cs, const WorkCondition& world_info_cs,
                     const EmData& em_output_lst1_cs, const EmParam& param_st,
                     const bc::TCArray<bc::bool_t, kNumInputChk> input_update_ar, const bc::float32_t time_cycle_f,
                     const StEndIdxPack& last_em_st_end_idx_, const bc::float64_t& time_stamp_f64,
                     BoundaryPack& boundary_pack_cs, SemanticPack& semantic_pack_cs, EmData& em_output_cs,
                     StaticObjProcess& static_obj_process_cs)
{
  /** 1. input validity check*/
  InputValidityCheck(per_obj_cs, per_lane_cs, per_static_obj_cs, navi_cs, navi_plus_cs, per_freespace_cs, ego_motion_cs,
                     ehr2em_cs, world_info_cs, param_st, input_update_ar, time_cycle_f);

  /** 2. calc x_ref_coordinate and determine how to discrete boundaries */
  CalcDiscreteCoordinate(param_st);

  /** 3. process ego status from ego motion */
  EgoMotionProcess(ego_motion_cs, param_st.prepro_param, time_cycle_f);

  /** 4. process object from perception */
  PerObjProcess(per_obj_cs, time_cycle_f, param_st.prepro_param);

  /** 4.5. process static objects */
  static_obj_process_cs.StaticObjMatching(input_diag_manager_ar_[kInputChkPerStaticObj], per_static_obj_cs);

  /** 5. match per lane and map lane to the EM output lane structure of last cycle*/
  PerMapMatching(em_output_lst1_cs, ehr2em_cs, per_lane_cs, time_cycle_f, ego_motion_cs, map_adapter_cs_);

  /** 6. process lane from map */
  MapProcess(ehr2em_cs, em_output_lst1_cs.lanes_, param_st.prepro_param, time_cycle_f, map_adapter_cs_);

  /** 7. track ego vehicle trajectory */
  EgoLaneTrack(em_output_lst1_cs, time_cycle_f, last_em_st_end_idx_, param_st.prepro_param, map_adapter_cs_);

  /// TODO: 识别Y型分叉并将左右边线处理成左左,右或者左,右右, 方便后续处理
  /// TODO: 非平行假设相关函数
  // ScenarioRecognition();

  /** 8. process lane from perception */
  PerLaneProcess(per_lane_cs, em_output_lst1_cs.lanes_, param_st.prepro_param, time_cycle_f,
                 ego_motion_cs.twist.linear.x);

  /** 9. perform map&per similiarity check */
  MapPerCrossCheck(map_lane_collection_cs_, per_lane_collection_cs_, param_st, time_cycle_f);

  /** 10. Process World Info from perception */
  WorldInfoProcess(world_info_cs, em_output_lst1_cs);

  /** 11. track virtual lane derived from leading vehicle */
  LdVehLaneTrack(obj_collection_cs_, em_output_lst1_cs, time_cycle_f, param_st.prepro_param);

  /** 12. repack boundary*/
  BoundaryRepack(ehr2em_cs, map_adapter_cs_, param_st.prepro_param.joint_similarity_threshold_f, boundary_pack_cs);

  /** 13. repack semantic info */
  SemanticRepack(per_static_obj_cs, map_adapter_cs_, navi_plus_cs, semantic_pack_cs);

  /** 14. static object info */
  StaticObjInfoTransmission(per_static_obj_cs);

  /** 15. diagnostic management*/
  DiagnosticManagement(time_stamp_f64);

  /** 16. output diagnosis information */
  OutputDiagInfo(input_diag_manager_ar_, em_output_cs);

  /** 15. navi guide information */
  NaviGuideInfo(navi_plus_cs, em_output_lst1_cs);

  /** 17.road edge */
  RoadedgeStore(per_lane_cs);
}

void Preprocess::InputValidityCheck(const ObjTracks& per_obj_cs, const LaneMarkings& per_lane_cs,
                                    const StaticObjectData& per_static_obj_cs, const NavigationData& navi_cs,
                                    const NavigationPlusData& navi_plus_cs, const FreespaceList& per_freespace_cs,
                                    const EgoMotionData& ego_motion_cs, const EhrToEmData& ehr2em_cs,
                                    const WorkCondition& world_info_cs, const EmParam& param_st,
                                    const bc::TCArray<bc::bool_t, kNumInputChk> input_update_ar,
                                    const bc::float32_t time_cycle_f)
{
  bc::uint16_t t_enable_cfg_u16 = param_st.prepro_param.validity_check_enable_u16;
  input_diag_manager_lst1_ar_ = input_diag_manager_ar_;
  if (GetBitU16(t_enable_cfg_u16, kInputChkPerObj) == bc::true_v)
  {
    ValidityCheckPerObj(per_obj_cs, time_cycle_f, input_update_ar[kInputChkPerObj]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkPerObj].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkPerLane) == bc::true_v)
  {
    ValidityCheckPerLane(per_lane_cs, time_cycle_f, input_update_ar[kInputChkPerLane]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkPerLane].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkPerFreeSp) == bc::true_v)
  {
    ValidityCheckPerFreeSpace(per_freespace_cs, time_cycle_f, input_update_ar[kInputChkPerFreeSp]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkPerFreeSp].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkPerStaticObj) == bc::true_v)
  {
    ValidityCheckPerStaticObj(per_static_obj_cs, time_cycle_f, input_update_ar[kInputChkPerStaticObj]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkPerStaticObj].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkNavi) == bc::true_v)
  {
    ValidityCheckNavi(navi_cs, time_cycle_f, input_update_ar[kInputChkNavi]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkNavi].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkNaviPlus) == bc::true_v)
  {
    ValidityCheckNaviPlus(navi_plus_cs, time_cycle_f, input_update_ar[kInputChkNaviPlus]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkNaviPlus].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkMap) == bc::true_v)
  {
    ValidityCheckMap(ehr2em_cs, time_cycle_f, input_update_ar[kInputChkMap]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkMap].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkEgoMotion) == bc::true_v)
  {
    ValidityCheckEgoMotion(ego_motion_cs, time_cycle_f, input_update_ar[kInputChkEgoMotion]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkEgoMotion].Reset();
  }

  if (GetBitU16(t_enable_cfg_u16, kInputChkWorldInfo) == bc::true_v)
  {
    ValidityCheckWorldInfo(world_info_cs, time_cycle_f, input_update_ar[kInputChkWorldInfo]);
  }
  else
  {
    input_diag_manager_ar_[kInputChkWorldInfo].Reset();
  }
}

void Preprocess::CalcDiscreteCoordinate(const EmParam& param_st)
{
  /** 0. Check input validity status, then return different result.
   *
   *  currently just check EgoMotion validity status.
   *  if kErrCfm, return Zeros;
   *  if kErrWrn, return Static default result;
   *  if kNormal, return Dynamic adaptive result;
   *
   */
  const DiagStatus& t_ego_diag_en =
      input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus();  ///< Egomotion status
  bc::float32_t t_dx_f = 2.0;                                                  ///< Discreting step.
  bc::float32_t t_vel_bm_f = param_st.prepro_param.velocity_benchmark_f;
  if (t_vel_bm_f < FLOAT32_EPSILON)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Param Error in EM CalcDiscreteCoordinate!!! " << t_vel_bm_f << std::endl;
#endif
    t_vel_bm_f = 16.67;
  }
  switch (t_ego_diag_en)
  {
    case DiagStatus::kErrCfm:
      /** Zeros: */
      t_dx_f = 2.0;
      break;
    case DiagStatus::kErrWrn:
      /** Static default result:
       *
       *  the equal interval discrete method is adopted.
       *  set x_ref_ar_ as a uniform discreting array from -40.0 to 100.0, with a step of 2.0.
       *
       */
      // t_dx_f = 2.0;
      DynamicAdaptiveCoordinate(t_dx_f, t_vel_bm_f);
      break;
    case DiagStatus::kNormal:
      /** Dynamic adaptive result:
       *
       *  method based on continuity.
       *  calc scaling factor for discreting step according to the radio of ego velocity and one benchmark.
       *  the scaling facctor should have upper and lower bounds.
       *
       */
      DynamicAdaptiveCoordinate(t_dx_f, t_vel_bm_f);
      break;
    default:
      t_dx_f = 2.0;
      break;
  }

  /** 1. Assignment for results.
   *
   *  return x_ref_ar_ for road segmentation in front and rear of ego.
   *
   */
  x_ref_ar_[0] = static_cast<bc::float32_t>(-t_dx_f * kMaxNumRearBoundary);
  for (bc::uint8_t t_idx_u8 = 1; t_idx_u8 < kMaxNumBoundary; ++t_idx_u8)
  {
    x_ref_ar_[t_idx_u8] = static_cast<bc::float32_t>(t_dx_f + x_ref_ar_[t_idx_u8 - 1]);
  }
}

void Preprocess::EgoMotionProcess(const EgoMotionData& ego_motion_cs, const PreProParam prep_param_st,
                                  const bc::float32_t time_cycle_f)
{
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkEgoMotion) == bc::false_v)
  {
    return;
  }

  if (input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      input_diag_manager_ar_[kInputChkEgoMotion].status_man_cs_.GetStatus() != DiagStatus::kErrCfm)
  {
    ego_pose_collection_cs_.AddEgoPose(ego_motion_cs, time_cycle_f);
  }
  else
  {
    ego_pose_collection_cs_.ClearEgoPose();
  }

  CurvParam<bc::float32_t, 2> t_curve_ego_cur;
  t_curve_ego_cur.x_arr = {0.0015, 0.003};
  t_curve_ego_cur.y_arr = {0, 1};
  bc::float32_t t_ego_cuv_rate_f =
      InterpCurv(t_curve_ego_cur, fabsf(ego_pose_collection_cs_.GetCurrentEgoPose().GetCurvature()));
  bc::float32_t t_ego_cuv_f = t_ego_cuv_rate_f * ego_pose_collection_cs_.GetCurrentEgoPose().GetCurvature();
  if (ego_curve_filter_b_)
  {
    filtered_ego_curve_ = LowPass(t_ego_cuv_f, filtered_ego_curve_, 0.5, time_cycle_f);
  }
  else
  {
    filtered_ego_curve_ = t_ego_cuv_f;
    ego_curve_filter_b_ = bc::true_v;
  }
  ego_pose_collection_cs_.GetCurrentEgoPose().SetCurvature(filtered_ego_curve_);
}

void Preprocess::PerObjProcess(const ObjTracks& per_obj_cs, const bc::float32_t time_cycle_f,
                               const PreProParam prep_param_st)
{
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkPerObj) == bc::false_v)
  {
    return;
  }
  (*em_collection_ptr_).obj_exposure_timestamp_ = per_obj_cs.time_stamp;
  const DiagStatus& t_obj_diag_en =
      input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_.GetStatus();  ///< Perception objects status
  const DiagStatus& t_ego_diag_en =
      input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus();  ///< Egomotion status

  switch (t_obj_diag_en)
  {
    case DiagStatus::kErrCfm:
      /** Reset internal collection data.*/
      obj_collection_cs_ = PerObjCollection();
      break;

    case DiagStatus::kErrWrn:
      /** Track obj_collection_cs_  only when EgoMotion status is kNormal.
       *
       *  set timestamp of obj_cs ts_k1 + dt.
       *  use EM algo cycle time as dt to change obj_cs pose.
       *  set source status of obj_cs kEmInternalTrack.
       *
       */
      if (t_ego_diag_en != DiagStatus::kNormal)
      {
        obj_collection_cs_ = PerObjCollection();
      }
      else
      {
        // obj_collection_cs_.time_stamp_f_ += time_cycle_f;
        for (bc::int8_t t_idx_u8 = 0; t_idx_u8 < kFusMaxObjNum; ++t_idx_u8)
        {
          PerObjTrack(time_cycle_f, obj_collection_cs_.objs_ar_[t_idx_u8]);
        }
      }
      break;

    case DiagStatus::kNormal:
      /** Store from external collection data to internal collection data.*/
      obj_collection_cs_ = PerObjCollection();
      StorePerObjsToInternal(per_obj_cs);
      /** TODO: Mapping first, then store, if some obj is not found, it should be tracked.*/
      break;

    default:
      break;
  }
}

void Preprocess::PerLaneProcess(const LaneMarkings& per_lane_cs,
                                const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                                const PreProParam prep_param_st, const bc::float32_t time_cycle_f,
                                const bc::float32_t ego_vel_f)
{
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkPerLane) == bc::false_v)
  {
    return;
  }
  (*em_collection_ptr_).lane_exposure_timestamp_ = per_lane_cs.time_stamp;

  if (per_analy_b_)
  {
    /** 0. Store and matching perlane in PerLaneMapping() */
    PerLaneMapping(lst_perlane_cs_, ego_track_lane_collection_cs_.ego_track_point_ar_,
                   em_lane_lst1_ar[kHostLane].lane_elements_[0].left_boundary_.source_type_,
                   em_lane_lst1_ar[kHostLane].lane_elements_[0].right_boundary_.source_type_, time_cycle_f);
    /** 1. Deduct invalid boundary if lane valid. */
    DeductPerBoundary(em_lane_lst1_ar, time_cycle_f, ego_vel_f);
    /** 1.1 Update ref with new perception input. */
    DoRefPerceptionMatch(per_lane_collection_cs_);
    /** 1.2 Check parallel lane. */
    DetectParallelLane();
    /** 2. Optional, should do if no hdmap && per hostlane valid to generate parallel lane.*/
    ParallelLaneGenerator();
    /** 3. Discrete each valid boundary according to x_ref_ar_ to per_point_ar_ in per_lane_collection_cs_. */
    DiscretePerLane();
    /** 4. Optional, time sync for per lane, can be triggered by parameters. Currently treat it as an empty.*/
    TimeSyncPerLane();

    if (map_lane_change_s8 < 0)
    {
      for (bc::uint8_t t_lane_idx_u8 = 1; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
      {
        if (t_lane_idx_u8 + map_lane_change_s8 < 0)
        {
          per_lane_collection_cs_.per_point_ar_[t_lane_idx_u8] = LaneBoundaryPoint();
          per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8] = PerLaneCoeff();
        }
        else
        {
          per_lane_collection_cs_.per_point_ar_[t_lane_idx_u8 + map_lane_change_s8] =
              per_lane_collection_cs_.per_point_ar_[t_lane_idx_u8];
          per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 + map_lane_change_s8] =
              per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8];
        }
      }
      per_lane_collection_cs_.per_point_ar_[kRRLane] = LaneBoundaryPoint();
      per_lane_collection_cs_.per_coeff_ar_[kRRLane] = PerLaneCoeff();
    }
    else if (map_lane_change_s8 > 0)
    {
      for (bc::int8_t t_lane_idx_s8 = 3; t_lane_idx_s8 >= 0; t_lane_idx_s8--)
      {
        if (t_lane_idx_s8 + map_lane_change_s8 >= kMaxLaneNum)
        {
          per_lane_collection_cs_.per_point_ar_[t_lane_idx_s8] = LaneBoundaryPoint();
          per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_s8] = PerLaneCoeff();
        }
        else
        {
          per_lane_collection_cs_.per_point_ar_[t_lane_idx_s8 + map_lane_change_s8] =
              per_lane_collection_cs_.per_point_ar_[t_lane_idx_s8];
          per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_s8 + map_lane_change_s8] =
              per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_s8];
        }
      }
      per_lane_collection_cs_.per_point_ar_[kLLLane] = LaneBoundaryPoint();
      per_lane_collection_cs_.per_coeff_ar_[kLLLane] = PerLaneCoeff();
    }
    else
    {
      /// do nothing
    }
    if (modified_ref_b)
    {
      lst_left_line_id_s32_ = per_lane_cs.lines[0].line_id;
      lst_right_line_id_s32_ = per_lane_cs.lines[1].line_id;
      lst_per_host_lane_coeff_cs_ = per_lane_collection_cs_.per_coeff_ar_[kHostLane];
    }
  }
  else
  {
    per_lane_collection_cs_ = PerLaneCollection();
  }
}

MapPerSimiliarityCheckState Preprocess::MapPerSimiliarityStateUpdate(const MapPerSimiliarityCheckState state,
                                                                     const bc::float32_t time_cycle_f,
                                                                     const bc::bool_t valid_similarity_b,
                                                                     bc::float32_t& accumu_time)
{
  MapPerSimiliarityCheckState t_state = state;
  switch (state)
  {
    case MapPerSimiliarityCheckState::kOff:
    {
      if (valid_similarity_b)
      {
        t_state = MapPerSimiliarityCheckState::kStandby;
      }
      else
      {
        accumu_time = 0.f;
        t_state = MapPerSimiliarityCheckState::kOff;
      }
      break;
    }

    case MapPerSimiliarityCheckState::kStandby:
    {
      if (valid_similarity_b)
      {
        accumu_time += time_cycle_f;
        if (accumu_time >= kActivePreparationTime)
        {
          t_state = MapPerSimiliarityCheckState::kActive;
        }
        else
        {
        }
      }
      else
      {
        accumu_time = 0.f;
        t_state = MapPerSimiliarityCheckState::kOff;
      }
      break;
    }

    case MapPerSimiliarityCheckState::kActive:
    {
      if (valid_similarity_b)
      {
        t_state = MapPerSimiliarityCheckState::kActive;
      }
      else
      {
        accumu_time = 0.f;
        t_state = MapPerSimiliarityCheckState::kOff;
      }
      break;
    }
    default:
    {

      accumu_time = 0.f;
      t_state = MapPerSimiliarityCheckState::kOff;
      break;
    }
  }

  return t_state;
}

void Preprocess::MapPerCrossCheck(const MapLaneCollection& map_lane_data, const PerLaneCollection& per_lane_data,
                                  const EmParam& param_st, bc::float32_t time_cycle_f)
{
  // per_lane_data是感知边线被补过的状态
  const LaneElementPoint& map_lane_element = map_lane_data.map_point_ar_[2];
  const LaneBoundaryPoint& per_lane_element = per_lane_data.per_point_ar_[2];
  bc::float32_t t_thresh_f = 0.3f;
  bc::float32_t t_cur_left_map_per_similarity_f = 0.f;
  bc::float32_t t_cur_right_map_per_similarity_f = 0.f;

  bc::float32_t t_left_p_a_f = 0.f;
  bc::float32_t t_left_p_b_f = 0.f;
  bc::float32_t t_right_p_a_f = 0.f;
  bc::float32_t t_right_p_b_f = 0.f;
  bc::float32_t t_left_prior_map_per_similarity_f = 0.99f;
  bc::float32_t t_right_prior_map_per_similarity_f = 0.99f;
  //到boundary一层
  bc::bool_t valid_left_similiarity_b_ =
      map_lane_element.valid_b_ && per_lane_element.valid_b_ && map_lane_element.lane_element_point_ar_[0].valid_b_ &&
      map_lane_element.lane_element_point_ar_[0].left_point_cs_.valid_b_ && per_lane_element.left_point_cs_.valid_b_ &&
      per_lane_element.left_point_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual;
  bc::bool_t valid_right_similiarity_b_ =
      map_lane_element.valid_b_ && per_lane_element.valid_b_ && map_lane_element.lane_element_point_ar_[0].valid_b_ &&
      map_lane_element.lane_element_point_ar_[0].right_point_cs_.valid_b_ &&
      per_lane_element.right_point_cs_.valid_b_ &&
      per_lane_element.right_point_cs_.boundary_type_en_ != LaneBoundaryType::kVirtual;

  bc::uint8_t t_left_max_start_idx_u8 =
      bc::max(per_lane_element.left_point_cs_.dy_start_idx_u8_,
              map_lane_element.lane_element_point_ar_[0].left_point_cs_.dy_start_idx_u8_);
  bc::uint8_t t_left_min_end_idx_u8 = bc::min(per_lane_element.left_point_cs_.dy_end_idx_u8_,
                                              map_lane_element.lane_element_point_ar_[0].left_point_cs_.dy_end_idx_u8_);
  valid_left_similiarity_b_ = valid_left_similiarity_b_ && t_left_max_start_idx_u8 < t_left_min_end_idx_u8;

  bc::uint8_t t_right_max_start_idx_u8 =
      bc::max(per_lane_element.right_point_cs_.dy_start_idx_u8_,
              map_lane_element.lane_element_point_ar_[0].right_point_cs_.dy_start_idx_u8_);
  bc::uint8_t t_right_min_end_idx_u8 =
      bc::min(per_lane_element.right_point_cs_.dy_end_idx_u8_,
              map_lane_element.lane_element_point_ar_[0].right_point_cs_.dy_end_idx_u8_);
  valid_right_similiarity_b_ = valid_right_similiarity_b_ && t_right_max_start_idx_u8 < t_right_min_end_idx_u8;

  left_similiarity_check_state_ = MapPerSimiliarityStateUpdate(left_similiarity_check_state_, time_cycle_f,
                                                               valid_left_similiarity_b_, left_accumu_time_);
  right_similiarity_check_state_ = MapPerSimiliarityStateUpdate(right_similiarity_check_state_, time_cycle_f,
                                                                valid_right_similiarity_b_, right_accumu_time_);

  switch (left_similiarity_check_state_)
  {
    case MapPerSimiliarityCheckState::kOff:
      break;
    case MapPerSimiliarityCheckState::kStandby:
      left_map_per_similarity_f_ = CheckMapPerLaneSimilarity(
          per_lane_element.left_point_cs_.dy_ar_, map_lane_element.lane_element_point_ar_[0].left_point_cs_.dy_ar_,
          param_st, t_left_max_start_idx_u8, t_left_min_end_idx_u8);
      left_post_map_per_similarity_f_ = 0.99f;
      break;
    case MapPerSimiliarityCheckState::kActive:
      t_cur_left_map_per_similarity_f = CheckMapPerLaneSimilarity(
          per_lane_element.left_point_cs_.dy_ar_, map_lane_element.lane_element_point_ar_[0].left_point_cs_.dy_ar_,
          param_st, t_left_max_start_idx_u8, t_left_min_end_idx_u8);
      left_map_per_similarity_f_ = LowPass(t_cur_left_map_per_similarity_f, left_map_per_similarity_f_,
                                           kPerMapLaneCheckTimeFactor, time_cycle_f);

      t_left_prior_map_per_similarity_f = left_post_map_per_similarity_f_;
      t_left_p_a_f = t_cur_left_map_per_similarity_f * (t_left_prior_map_per_similarity_f * kPSimiliarity +
                                                        (1 - t_left_prior_map_per_similarity_f) * kPNSimiliarity);
      t_left_p_b_f = (1 - t_cur_left_map_per_similarity_f) * ((1 - t_left_prior_map_per_similarity_f) * kPSimiliarity +
                                                              t_left_prior_map_per_similarity_f * kPNSimiliarity);
      left_post_map_per_similarity_f_ = t_left_p_a_f / bc::max(t_left_p_a_f + t_left_p_b_f, FLOAT32_EPSILON);
      break;
    default:
      break;
  }

  switch (right_similiarity_check_state_)
  {
    case MapPerSimiliarityCheckState::kOff:
      break;
    case MapPerSimiliarityCheckState::kStandby:
      right_map_per_similarity_f_ = CheckMapPerLaneSimilarity(
          per_lane_element.right_point_cs_.dy_ar_, map_lane_element.lane_element_point_ar_[0].right_point_cs_.dy_ar_,
          param_st, t_right_max_start_idx_u8, t_right_min_end_idx_u8);
      right_post_map_per_similarity_f_ = 0.99f;
      break;
    case MapPerSimiliarityCheckState::kActive:
      t_cur_right_map_per_similarity_f = CheckMapPerLaneSimilarity(
          per_lane_element.right_point_cs_.dy_ar_, map_lane_element.lane_element_point_ar_[0].right_point_cs_.dy_ar_,
          param_st, t_right_max_start_idx_u8, t_right_min_end_idx_u8);
      right_map_per_similarity_f_ = LowPass(t_cur_right_map_per_similarity_f, right_map_per_similarity_f_,
                                            kPerMapLaneCheckTimeFactor, time_cycle_f);

      t_right_prior_map_per_similarity_f = right_post_map_per_similarity_f_;
      t_right_p_a_f = t_cur_right_map_per_similarity_f * (t_right_prior_map_per_similarity_f * kPSimiliarity +
                                                          (1 - t_right_prior_map_per_similarity_f) * kPNSimiliarity);
      t_right_p_b_f =
          (1 - t_cur_right_map_per_similarity_f) * ((1 - t_right_prior_map_per_similarity_f) * kPSimiliarity +
                                                    t_right_prior_map_per_similarity_f * kPNSimiliarity);
      right_post_map_per_similarity_f_ = t_right_p_a_f / bc::max(t_right_p_a_f + t_right_p_b_f, FLOAT32_EPSILON);

      break;
    default:
      break;
  }
}

bc::float32_t Preprocess::CheckMapPerLaneSimilarity(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& per_boundary,
                                                    const bc::TCArray<bc::float32_t, kMaxNumBoundary>& ref_boundary,
                                                    const EmParam& param_st, const bc::uint8_t start_index,
                                                    bc::uint8_t end_index)
{
  // raw likelihood = 1/(sqrt(2*bc::G_PI)*t_sigma_f)*exp(-t_dy_diff_f*t_dy_diff_f/(2*t_sigma_f*t_sigma_f)),max
  // likelihood = 1/(sqrt(2*bc::G_PI)*t_sigma_f);divide and simplify to obtain the final likelihood expression
  // :exp(-t_dy_diff_f*t_dy_diff_f/(2*t_sigma_f*t_sigma_f))
  // bc::float32_t t_map_sigma_f = 0.15f;
  // bc::float32_t t_per_sigma_f = 0.1f;
  // bc::float32_t t_sigma_f = sqrt(pow(t_map_sigma_f,2)+pow(t_per_sigma_f,2));
  bc::float32_t t_dy_diff_f = 0;
  bc::float32_t t_s_scale_f = 30;
  bc::float32_t t_likelihood_f = 0.f;
  bc::float32_t t_weight_f = 0.f;
  bc::float32_t t_weight_sum_f = 0.f;
  bc::float32_t t_weight_likelihood_sum_f = 0.f;
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_s_ar;
  bc::float32_t t_per_front_lateral_sigma_f = param_st.cross_check_param_st_.p_per_front_lateral_sigma_f_;
  bc::float32_t t_per_back_lateral_sigma_f = param_st.cross_check_param_st_.p_per_back_lateral_sigma_f_;
  bc::float32_t t_map_heading_sigma_f = param_st.cross_check_param_st_.p_map_heading_sigma_f_;
  bc::float32_t t_map_lateral_sigma_f = param_st.cross_check_param_st_.p_map_lateral_sigma_f_;
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_map_sigma_ar = {0.f};
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_sigma_ar = {0.f};
  bc::float32_t t_dis_dead_zone = 0.1f;

  if (start_index >= kFrontStartIdx)
  {
    if (start_index == kFrontStartIdx)
    {
      t_s_ar[start_index] = 0.f;
    }
    else
    {
      t_s_ar[start_index] = sqrt(pow(ref_boundary[start_index] - ref_boundary[kFrontStartIdx], 2) +
                                 pow(x_ref_ar_[start_index] - x_ref_ar_[kFrontStartIdx], 2));
    }
    for (bc::uint8_t t_point_idx_u8 = start_index + 1; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 - 1] + sqrt(pow(ref_boundary[t_point_idx_u8 - 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(x_ref_ar_[t_point_idx_u8 - 1] - x_ref_ar_[t_point_idx_u8], 2));
    }
  }
  else if (end_index <= kFrontStartIdx)
  {
    if (end_index == kFrontStartIdx)
    {
      t_s_ar[end_index] = 0.f;
    }
    else
    {
      t_s_ar[end_index] = sqrt(pow(ref_boundary[end_index] - ref_boundary[kFrontStartIdx], 2) +
                               pow(x_ref_ar_[end_index] - x_ref_ar_[kFrontStartIdx], 2));
    }
    for (bc::int8_t t_point_idx_u8 = end_index - 1; t_point_idx_u8 >= start_index; --t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 + 1] + sqrt(pow(ref_boundary[t_point_idx_u8 + 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(x_ref_ar_[t_point_idx_u8 + 1] - x_ref_ar_[t_point_idx_u8], 2));
    }
  }
  else
  {
    t_s_ar[kFrontStartIdx] = 0.f;
    for (bc::uint8_t t_point_idx_u8 = kFrontStartIdx + 1; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 - 1] + sqrt(pow(ref_boundary[t_point_idx_u8 - 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(x_ref_ar_[t_point_idx_u8 - 1] - x_ref_ar_[t_point_idx_u8], 2));
    }
    for (bc::int8_t t_point_idx_u8 = kFrontStartIdx - 1; t_point_idx_u8 >= start_index; --t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 + 1] + sqrt(pow(ref_boundary[t_point_idx_u8 + 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(x_ref_ar_[t_point_idx_u8 + 1] - x_ref_ar_[t_point_idx_u8], 2));
    }
  }

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  {
    t_map_sigma_ar[t_idx_u8] =
        sqrt(pow(t_map_lateral_sigma_f, 2) + pow(x_ref_ar_[t_idx_u8] * tan(t_map_heading_sigma_f * bc::G_PI / 180), 2));
  }
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  {
    if (t_s_ar[t_idx_u8] <= kPerSigmaDividingDis)
    {
      t_sigma_ar[t_idx_u8] = sqrt(pow(t_per_front_lateral_sigma_f, 2) + pow(t_map_sigma_ar[t_idx_u8], 2));
    }
    else
    {
      t_sigma_ar[t_idx_u8] = sqrt(pow(t_per_back_lateral_sigma_f, 2) + pow(t_map_sigma_ar[t_idx_u8], 2));
    }
  }

  for (bc::uint8_t t_point_idx_u8 = start_index; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
  {
    t_dy_diff_f = bc::abs(ref_boundary[t_point_idx_u8] - per_boundary[t_point_idx_u8]);
    t_dy_diff_f -= t_dis_dead_zone;
    if (bc::abs(t_dy_diff_f - 0.f) < 1e-6)
    {
      t_dy_diff_f = 0.f;
    }
    t_likelihood_f = exp(-t_dy_diff_f * t_dy_diff_f / (2 * t_sigma_ar[t_point_idx_u8] * t_sigma_ar[t_point_idx_u8]));
    t_weight_f = exp(-t_s_ar[t_point_idx_u8] * t_s_ar[t_point_idx_u8] / (t_s_scale_f * t_s_scale_f));
    t_weight_sum_f += t_weight_f;
    t_weight_likelihood_sum_f += t_weight_f * t_likelihood_f;
  }
  t_weight_likelihood_sum_f = t_weight_likelihood_sum_f / t_weight_sum_f;

  return t_weight_likelihood_sum_f;
}

void Preprocess::DetectParallelLane()
{
  for (bc::uint8_t t_idx_u8 = 0U; t_idx_u8 < kMaxLaneNum; ++t_idx_u8)
  {
    if (per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].valid_b_)
    {
      PerLaneBoundaryCoeff& t_left_internal_ptr = per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].left_coeff_cs_;
      PerLaneBoundaryCoeff& t_right_internal_ptr = per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].right_coeff_cs_;
            if (t_left_internal_ptr.valid_b_ && t_right_internal_ptr.valid_b_)
      {
        if (t_idx_u8 <= kHostLane && t_left_internal_ptr.is_paralleled_b)
        {

          t_right_internal_ptr.is_paralleled_b = bc::true_v;
        }
        else if (t_idx_u8 >= kHostLane && t_right_internal_ptr.is_paralleled_b)
        {
          t_left_internal_ptr.is_paralleled_b = bc::true_v;
        }
        /// TODO: Consider general method to guess
      }
      /// TODO: 在非平行假设下需要删除以下代码
      // 强行构造平行车道模型
      if (t_left_internal_ptr.valid_b_)
      {
        t_left_internal_ptr.is_paralleled_b = bc::true_v;
      }
      if (t_right_internal_ptr.valid_b_)
      {
        t_right_internal_ptr.is_paralleled_b = bc::true_v;
      }
    }
  }
}

void Preprocess::ParallelLaneGenerator()
{
  /** 1. Generate centerline in hostlane.*/
  ClothoidModel t_coeff_cs;

  PerLaneBoundaryCoeff* t_left_internal_ptr = nullptr;
  PerLaneBoundaryCoeff* t_right_internal_ptr = nullptr;
  bc::float32_t normal_start_x_f = 0;
  bc::float32_t normal_end_x_f = 0;

  if (per_lane_collection_cs_.per_coeff_ar_[kHostLane].valid_b_)
  {
    t_left_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_);
    t_right_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_);
  }
  else
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Warning in Preprocess::ParallelLaneGenerator: Perception is normal but host lane is invalid!!!"
              << std::endl;
#endif
    return;
  }

  /** Use std and longth of boundaries for coeff ratio but c0.*/
  bc::TCArray<float32_t, 4> t_ratio_ar = {0.5, 0.5, 0.5, 0.5};  ///< c0,c1,c2,c3 ratio of host left boundary.
  bc::float32_t t_c1_std_sum_f = t_left_internal_ptr->std_c1_f_ + t_right_internal_ptr->std_c1_f_;
  bc::float32_t t_c2_std_sum_f = t_left_internal_ptr->std_c2_f_ + t_right_internal_ptr->std_c2_f_;
  bc::float32_t t_c3_std_sum_f = t_left_internal_ptr->std_c3_f_ + t_right_internal_ptr->std_c3_f_;
  if (t_c1_std_sum_f > FLOAT32_EPSILON) t_ratio_ar[1] = t_right_internal_ptr->std_c1_f_ / t_c1_std_sum_f;
  if (t_c2_std_sum_f > FLOAT32_EPSILON) t_ratio_ar[2] = t_right_internal_ptr->std_c2_f_ / t_c1_std_sum_f;
  if (t_c3_std_sum_f > FLOAT32_EPSILON) t_ratio_ar[3] = t_right_internal_ptr->std_c3_f_ / t_c1_std_sum_f;

  bc::float32_t t_left_long_sum_f = t_left_internal_ptr->dx_end_f_ - t_left_internal_ptr->dx_start_f_;
  bc::float32_t t_right_long_sum_f = t_right_internal_ptr->dx_end_f_ - t_right_internal_ptr->dx_start_f_;
    if (t_left_long_sum_f < FLOAT32_EPSILON || t_right_long_sum_f < FLOAT32_EPSILON)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Warning: Perlane is normal but longth of per host lane is zero !!!" << std::endl;
#endif
  }
  else
  {
    bc::float32_t t_left_ratio_f = t_left_long_sum_f / (t_left_long_sum_f + t_right_long_sum_f);
    bc::float32_t t_longth_ratio_f = 0.5f * (1 + std::sin(t_left_ratio_f * bc::G_PI - bc::G_PI_2));
    t_ratio_ar[1] = t_longth_ratio_f;
    t_ratio_ar[2] = t_longth_ratio_f;
    t_ratio_ar[3] = t_longth_ratio_f;
  }

  if (t_left_long_sum_f > t_right_long_sum_f)
  {
    normal_start_x_f = t_left_internal_ptr->dx_start_f_;
    normal_end_x_f = t_left_internal_ptr->dx_end_f_;
  }
  else
  {
    normal_start_x_f = t_right_internal_ptr->dx_start_f_;
    normal_end_x_f = t_right_internal_ptr->dx_end_f_;
  }

  /** Recalc coeff of centerline.*/
  t_coeff_cs.c0_position = t_ratio_ar[0] * t_left_internal_ptr->coeff_cs_.c0_position +
                           (1 - t_ratio_ar[0]) * t_right_internal_ptr->coeff_cs_.c0_position;
  t_coeff_cs.c1_heading_angle = t_ratio_ar[1] * t_left_internal_ptr->coeff_cs_.c1_heading_angle +
                                (1 - t_ratio_ar[1]) * t_right_internal_ptr->coeff_cs_.c1_heading_angle;
  t_coeff_cs.c2_curvature = t_ratio_ar[2] * t_left_internal_ptr->coeff_cs_.c2_curvature +
                            (1 - t_ratio_ar[2]) * t_right_internal_ptr->coeff_cs_.c2_curvature;
  t_coeff_cs.c3_curvature_derivative = t_ratio_ar[3] * t_left_internal_ptr->coeff_cs_.c3_curvature_derivative +
                                       (1 - t_ratio_ar[3]) * t_right_internal_ptr->coeff_cs_.c3_curvature_derivative;

  /** 2. All lanes are translated by host centerline and .*/
  for (bc::uint8_t t_idx_u8 = 0U; t_idx_u8 < kMaxLaneNum; ++t_idx_u8)
  {
    if (per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].valid_b_ == bc::false_v)
    {
      continue;
    }
    PerLaneBoundaryCoeff* t_left_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].left_coeff_cs_);
    PerLaneBoundaryCoeff* t_right_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[t_idx_u8].right_coeff_cs_);
    if (t_left_internal_ptr->valid_b_)
    {
      if (t_left_internal_ptr->is_paralleled_b)
      {
        t_left_internal_ptr->coeff_cs_.c1_heading_angle = t_coeff_cs.c1_heading_angle;
        t_left_internal_ptr->coeff_cs_.c2_curvature = t_coeff_cs.c2_curvature;
        t_left_internal_ptr->coeff_cs_.c3_curvature_derivative = t_coeff_cs.c3_curvature_derivative;
      }
      t_left_internal_ptr->dx_start_f_ = normal_start_x_f;
      t_left_internal_ptr->dx_end_f_ = normal_end_x_f;
    }
    if (t_right_internal_ptr->valid_b_)
    {
      if (t_right_internal_ptr->is_paralleled_b)
      {
        t_right_internal_ptr->coeff_cs_.c1_heading_angle = t_coeff_cs.c1_heading_angle;
        t_right_internal_ptr->coeff_cs_.c2_curvature = t_coeff_cs.c2_curvature;
        t_right_internal_ptr->coeff_cs_.c3_curvature_derivative = t_coeff_cs.c3_curvature_derivative;
      }
      t_right_internal_ptr->dx_start_f_ = normal_start_x_f;
      t_right_internal_ptr->dx_end_f_ = normal_end_x_f;
    }
  }
  return;
}

void Preprocess::MapProcess(const EhrToEmData& ehr2em_cs, const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                            const PreProParam prep_param_st, const bc::float32_t time_cycle_f,
                            EmAdapterHDmapData& map_adapter_cs)
{
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkMap) == bc::false_v)
  {
    return;
  }

  /** 0. Check input_diag_manager, and keep map_adapter as default value if any related errors detected. Only do
   * following process when input is valid. */
  /** 1. Map Adaption. TODO: need to be detialized */
  if (ehr_analy_b_)
  {
    /** 1.4 store */
    // ReorganizeMapData(map_adapter_cs);
    StoreHDMapLaneInfoToInternal(map_adapter_cs);
    // CheckJumpPoint();
  }
  // else if (input_diag_manager_ar_[kInputChkMap].status_man_cs_.status_en_ != DiagStatus::kErrCfm &&
  //          input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.status_en_ != DiagStatus::kErrCfm)
  // {
  //   /* Do nothing, keep last cycle value. */
  // }
  else
  {
    /* Reset stored map data. */
    map_lane_collection_cs_ = MapLaneCollection();
  }
}

/** check split and open and fix jump boundary*/
void Preprocess::CheckJumpPoint()
{
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8].lane_element_point_ar_[t_elem_idx_u8].valid_b_)
        {
          BoundaryPoint& t_lf_map_pt_cs =
              map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8].lane_element_point_ar_[t_elem_idx_u8].left_point_cs_;
          BoundaryPoint& t_rt_map_pt_cs = map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8]
                                              .lane_element_point_ar_[t_elem_idx_u8]
                                              .right_point_cs_;
          for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < t_lf_map_pt_cs.dy_end_idx_u8_; t_pt_idx_u8++)
          {
            if (fabsf(t_lf_map_pt_cs.dy_ar_[t_pt_idx_u8 + 1] - t_lf_map_pt_cs.dy_ar_[t_pt_idx_u8]) > 2)
            {
              t_lf_map_pt_cs.dy_start_idx_u8_ = t_pt_idx_u8 + 1;
              break;
            }
          }
          for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < t_rt_map_pt_cs.dy_end_idx_u8_; t_pt_idx_u8++)
          {
            if (fabsf(t_rt_map_pt_cs.dy_ar_[t_pt_idx_u8 + 1] - t_rt_map_pt_cs.dy_ar_[t_pt_idx_u8]) > 2)
            {
              t_rt_map_pt_cs.dy_start_idx_u8_ = t_pt_idx_u8 + 1;
              break;
            }
          }
        }
      }
    }
  }
}

void Preprocess::ReorganizeMapData(EmAdapterHDmapData& map_em_data)
{
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    ::zone::data::em_data::LaneData& cur_lane = map_em_data.lanes_[t_lane_idx_u8];
    if (cur_lane.lane_valid_ && cur_lane.lane_elements_[0].element_valid_ && cur_lane.lane_elements_[1].element_valid_)
    {
      for (bc::uint8_t t_seg_idx_u8 = 0; t_seg_idx_u8 < kMaxLaneTransitionSegsInOneLaneElement; t_seg_idx_u8++)
      {
        if (cur_lane.lane_elements_[0].reference_line_.lane_transition_dir_segs_[t_seg_idx_u8].lane_trans_dir_ ==
            LaneTransitionDirection::kSplitToRight)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            bc::uint16_t t_total_valid_num_u16 =
                std::min(cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.total_valid_number_,
                         cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.total_valid_number_);
            if (t_total_valid_num_u16 > kMaxBoundaryPoint || t_total_valid_num_u16 < 1)
            {
#ifdef STD_COUT_ENABLE
              std::cout << "Error in Preprocess::ReorganizeMapData: Lane valid number error!" << std::endl;
#endif
            }
            else
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < (bc::uint8_t)(t_total_valid_num_u16 - 1); t_pt_idx_u8++)
              {
                bc::float32_t delta_y = cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                        cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[t_pt_idx_u8].y;
                bc::float32_t delta_y_other =
                    cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[t_pt_idx_u8 + 1].y -
                    cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[t_pt_idx_u8].y;
                if (fabsf(delta_y) > 2.f && fabsf(delta_y_other) < 1.5f)
                {
                  // for (bc::uint8_t idx = 0; idx <= t_pt_idx_u8 + 1; idx++)
                  // {
                  //   cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[idx] =
                  //       cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[idx];
                  // }
                  cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.reserve_[0] =
                      cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.reserve_[0] = t_pt_idx_u8 + 1;
                  cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.total_valid_number_ =
                      t_total_valid_num_u16 - (t_pt_idx_u8 + 1);
                  cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.total_valid_number_ =
                      t_total_valid_num_u16 - (t_pt_idx_u8 + 1);
                  if (t_total_valid_num_u16 - (t_pt_idx_u8 + 1) > kMaxBoundaryPoint)
                  {
#ifdef STD_COUT_ENABLE
                    std::cout << "Error in Preprocess::ReorganizeMapData: Split Right error!" << std::endl;
#endif
                  }
                }
              }
            }
          }
        }
        if (cur_lane.lane_elements_[0].reference_line_.lane_transition_dir_segs_[t_seg_idx_u8].lane_trans_dir_ ==
            LaneTransitionDirection::kSplitToLeft)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            bc::uint16_t t_total_valid_num_u16 =
                std::min(cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.total_valid_number_,
                         cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.total_valid_number_);
            if (t_total_valid_num_u16 > kMaxBoundaryPoint || t_total_valid_num_u16 < 1)
            {
#ifdef STD_COUT_ENABLE
              std::cout << "Error in Preprocess::ReorganizeMapData: Lane valid number error!" << std::endl;
#endif
            }
            else
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < (bc::uint8_t)(t_total_valid_num_u16 - 1); t_pt_idx_u8++)
              {
                bc::float32_t delta_y = cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                        cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[t_pt_idx_u8].y;
                bc::float32_t delta_y_other =
                    cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[t_pt_idx_u8 + 1].y -
                    cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[t_pt_idx_u8].y;
                if (fabsf(delta_y) > 2.f && fabsf(delta_y_other) < 1.5f)
                {
                  // for (bc::uint8_t idx = 0; idx <= t_pt_idx_u8 + 1; idx++)
                  // {
                  //   cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.pts_[idx] =
                  //       cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.pts_[idx];
                  // }
                  cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.reserve_[0] =
                      cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.reserve_[0] = t_pt_idx_u8 + 1;
                  cur_lane.lane_elements_[t_ele_idx_u8].left_boundary_.total_valid_number_ =
                      t_total_valid_num_u16 - (t_pt_idx_u8 + 1);
                  cur_lane.lane_elements_[t_ele_idx_u8].right_boundary_.total_valid_number_ =
                      t_total_valid_num_u16 - (t_pt_idx_u8 + 1);
                  if (t_total_valid_num_u16 - (t_pt_idx_u8 + 1) > kMaxBoundaryPoint)
                  {
#ifdef STD_COUT_ENABLE
                    std::cout << "Error in Preprocess::ReorganizeMapData: Split Left error!" << std::endl;
#endif
                  }
                }
              }
            }
          }
        }
      }
    }
    if (cur_lane.lane_valid_ && cur_lane.lane_elements_[0].element_valid_ && t_lane_idx_u8 < kMaxLaneNum - 1)
    {
      ::zone::data::em_data::LaneData& right_lane = map_em_data.lanes_[t_lane_idx_u8 + 1];
      if (right_lane.lane_valid_ && right_lane.lane_elements_[0].element_valid_ &&
          !right_lane.lane_elements_[1].element_valid_)
      {
        for (bc::uint8_t t_seg_idx_u8 = 0; t_seg_idx_u8 < kMaxLaneTransitionSegsInOneLaneElement; t_seg_idx_u8++)
        {
          if (cur_lane.lane_elements_[0].reference_line_.lane_transition_dir_segs_[t_seg_idx_u8].lane_trans_dir_ ==
              LaneTransitionDirection::kMergeFromRight)
          {
            bc::uint16_t t_total_valid_num_u16 =
                std::min(right_lane.lane_elements_[0].right_boundary_.total_valid_number_,
                         right_lane.lane_elements_[0].left_boundary_.total_valid_number_);
            if (t_total_valid_num_u16 > kMaxBoundaryPoint || t_total_valid_num_u16 < 1)
            {
#ifdef STD_COUT_ENABLE
              std::cout << "Error in Preprocess::ReorganizeMapData: Lane valid number error!" << std::endl;
#endif
            }
            else
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < (bc::uint8_t)(t_total_valid_num_u16 - 1); t_pt_idx_u8++)
              {
                bc::float32_t delta_y = right_lane.lane_elements_[0].left_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                        right_lane.lane_elements_[0].left_boundary_.pts_[t_pt_idx_u8].y;
                bc::float32_t delta_y_other = cur_lane.lane_elements_[0].right_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                              cur_lane.lane_elements_[0].right_boundary_.pts_[t_pt_idx_u8].y;
                if (fabsf(delta_y) > 1.65f && fabsf(delta_y_other) < 1.5f)
                {
                  right_lane.lane_elements_[0].left_boundary_.total_valid_number_ = t_pt_idx_u8 + 1;
                  right_lane.lane_elements_[0].right_boundary_.total_valid_number_ = t_pt_idx_u8 + 1;
                  if (right_lane.lane_elements_[0].left_boundary_.total_valid_number_ > kMaxBoundaryPoint)
                  {
#ifdef STD_COUT_ENABLE
                    std::cout << "Error in Preprocess::ReorganizeMapData: Merge Right error!" << std::endl;
#endif
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
    if (cur_lane.lane_valid_ && cur_lane.lane_elements_[0].element_valid_ && t_lane_idx_u8 > 0)
    {
      ::zone::data::em_data::LaneData& left_lane = map_em_data.lanes_[t_lane_idx_u8 - 1];
      if (left_lane.lane_valid_ && left_lane.lane_elements_[0].element_valid_ &&
          !left_lane.lane_elements_[1].element_valid_)
      {
        for (bc::uint8_t t_seg_idx_u8 = 0; t_seg_idx_u8 < kMaxLaneTransitionSegsInOneLaneElement; t_seg_idx_u8++)
        {
          if (cur_lane.lane_elements_[0].reference_line_.lane_transition_dir_segs_[t_seg_idx_u8].lane_trans_dir_ ==
              LaneTransitionDirection::kMergeFromLeft)
          {

            bc::uint8_t t_total_valid_num_u16 =
                std::min(left_lane.lane_elements_[0].right_boundary_.total_valid_number_,
                         left_lane.lane_elements_[0].left_boundary_.total_valid_number_);
            if (t_total_valid_num_u16 > kMaxBoundaryPoint || t_total_valid_num_u16 < 1)
            {
#ifdef STD_COUT_ENABLE
              std::cout << "Error in Preprocess::ReorganizeMapData: Lane valid number error!" << std::endl;
#endif
            }
            else
            {
              for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < (bc::uint8_t)(t_total_valid_num_u16 - 1); t_pt_idx_u8++)
              {
                bc::float32_t delta_y = left_lane.lane_elements_[0].right_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                        left_lane.lane_elements_[0].right_boundary_.pts_[t_pt_idx_u8].y;
                bc::float32_t delta_y_other = cur_lane.lane_elements_[0].left_boundary_.pts_[t_pt_idx_u8 + 1].y -
                                              cur_lane.lane_elements_[0].left_boundary_.pts_[t_pt_idx_u8].y;
                if (fabsf(delta_y) > 1.65f && fabsf(delta_y_other) < 1.5f)
                {

                  // right_lane.lane_elements_[0].left_boundary_.reserve_[0] =
                  //     cur_lane.lane_elements_[0].right_boundary_.reserve_[0] = t_pt_idx_u8 + 1;
                  left_lane.lane_elements_[0].left_boundary_.total_valid_number_ = t_pt_idx_u8 + 1;
                  left_lane.lane_elements_[0].right_boundary_.total_valid_number_ = t_pt_idx_u8 + 1;
                  if (left_lane.lane_elements_[0].left_boundary_.total_valid_number_ > kMaxBoundaryPoint)
                  {
#ifdef STD_COUT_ENABLE
                    std::cout << "Error in Preprocess::ReorganizeMapData: Merge Left error!" << std::endl;
#endif
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}

void Preprocess::LdVehLaneTrack(const PerObjCollection& obj_collection_cs, const EmData& em_output_lst1_cs,
                                const bc::float32_t& time_cycle_f, const PreProParam& prep_param_st)
{
  lv_dbg_cs_ = EmPreLVDbg();  ///< init
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkLdveh) == bc::false_v)
  {
    return;
  }
  if (input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
      input_diag_manager_ar_[kInputChkEgoMotion].status_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
    obj_traj_collection_cs_ = ObjectTrajectoryCollection();
    return;
  }
  /** 1. Track relevant objects' trajectories. */
  ObjHistTrajTrack(obj_collection_cs, em_output_lst1_cs.agents_);

  /** 2. Maintain virtual lane derived by the trajectory of LdVeh. */
  LdVehLaneGen(em_output_lst1_cs, prep_param_st, time_cycle_f);
}

void Preprocess::EgoLaneTrack(const EmData& em_output_lst1_cs, const bc::float32_t& time_cycle_f,
                              const StEndIdxPack& last_em_st_end_idx_, const PreProParam& prep_param_st,
                              const EmAdapterHDmapData& map_adapter_cs)
{
  if (GetBitU16(prep_param_st.input_source_consider_u16, kInputChkRef) == bc::false_v)
  {
    return;
  }
  if (input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
      input_diag_manager_ar_[kInputChkEgoMotion].status_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
    ego_track_lane_collection_cs_ = EgoTrackLaneCollection();
    return;
  }
  /** 0. Check em output validity: if invalid, return directly */
  const bc::uint8_t t_host_type_en = em_output_lst1_cs.lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_;
  const bc::bool_t t_lst_left_lane_b = em_output_lst1_cs.lanes_[kLeftLane].lane_valid_;
  const bc::bool_t t_lst_right_lane_b = em_output_lst1_cs.lanes_[kRightLane].lane_valid_;
  const bc::uint8_t t_lst_left_type_en =
      em_output_lst1_cs.lanes_[kLeftLane].lane_elements_[0].left_boundary_.source_type_;
  const bc::uint8_t t_lst_right_type_en =
      em_output_lst1_cs.lanes_[kRightLane].lane_elements_[0].right_boundary_.source_type_;

  if ((ld_veh_lst1_id_u16_ == kInvalidObjectId) &&
      (GetBitU8(t_host_type_en, LaneBoundary::kSrcMaskEgo) || t_host_type_en == 0) &&
      ((t_lst_left_lane_b == bc::false_v && t_lst_right_lane_b == bc::false_v) ||
       ((GetBitU8(t_lst_left_type_en, LaneBoundary::kSrcMaskEgo) || t_lst_left_type_en == 0) &&
        (GetBitU8(t_lst_right_type_en, LaneBoundary::kSrcMaskEgo) || t_lst_right_type_en == 0))))
  {
    ego_track_lane_collection_cs_ = EgoTrackLaneCollection();  // reset
    return;
  }
  /** 1. Tracking ego lane geometry from last EM output, set result to ego_track_lane_collection_cs_ */

  /** 2. Align ego_track_lane_collection_cs_ to x_ref */
  ego_track_lane_collection_cs_ = EgoTrackLaneCollection();
  for (bc::uint8_t t_lane_u8 = 0; t_lane_u8 < kMaxLaneNum; ++t_lane_u8)
  {
    const LaneData& t_lane_cs = em_output_lst1_cs.lanes_[t_lane_u8];
    LaneElementPoint& t_track_lane_cs = ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_u8];
    if (t_lane_cs.lane_valid_)
    {
      t_track_lane_cs.valid_b_ = bc::true_v;

      /** Track front and rear lane */
      for (bc::uint8_t t_element_u8 = 0; t_element_u8 < kMaxElemNumInOneLane; ++t_element_u8)
      {
        bc::uint8_t t_left_valid_pt_num_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_right_valid_pt_num_u8 = kMaxBoundaryPoint;
        if (t_lane_cs.lane_elements_[t_element_u8].element_valid_)
        {
          t_track_lane_cs.lane_element_point_ar_[t_element_u8].valid_b_ = bc::true_v;
          t_track_lane_cs.lane_element_point_ar_[t_element_u8].element_id_ =
              t_lane_cs.lane_elements_[t_element_u8].element_id_;
          if (t_lane_u8 == kHostLane)
          {
            for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 4; t_idx_u8++)
            {
              if (joint_points_collection_cs_[t_idx_u8].joint_b_)
              {
                TrackLstCrossPoint(joint_points_collection_cs_[t_idx_u8]);
              }
            }
          }
          if (t_lane_cs.lane_elements_[t_element_u8].left_boundary_.existence_)
          {
            t_track_lane_cs.lane_element_point_ar_[t_element_u8].left_point_cs_.matching_id_u8_ =
                t_lane_cs.lane_elements_[t_element_u8].left_boundary_.matching_id_;
            t_track_lane_cs.lane_element_point_ar_[t_element_u8].left_point_cs_.per_line_id_s32_ =
                t_lane_cs.lane_elements_[t_element_u8].left_boundary_.per_line_id_;
            /** Track points from em last.*/
            bc::uint8_t t_st_idx_u8 =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_st_idx_;
            bc::uint8_t t_end_idx_u8 =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_end_idx_;
            bc::float32_t t_ref_st_idx = std::min(kFrontStartIdx, t_st_idx_u8);
            bc::TCArray<Point3D, kMaxBoundaryPoint> t_l_points_ar;
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; t_pt_idx_u8++)
            {
              t_l_points_ar[t_pt_idx_u8].x = FLOAT32_MIN;
              t_l_points_ar[t_pt_idx_u8].y = 0;
            }
            for (bc::uint8_t t_idx_u8 = t_ref_st_idx; t_idx_u8 <= t_end_idx_u8; ++t_idx_u8)
            {
              t_l_points_ar[t_idx_u8] = t_lane_cs.lane_elements_[t_element_u8].left_boundary_.pts_[t_idx_u8];
            }
            Point2D end_point, start_point;
            end_point.x =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_end_x_f_;
            end_point.y = FindPointDy(end_point.x, t_lane_cs.lane_elements_[t_element_u8].left_boundary_, t_end_idx_u8);
            start_point.x =
                std::max(last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_st_x_f_,
                         t_lane_cs.lane_elements_[t_element_u8].left_boundary_.pts_[0].x);
            start_point.y =
                FindPointDy(start_point.x, t_lane_cs.lane_elements_[t_element_u8].left_boundary_, t_st_idx_u8);
            TrackBoundaryPoints(t_l_points_ar, end_point, start_point);  ///< 100 points

            bc::float32_t t_last_em_end_x_f = end_point.x;
            bc::float32_t t_last_em_st_x_f = start_point.x;
            if (t_last_em_end_x_f < x_ref_ar_[kFrontStartIdx])
            {
              t_track_lane_cs.lane_element_point_ar_[t_element_u8] = LaneBoundaryPoint();
              break;
            }
            bc::uint8_t t_ref_val_end_idx_u8 = FindXRefIdx(t_last_em_end_x_f);
            bc::uint8_t t_ref_val_st_idx_u8 = FindXRefStartIdx(t_last_em_st_x_f);
            /** Copy points to internal.*/
            /** t_end_idx_u8 - t_st_idx_u8 + 1 : berore transform valid total num(last cycle)
             *  kMaxBoundaryPoint : after alignment need total num
             *  t_ref_val_st_idx_u8: after transform real start idx
             *  t_ref_val_end_idx_u8: after transform real end idx
             *  t_st_idx_u8: before transform start idx(last cycle)
             *  last cycle's idx to choose "t_l_points_ar"'s real valid pts
             *  after transform's idx to compare for final start and end idx
             */
            BoundaryPoint& t_l_track_points_cs = t_track_lane_cs.lane_element_point_ar_[t_element_u8].left_point_cs_;
            t_l_track_points_cs.dx_end_f_ = t_last_em_end_x_f;
            t_l_track_points_cs.dx_start_f_ = t_last_em_st_x_f;
            if (StoreTrackBoundaryPoints(t_l_points_ar, t_l_track_points_cs, t_end_idx_u8 - t_ref_st_idx + 1,
                                         t_ref_st_idx, t_ref_val_st_idx_u8, t_ref_val_end_idx_u8, bc::true_v))
            {
              t_l_track_points_cs.valid_b_ = bc::true_v;
              t_l_track_points_cs.boundary_type_en_ =
                  t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[0].boundary_type_;
              t_l_track_points_cs.boundary_color_en_ =
                  t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[0].color_type_;
              for (bc::uint8_t seg_idx = 0; seg_idx < kMaxBoundarySegment; seg_idx++)
              {
                if (t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].valid_ &&
                    t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].start_s_ <= 0.f &&
                    t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].end_s_ >= 0.f)
                {
                  t_l_track_points_cs.boundary_type_en_ =
                      t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].boundary_type_;
                  t_l_track_points_cs.boundary_color_en_ =
                      t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].color_type_;
                  break;
                }
                // bc::float32_t t_start_dx_f = 0.f;
                // bc::float32_t t_end_dx_f = 0.f;
                // TrackSegsPoint(t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx], t_start_dx_f,
                //                t_end_dx_f);
                // if (t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].valid_ &&
                //     t_start_dx_f <= 0.f && t_end_dx_f >= 0.f)
                // {
                //   t_l_track_points_cs.boundary_type_en_ =
                //       t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].boundary_type_;
                //   t_l_track_points_cs.boundary_color_en_ = 
                //   t_lane_cs.lane_elements_[t_element_u8].left_boundary_.segs_[seg_idx].color_type_;
                //   break;
                // }
              }
              t_l_track_points_cs.source_type_ = t_lane_cs.lane_elements_[t_element_u8].left_boundary_.source_type_;
            }
          }
          if (t_lane_cs.lane_elements_[t_element_u8].right_boundary_.existence_)
          {
            t_track_lane_cs.lane_element_point_ar_[t_element_u8].right_point_cs_.matching_id_u8_ =
                t_lane_cs.lane_elements_[t_element_u8].right_boundary_.matching_id_;
            t_track_lane_cs.lane_element_point_ar_[t_element_u8].right_point_cs_.per_line_id_s32_ =
                t_lane_cs.lane_elements_[t_element_u8].right_boundary_.per_line_id_;
            /** Track points from em last.*/
            bc::TCArray<Point3D, kMaxBoundaryPoint> t_r_points_ar;
            bc::uint8_t t_st_idx_u8 =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_st_idx_;
            bc::uint8_t t_end_idx_u8 =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_end_idx_;
            bc::float32_t t_ref_st_idx = std::min(kFrontStartIdx, t_st_idx_u8);

            for (bc::uint8_t t_idx_u8 = t_ref_st_idx; t_idx_u8 <= t_end_idx_u8; ++t_idx_u8)
            {
              t_r_points_ar[t_idx_u8] = t_lane_cs.lane_elements_[t_element_u8].right_boundary_.pts_[t_idx_u8];
            }
            Point2D end_point, start_point;
            end_point.x =
                last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_end_x_f_;
            end_point.y =
                FindPointDy(end_point.x, t_lane_cs.lane_elements_[t_element_u8].right_boundary_, t_end_idx_u8);
            start_point.x =
                std::max(last_em_st_end_idx_.lane_st_end_idx_[t_lane_u8].ele_st_end_idx_[t_element_u8].boundary_st_x_f_,
                         t_lane_cs.lane_elements_[t_element_u8].right_boundary_.pts_[0].x);
            start_point.y =
                FindPointDy(start_point.x, t_lane_cs.lane_elements_[t_element_u8].right_boundary_, t_st_idx_u8);
            TrackBoundaryPoints(t_r_points_ar, end_point, start_point);  ///< 100 points
            bc::float32_t t_last_em_end_x_f = end_point.x;
            bc::float32_t t_last_em_st_x_f = start_point.x;

            if (t_last_em_end_x_f < x_ref_ar_[kFrontStartIdx])
            {
              t_track_lane_cs.lane_element_point_ar_[t_element_u8] = LaneBoundaryPoint();
              break;
            }
            bc::uint8_t t_ref_val_end_idx_u8 = FindXRefIdx(t_last_em_end_x_f);
            bc::uint8_t t_ref_val_st_idx_u8 = FindXRefStartIdx(t_last_em_st_x_f);
            if (t_ref_val_end_idx_u8 > 69)
            {
              t_ref_val_end_idx_u8 = t_ref_val_end_idx_u8;
            }
            /** Copy points to internal.*/
            BoundaryPoint& t_r_track_points_cs = t_track_lane_cs.lane_element_point_ar_[t_element_u8].right_point_cs_;
            t_r_track_points_cs.dx_end_f_ = t_last_em_end_x_f;
            t_r_track_points_cs.dx_start_f_ = t_last_em_st_x_f;
            if (StoreTrackBoundaryPoints(t_r_points_ar, t_r_track_points_cs, t_end_idx_u8 - t_ref_st_idx + 1,
                                         t_ref_st_idx, t_ref_val_st_idx_u8, t_ref_val_end_idx_u8, bc::true_v))
            {
              t_r_track_points_cs.valid_b_ = bc::true_v;
              t_r_track_points_cs.boundary_type_en_ =
                  t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[0].boundary_type_;
              t_r_track_points_cs.boundary_color_en_ =
                  t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[0].color_type_;
              for (bc::uint8_t seg_idx = 0; seg_idx < kMaxBoundarySegment; seg_idx++)
              {
                if (t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].valid_ &&
                    t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].start_s_ <= 0.f &&
                    t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].end_s_ >= 0.f)
                {
                  t_r_track_points_cs.boundary_type_en_ =
                      t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].boundary_type_;
                  t_r_track_points_cs.boundary_color_en_ =
                      t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].color_type_;
                  break;
                }
                // bc::float32_t t_start_dx_f = 0.f;
                // bc::float32_t t_end_dx_f = 0.f;
                // TrackSegsPoint(t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx], t_start_dx_f,
                //                t_end_dx_f);
                // if (t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].valid_ &&
                //     t_start_dx_f <= 0.f && t_end_dx_f >= 0.f)
                // {
                //   t_r_track_points_cs.boundary_type_en_ =
                //       t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].boundary_type_;
                //   t_r_track_points_cs.boundary_color_en_ =
                //       t_lane_cs.lane_elements_[t_element_u8].right_boundary_.segs_[seg_idx].color_type_;
                //   break;
                // }
              }
              t_r_track_points_cs.source_type_ = t_lane_cs.lane_elements_[t_element_u8].right_boundary_.source_type_;
            }
          }
        }
      }
    }
  }
  DoRefMapMatch(map_adapter_cs);
}

void Preprocess::DoRefMapMatch(const EmAdapterHDmapData& map_adapter_cs)
{
  if (map_lane_change_s8 < 0)
  {
    for (bc::uint8_t t_lane_idx_u8 = 1; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
    {
      if (t_lane_idx_u8 + map_lane_change_s8 < 0)
      {
        ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8] = LaneElementPoint();
      }
      else
      {
        ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8 + map_lane_change_s8] =
            ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8];
      }
    }
    ego_track_lane_collection_cs_.ego_track_point_ar_[kRRLane] = LaneElementPoint();
  }
  else if (map_lane_change_s8 > 0)
  {
    for (bc::int8_t t_lane_idx_s8 = 3; t_lane_idx_s8 >= 0; t_lane_idx_s8--)
    {
      if (t_lane_idx_s8 + map_lane_change_s8 >= kMaxLaneNum)
      {
        ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_s8] = LaneElementPoint();
      }
      else
      {
        ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_s8 + map_lane_change_s8] =
            ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_s8];
      }
    }
    ego_track_lane_collection_cs_.ego_track_point_ar_[kLLLane] = LaneElementPoint();
  }
  else
  {
    /// do nothing
  }
  /// 判断是否要进行ref与map的element ID匹配
  /// 利用左中右三条车道判断上个cycle是否存在地图源，避免切换源的情况也进入elementID的匹配
  /// 仅针对道路拓扑结构变化的情况进行匹配
  /// 同时需要当前cycle的map是有效的（加入map校验）
  bc::bool_t t_lst_element_id_exist_b = bc::false_v;
  for (bc::uint8_t t_lane_idx_u8 = kLeftLane; t_lane_idx_u8 < kRRLane; t_lane_idx_u8++)
  {
    if (ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].valid_b_)
    {
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8]
                .lane_element_point_ar_[t_ele_idx_u8]
                .valid_b_)
        {
          if (ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8]
                  .lane_element_point_ar_[t_ele_idx_u8]
                  .element_id_ != 0)
          {
            t_lst_element_id_exist_b = bc::true_v;
            break;
          }
        }
      }
      if (t_lst_element_id_exist_b == bc::true_v)
      {
        break;
      }
    }
  }
  if (t_lst_element_id_exist_b && ehr_analy_b_)
  {
    bc::TCArray<LaneElementPoint, kMaxLaneNum> t_track_point_ar = ego_track_lane_collection_cs_.ego_track_point_ar_;
    for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
    {
      if (t_track_point_ar[t_lane_idx_u8].valid_b_)
      {
        for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
        {
          if (t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].valid_b_)
          {
            bc::bool_t t_lst_map_match_b = bc::false_v;
            bc::uint16_t lst_element_id =
                t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].element_id_;
            bc::uint16_t map_element_id = 0;
            if (map_adapter_cs.lanes_[t_lane_idx_u8].lane_valid_ &&
                map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_valid_)
            {
              map_element_id = map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_id_;
            }
            ///上个cycle这个车道是有地图数据时，可以与当前map进行匹配
            if (lst_element_id != 0)
            {
              if (lst_element_id == map_element_id)
              {
                ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8] =
                    t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8];
                if (!ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].valid_b_)
                {
                  ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].valid_b_ = bc::true_v;
                }
                t_lst_map_match_b = bc::true_v;
              }
              else
              {
                for (bc::uint8_t t_map_lane_idx_u8 = 0; t_map_lane_idx_u8 < kMaxLaneNum; t_map_lane_idx_u8++)
                {
                  if (map_adapter_cs.lanes_[t_map_lane_idx_u8].lane_valid_)
                  {
                    for (bc::uint8_t t_map_ele_idx_u8 = 0; t_map_ele_idx_u8 < kMaxElemNumInOneLane; t_map_ele_idx_u8++)
                    {
                      if (map_adapter_cs.lanes_[t_map_lane_idx_u8].lane_elements_[t_map_ele_idx_u8].element_valid_)
                      {
                        /// map_element_id 一定不为0
                        map_element_id =
                            map_adapter_cs.lanes_[t_map_lane_idx_u8].lane_elements_[t_map_ele_idx_u8].element_id_;
                        if (lst_element_id == map_element_id)
                        {
                          ego_track_lane_collection_cs_.ego_track_point_ar_[t_map_lane_idx_u8]
                              .lane_element_point_ar_[t_map_ele_idx_u8] =
                              t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8];
                          ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8]
                              .lane_element_point_ar_[t_ele_idx_u8] = LaneBoundaryPoint();
                          if (ego_track_lane_collection_cs_.ego_track_point_ar_[t_map_lane_idx_u8]
                                  .lane_element_point_ar_[t_map_ele_idx_u8]
                                  .valid_b_)
                          {
                            ego_track_lane_collection_cs_.ego_track_point_ar_[t_map_lane_idx_u8].valid_b_ = bc::true_v;
                          }
                          t_lst_map_match_b = bc::true_v;
                          break;
                        }
                        else
                        {
                          continue;
                        }
                      }
                    }
                    if (t_lst_map_match_b)
                    {
                      break;
                    }
                    else
                    {
                      continue;
                    }
                  }
                }
                if (t_lst_map_match_b == bc::false_v)
                {
                  t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].map_unavailable_b_ = bc::true_v;
                  ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8]
                      .lane_element_point_ar_[t_ele_idx_u8] = LaneBoundaryPoint();
                }
              }
            }
            else
            {
              t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].map_unavailable_b_ = bc::true_v;
              ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8] =
                  LaneBoundaryPoint();
            }
          }
        }
        if (!ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].lane_element_point_ar_[0].valid_b_ &&
            !ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8].lane_element_point_ar_[1].valid_b_)
        {
          ego_track_lane_collection_cs_.ego_track_point_ar_[t_lane_idx_u8] = LaneElementPoint();
        }
      }
    }
  }
}

/** Find end idx --> output
 */
bc::uint8_t Preprocess::FindXRefIdx(const bc::float32_t temp_x)
{

  bc::uint8_t t_return_idx_u8 = 0;
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
  {
    if (temp_x > x_ref_ar_[t_pt_idx_u8] && temp_x < x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8;
    }
    else if (t_pt_idx_u8 == 0 && temp_x <= x_ref_ar_[t_pt_idx_u8])
    {
      t_return_idx_u8 = t_pt_idx_u8;
    }
    else if (t_pt_idx_u8 == kMaxBoundaryPoint - 2 && temp_x >= x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8 + 1;
    }
    else if (fabsf(temp_x - x_ref_ar_[t_pt_idx_u8]) <= FLOAT32_EPSILON && temp_x < x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8;
    }
  }
  return t_return_idx_u8;
}

//  Find start idx --> output + 1
bc::uint8_t Preprocess::FindXRefStartIdx(const bc::float32_t temp_x)
{
  bc::uint8_t t_return_idx_u8 = 0;
  for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
  {
    if (temp_x > x_ref_ar_[t_pt_idx_u8] && temp_x < x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8 + 1;
    }
    else if (t_pt_idx_u8 == 0 && temp_x <= x_ref_ar_[t_pt_idx_u8])
    {
      t_return_idx_u8 = t_pt_idx_u8;
    }
    else if (t_pt_idx_u8 == kMaxBoundaryPoint - 2 && temp_x >= x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8 + 1;
    }
    else if (fabsf(temp_x - x_ref_ar_[t_pt_idx_u8]) <= FLOAT32_EPSILON && temp_x < x_ref_ar_[t_pt_idx_u8 + 1])
    {
      t_return_idx_u8 = t_pt_idx_u8;
    }
  }
  return t_return_idx_u8;
}

bc::float32_t Preprocess::FindPointDy(const bc::float32_t dx, const LaneBoundary& lane_boundary,
                                      const bc::uint8_t end_idx_u8)
{
  bc::float32_t temp_y_f = 0.f;
  if (dx > lane_boundary.pts_[end_idx_u8].x)
  {
    for (bc::uint8_t idx = end_idx_u8; idx < kMaxBoundaryPoint - 1; idx++)
    {
      if (idx == kMaxBoundaryPoint - 2)
      {
        if (dx >= lane_boundary.pts_[idx + 1].x)
        {
          temp_y_f =
              LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y, lane_boundary.pts_[idx + 1].x,
                                  lane_boundary.pts_[idx + 1].y, dx, Lin_Interp_Method::kExtrapolate);
          break;
        }
        else
        {
          temp_y_f = LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y,
                                         lane_boundary.pts_[idx + 1].x, lane_boundary.pts_[idx + 1].y, dx);
          break;
        }
      }
      else
      {
        if (dx <= lane_boundary.pts_[idx + 1].x)
        {
          temp_y_f = LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y,
                                         lane_boundary.pts_[idx + 1].x, lane_boundary.pts_[idx + 1].y, dx);
          break;
        }
        else
        {
          continue;
        }
      }
    }
  }
  else
  {
    for (bc::int8_t idx = end_idx_u8 - 1; idx >= 0; idx--)
    {
      if (idx == 0)
      {
        if (dx <= lane_boundary.pts_[idx].x)
        {
          temp_y_f =
              LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y, lane_boundary.pts_[idx + 1].x,
                                  lane_boundary.pts_[idx + 1].y, dx, Lin_Interp_Method::kExtrapolate);
          break;
        }
        else
        {
          temp_y_f = LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y,
                                         lane_boundary.pts_[idx + 1].x, lane_boundary.pts_[idx + 1].y, dx);
          break;
        }
      }
      else
      {
        if (dx >= lane_boundary.pts_[idx].x)
        {
          temp_y_f = LinearInterpolation(lane_boundary.pts_[idx].x, lane_boundary.pts_[idx].y,
                                         lane_boundary.pts_[idx + 1].x, lane_boundary.pts_[idx + 1].y, dx);
          break;
        }
        else
        {
          continue;
        }
      }
    }
  }
  return temp_y_f;
}

void Preprocess::ScenarioRecognition()
{
  /// TODO: 识别Y型分叉并将左右边线处理成左左,右或者左,右右, 方便后续处理
  LaneBoundaryPoint& t_host_lane_cs =
      ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0];
  if ((t_host_lane_cs.left_point_cs_.per_line_id_s32_ != lst_left_line_id_s32_ &&
       t_host_lane_cs.right_point_cs_.per_line_id_s32_ != lst_right_line_id_s32_) ||
      (((lst_per_host_lane_coeff_cs_.left_coeff_cs_.per_line_id_s32_ != lst_perlane_cs_.lines[0].line_id &&
         lst_right_line_id_s32_ == t_host_lane_cs.right_point_cs_.per_line_id_s32_) ||
        (lst_per_host_lane_coeff_cs_.right_coeff_cs_.per_line_id_s32_ != lst_perlane_cs_.lines[1].line_id &&
         lst_left_line_id_s32_ == t_host_lane_cs.left_point_cs_.per_line_id_s32_)) &&
       lst_per_host_lane_coeff_cs_.left_coeff_cs_.is_paralleled_b &&
       lst_per_host_lane_coeff_cs_.right_coeff_cs_.is_paralleled_b))
  {
    if (modified_ref_b)
    {
      if (t_host_lane_cs.left_point_cs_.per_line_id_s32_ == -1 &&
          lst_perlane_cs_.lines[0].lines_3d[0].start_pt.x < 10.f)
      {
        t_host_lane_cs.left_point_cs_ = BoundaryPoint();
        modified_ref_b = bc::false_v;
      }
      if (t_host_lane_cs.right_point_cs_.per_line_id_s32_ == -1 &&
          lst_perlane_cs_.lines[1].lines_3d[0].start_pt.x < 10.f)
      {
        t_host_lane_cs.right_point_cs_ = BoundaryPoint();
        modified_ref_b = bc::false_v;
      }
    }
  }
  // 不需要对地图源场景识别
  if (t_host_lane_cs.valid_b_ && t_host_lane_cs.element_id_ == 0)
  {
    BoundaryPoint& t_left_boundary_cs = t_host_lane_cs.left_point_cs_;
    BoundaryPoint& t_right_boundary_cs = t_host_lane_cs.right_point_cs_;
    // 两边都是由真实车道线追踪得到
    if (t_left_boundary_cs.valid_b_ && t_right_boundary_cs.valid_b_ && t_left_boundary_cs.per_line_id_s32_ != -1 &&
        t_right_boundary_cs.per_line_id_s32_ != -1 &&
        t_left_boundary_cs.per_line_id_s32_ != t_right_boundary_cs.per_line_id_s32_)
    {
      bc::float32_t rear_most_width = t_left_boundary_cs.dy_ar_[t_left_boundary_cs.dy_start_idx_u8_] -
                                      t_right_boundary_cs.dy_ar_[t_right_boundary_cs.dy_start_idx_u8_];
      bc::float32_t ego_width = t_left_boundary_cs.dy_ar_[20] - t_right_boundary_cs.dy_ar_[20];
      bc::float32_t front_most_width = t_left_boundary_cs.dy_ar_[t_left_boundary_cs.dy_end_idx_u8_] -
                                       t_right_boundary_cs.dy_ar_[t_right_boundary_cs.dy_end_idx_u8_];
      /// TODO: 更common的方法识别Y型分叉
      if (front_most_width - ego_width > 6.f && ego_width > 2.f)
      {
        /// TODO: 判断哪个边线变化最小 (直接起点终点拉条线用平均差判断?)
        if (fabsf(t_left_boundary_cs.dy_ar_[t_left_boundary_cs.dy_end_idx_u8_] - t_left_boundary_cs.dy_ar_[20]) >
            fabsf(t_right_boundary_cs.dy_ar_[t_right_boundary_cs.dy_end_idx_u8_] - t_right_boundary_cs.dy_ar_[20]))
        {
          LaneBoundaryPoint& t_left_lane_cs =
              ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0];
          if (t_left_lane_cs.element_id_ == 0)
          {
            t_left_lane_cs.left_point_cs_ = t_left_boundary_cs;
            t_left_lane_cs.valid_b_ = bc::true_v;
            ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].valid_b_ = bc::true_v;
          }
          t_host_lane_cs.left_point_cs_ = BoundaryPoint();
          modified_ref_b = bc::true_v;
        }
        else
        {
          LaneBoundaryPoint& t_right_lane_cs =
              ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0];
          if (t_right_lane_cs.element_id_ == 0)
          {
            t_right_lane_cs.right_point_cs_ = t_right_boundary_cs;
            t_right_lane_cs.valid_b_ = bc::true_v;
            ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].valid_b_ = bc::true_v;
          }
          t_host_lane_cs.right_point_cs_ = BoundaryPoint();
          modified_ref_b = bc::true_v;
        }
      }
    }
    // 将因为分叉生成的虚拟边线用真实的平行车道线替代
    else if (t_left_boundary_cs.valid_b_ && t_right_boundary_cs.valid_b_ &&
             (t_left_boundary_cs.per_line_id_s32_ != -1 || t_right_boundary_cs.per_line_id_s32_ != -1))
    {
      BoundaryPoint& t_left_lane_left_boundary_cs =
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].left_point_cs_;
      BoundaryPoint& t_right_lane_right_boundary_cs =
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].right_point_cs_;
      if (t_left_boundary_cs.per_line_id_s32_ == -1 && (!(lst_perlane_cs_.lines[0].line_source >> 26) & 1) &&
          t_left_lane_left_boundary_cs.valid_b_ && t_left_lane_left_boundary_cs.dy_ar_[20] < 3.f)
      {
        t_host_lane_cs.left_point_cs_ = t_left_lane_left_boundary_cs;
        ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane] =
            ego_track_lane_collection_cs_.ego_track_point_ar_[kLLLane];
      }
      else if (t_right_boundary_cs.per_line_id_s32_ == -1 && (!(lst_perlane_cs_.lines[1].line_source >> 26) & 1) &&
               t_right_lane_right_boundary_cs.valid_b_ && t_right_lane_right_boundary_cs.dy_ar_[20] > -3.f)
      {
        t_host_lane_cs.right_point_cs_ = t_right_lane_right_boundary_cs;
        ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane] =
            ego_track_lane_collection_cs_.ego_track_point_ar_[kRRLane];
      }
    }
  }
}

void Preprocess::PerMapMatching(const EmData& em_output_lst1_cs, const EhrToEmData& ehr2em_cs,
                                const LaneMarkings& per_lane_cs, const bc::float32_t& time_cycle_f,
                                const EgoMotionData& ego_motion_data_st, EmAdapterHDmapData& map_adapter_cs)
{
  /** 0. Set Perception Data */
  per_analy_b_ = bc::false_v;
  if (input_diag_manager_ar_[kInputChkPerLane].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    lst_perlane_cs_ = per_lane_cs;
    per_analy_b_ = bc::true_v;
  }
  else if (input_diag_manager_ar_[kInputChkPerLane].status_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
           input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
  }
  else
  {
    per_analy_b_ = bc::true_v;
  }
  if (per_analy_b_)
  {
    /** 0. Reset internal collection data.*/
    per_lane_collection_cs_ = PerLaneCollection();
    /** 1. Mapping per lane info to per_coeff_ar_ in per_lane_collection_cs_ and map. */
    // PerLaneMapping(lst_perlane_cs_, em_output_lst1_cs.lanes_);
  }
  else
  {
    /** 0. Reset internal collection data.*/
    per_lane_collection_cs_ = PerLaneCollection();
  }

  /** 1. Set Map Data */
  /** 1.1 hdmap adapter */
  debug_map_time_ = (bc::float32_t)input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.GetStatus();
  debug_map_status_ = (bc::float32_t)input_diag_manager_ar_[kInputChkMap].status_man_cs_.GetStatus();
  /** 1. Map Adaption. TODO: need to be detialized */
  ehr_analy_b_ = bc::false_v;
  if (input_diag_manager_ar_[kInputChkMap].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    lst_ehr2em_cs_ = ehr2em_cs;
    if (lst_ehr2em_cs_.geofence_data_.is_in_map_)
    {
      ehr_analy_b_ = bc::true_v;
    }
    else
    {
      /// do nothing
    }
  }
  else if (input_diag_manager_ar_[kInputChkMap].status_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
           input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
    lst_ehr2em_cs_ = EhrToEmData();
  }
  else
  {
    if (lst_ehr2em_cs_.geofence_data_.is_in_map_)
    {
      ehr_analy_b_ = bc::true_v;
    }
    else
    {
      /// do nothing
    }
  }
  if (ehr_analy_b_)
  {
    map_adapter_cs = EmAdapterHDmapData();
    map_lane_collection_cs_ = MapLaneCollection();
    /** 1.4 store */
    ehrdatahandle_.setEhrToEmData(lst_ehr2em_cs_);
    MapDataAdapter(lst_perlane_cs_, time_cycle_f, ego_motion_data_st, map_adapter_cs);
    map_dbg_data_.map_host_ele_[2] = map_adapter_cs.lanes_[2].lane_elements_[0];

    map_host_lane_element_id_ = map_adapter_cs.lanes_[kHostLane].lane_elements_[0].element_id_;
    // MapScenarioDistinction(map_adapter_cs);
    SplitMapLaneData(map_adapter_cs);
    raw_map_ele_id_0 = map_adapter_cs.lanes_[0].lane_elements_[0].element_id_;
    raw_map_ele_id_1 = map_adapter_cs.lanes_[1].lane_elements_[0].element_id_;
    raw_map_ele_id_2 = map_adapter_cs.lanes_[2].lane_elements_[0].element_id_;
    raw_map_ele_id_3 = map_adapter_cs.lanes_[3].lane_elements_[0].element_id_;
    raw_map_ele_id_4 = map_adapter_cs.lanes_[4].lane_elements_[0].element_id_;
    map_adapter_cs.global_lane_num_ = ehr2em_cs.lane_data_num_;
    map_adapter_cs.host_lane_idx_ = ehr2em_cs.host_lane_idx_;
    // CheckJumpPoint();
    // std::cout << "MapDataAdapter finished" << std::endl;
    /** 1.2 Fix boundary bends in the split */
    // FixElementBoundaryBend(map_adapter_cs);
    /** 1.3 matching with em last output */
    if ((!GetBitU8(em_output_lst1_cs.lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_,
                   LaneBoundary::kSrcMaskEgo) &&
         !GetBitU8(em_output_lst1_cs.lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_,
                   LaneBoundary::kSrcMaskEgo)) ||
        ((em_output_lst1_cs.lanes_[kLeftLane].lane_valid_ == bc::true_v &&
          !GetBitU8(em_output_lst1_cs.lanes_[kLeftLane].lane_elements_[0].left_boundary_.source_type_,
                    LaneBoundary::kSrcMaskEgo)) ||
         (em_output_lst1_cs.lanes_[kRightLane].lane_valid_ == bc::true_v &&
          !GetBitU8(em_output_lst1_cs.lanes_[kRightLane].lane_elements_[0].right_boundary_.source_type_,
                    LaneBoundary::kSrcMaskEgo))))
    {
      MapMatchingWithLastEm(em_output_lst1_cs.lanes_, map_adapter_cs);
    }
    map_dbg_data_.map_host_ele_[3] = map_adapter_cs.lanes_[2].lane_elements_[0];
    if (map_adapter_cs.lanes_[2].lane_valid_ == bc::false_v)
    {
      map_adapter_cs = EmAdapterHDmapData();
      map_host_lane_element_id_ = 0;
      map_lane_change_s8 = 0;
      lane_match_s8 = 0;
      ehr_analy_b_ = bc::false_v;
    }
  }
  else
  {
    /* Reset stored map data. */
    map_adapter_cs = EmAdapterHDmapData();
    map_lane_collection_cs_ = MapLaneCollection();
    map_host_lane_element_id_ = 0;
    map_lane_change_s8 = 0;
    lane_match_s8 = 0;
    // Tracking tollbooth start&end s
    if (tollbooth_scenario_cs_.valid_)
    {
      // TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();
      // bc::TCArray<bc::float32_t, kTransMatrixNum> t_trans_matrix_ar =
      //     {cosf(t_transform_st.rotation_f_), sinf(t_transform_st.rotation_f_),
      //      -sinf(t_transform_st.rotation_f_), cosf(t_transform_st.rotation_f_)};
      // /// Transform left/right point
      // // Left point
      // bc::float32_t t_dx_f = left_boundary_ptn_.x;
      // bc::float32_t t_dy_f = left_boundary_ptn_.y;
      // t_dx_f -= t_transform_st.translation_dx_f_;
      // t_dy_f -= t_transform_st.translation_dy_f_;
      // t_dx_f = t_trans_matrix_ar[0] * t_dx_f + t_trans_matrix_ar[1] * t_dy_f;
      // t_dy_f = t_trans_matrix_ar[2] * t_dx_f + t_trans_matrix_ar[3] * t_dy_f;
      // left_boundary_ptn_.x = t_dx_f;
      // left_boundary_ptn_.y = t_dy_f;
      // // Right point
      // t_dx_f = right_boundary_ptn_.x;
      // t_dy_f = right_boundary_ptn_.y;
      // t_dx_f -= t_transform_st.translation_dx_f_;
      // t_dy_f -= t_transform_st.translation_dy_f_;
      // t_dx_f = t_trans_matrix_ar[0] * t_dx_f + t_trans_matrix_ar[1] * t_dy_f;
      // t_dy_f = t_trans_matrix_ar[2] * t_dx_f + t_trans_matrix_ar[3] * t_dy_f;
      // right_boundary_ptn_.x = t_dx_f;
      // right_boundary_ptn_.y = t_dy_f;
      tollbooth_scenario_cs_.start_s_ -= time_cycle_f * ego_motion_data_st.twist.linear.x;
      tollbooth_scenario_cs_.end_s_ -= time_cycle_f * ego_motion_data_st.twist.linear.x;
      if (tollbooth_scenario_cs_.end_s_ < 0.f)
      {
        tollbooth_scenario_cs_ = zone::data::em_data::ScenarioClassification();
      }
    }
    // 对异常状态下有效地图数据的处理
    else if (!((ehr2em_cs.return_code_[0] != 0 && ehr2em_cs.return_code_[0] != 0x80000000) ||
               ehr2em_cs.return_code_[1] >= (1 << 20) || ehr2em_cs.return_code_[2] != 0 ||
               (ehr2em_cs.return_code_[3] != 0 && ehr2em_cs.return_code_[3] != 0x10 &&
                ehr2em_cs.return_code_[3] != 0x20 && ehr2em_cs.return_code_[3] != 0x40)))
    {
      if (ehr2em_cs.guide_point_num_ > 0)
      {
        for (uint32_t i = 0; i < ehr2em_cs.guide_point_num_; i++)
        {
          if (ehr2em_cs.guide_point_[i].guide_point_type_ ==
              zone::common::EEhrToEmGuidePointType::EHRTOEM_GUIDE_POINT_TYPE_TOLL_AREA)
          {
            tollbooth_scenario_cs_ = zone::data::em_data::ScenarioClassification();
            bc::float32_t temp_start = ehr2em_cs.guide_point_[i].start_offset_ / 100.f;
            bc::float32_t temp_end = ehr2em_cs.guide_point_[i].end_offset_ / 100.f;
            if (temp_start < 200.f && temp_end > 0.f)
            {
              tollbooth_scenario_cs_.scenario_type_ = zone::data::em_data::ScenarioType::kTollBooth;
              tollbooth_scenario_cs_.start_s_ = temp_start;
              tollbooth_scenario_cs_.end_s_ = temp_end + 20.f;  //增加冗余距离
              tollbooth_scenario_cs_.valid_ = bc::true_v;
            }
            break;
          }
        }
      }
    }
  }

  /// Fix loc rotation angle error
  // TransformMapdata(time_cycle_f, map_adapter_cs);
  // std::cout << "TransformMapdata finished" << std::endl;

  /** 1. Matching per lane info to the EM output lane structure of last cycle. */

  /** 2. Matching map lane info to the EM output lane structure of last cycle */
}

void Preprocess::BoundaryRepack(const EhrToEmData& ehr2em_cs, const EmAdapterHDmapData& map_adapter_cs,
                                const float32_t joint_similarity_threshold, BoundaryPack& boundary_pack_cs)
{
  /// Intersection
  bc::bool_t t_intersection_b = bc::false_v;
  bc::bool_t t_ego_intersection_b = bc::false_v;
  bc::bool_t t_guide_intersection_b = bc::false_v;
  for (bc::uint8_t intersection_idx = 0;
       intersection_idx < kMaxLanePropertySegsInOneLaneElement && !t_ego_intersection_b; intersection_idx++)
  {
    t_ego_intersection_b = map_adapter_cs.lanes_[kHostLane]
                               .lane_elements_[0]
                               .reference_line_.is_in_intersection_segs_[intersection_idx]
                               .valid_ &&
                           map_adapter_cs.lanes_[kHostLane]
                                   .lane_elements_[0]
                                   .reference_line_.is_in_intersection_segs_[intersection_idx]
                                   .start_s_ < 100.f &&
                           map_adapter_cs.lanes_[kHostLane]
                                   .lane_elements_[0]
                                   .reference_line_.is_in_intersection_segs_[intersection_idx]
                                   .end_s_ > -100.f;
  }
  for (bc::uint8_t map_guide_idx = 0; map_guide_idx < kMaxMapGuidePtNum && !t_guide_intersection_b; map_guide_idx++)
  {
    t_guide_intersection_b = (map_adapter_cs.map_guide_pts_[map_guide_idx].valid_ &&
                              map_adapter_cs.map_guide_pts_[map_guide_idx].start_s_ <= 200) &&
                             map_adapter_cs.map_guide_pts_[map_guide_idx].type_ == MapGuidePointType::kIntersection;
  }
  if (t_ego_intersection_b || t_guide_intersection_b || map_adapter_cs.map_geofence_.is_on_intersection_)
  {
    t_intersection_b = bc::true_v;
  }
  /** 0. Fill in all input sources to boundary_pack_cs.
   * NOTE: element[0] represents main branch, so per_lane and ldveh_lane should be filled in element[0].
   * NOTE: In boundary_pack, 4 sources are perception, map, ldveh and reference. Here reference has different meanings
   * for different lanes. As to ego lane, reference represents the tracking result of ego lane. As to other neighbor
   * lane, reference represents the fused reference boundary. Take Left Lane as an example, its left boundary has no
   * reference, and its right boundary has a reference source: the left fused boundary of ego lane. This will be
   * handled
   * in detail in WeightDistribution and HLMF. In this function, we just leave reference as empty for neighbor lane,
   * and
   * put ego lane tracking result into the reference source of ego lane. */
  boundary_pack_cs = BoundaryPack();
  for (bc::uint8_t t_lane_u8 = 0; t_lane_u8 < kMaxLaneNum; ++t_lane_u8)
  {
    for (bc::uint8_t t_element_u8 = 0; t_element_u8 < kMaxElemNumInOneLane; ++t_element_u8)
    {
      /** Repack left boundary.*/
      IndividualBoundaryRepack(boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].left_pack_cs_,
                               t_lane_u8, t_element_u8, kLeftBoundary, t_intersection_b);
      /** Repack right boundary.*/
      IndividualBoundaryRepack(boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_,
                               t_lane_u8, t_element_u8, kRightBoundary, t_intersection_b);
      /** Check element validity.*/
      if (boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].left_pack_cs_.valid_b_ &&
          boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.valid_b_)
      {
        boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].valid_b_ = bc::true_v;
        /** Once one boundary is valid, the validity of this lane will be set true.*/
        boundary_pack_cs.lane_pack_ar_[t_lane_u8].valid_b_ = bc::true_v;
      }
      if (map_adapter_cs.lanes_[t_lane_u8].lane_valid_ &&
          map_adapter_cs.lanes_[t_lane_u8].lane_elements_[t_element_u8].element_valid_ &&
          map_adapter_cs.lanes_[t_lane_u8].lane_elements_[t_element_u8].merge_flag_)
      {
        boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].left_pack_cs_.ref_cs_ =
            BoundaryPoint();
        boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.ref_cs_ =
            BoundaryPoint();
      }
    }
  }
  if (boundary_pack_cs.lane_pack_ar_[kHostLane].element_pack_ar_[0].left_pack_cs_.map_cs_.valid_b_ ||
      boundary_pack_cs.lane_pack_ar_[kHostLane].element_pack_ar_[0].right_pack_cs_.map_cs_.valid_b_)
  {
    for (bc::uint8_t t_lane_u8 = 0; t_lane_u8 < kMaxLaneNum; ++t_lane_u8)
    {
      for (bc::uint8_t t_element_u8 = 0; t_element_u8 < kMaxElemNumInOneLane; ++t_element_u8)
      {
        if (boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].left_pack_cs_.map_cs_.valid_b_ ==
                bc::false_v &&
            boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.map_cs_.valid_b_ ==
                bc::false_v &&
            boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].left_pack_cs_.per_cs_.valid_b_ ==
                bc::false_v &&
            boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.per_cs_.valid_b_ ==
                bc::false_v &&
            (boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.ref_cs_.valid_b_ ==
                 bc::true_v ||
             boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8].right_pack_cs_.ref_cs_.valid_b_ ==
                 bc::true_v))
        {
          boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[t_element_u8] = ElementPack();
        }
        if (boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[0].valid_b_ == bc::false_v &&
            boundary_pack_cs.lane_pack_ar_[t_lane_u8].element_pack_ar_[1].valid_b_ == bc::false_v)
        {
          boundary_pack_cs.lane_pack_ar_[t_lane_u8] = LanePack();
        }
      }
    }
  }

  BoundaryJoint(joint_similarity_threshold, boundary_pack_cs);
}

void Preprocess::SemanticRepack(const StaticObjectData& per_static_obj_cs, const EmAdapterHDmapData& map_adapter_cs,
                                const NavigationPlusData& navi_plus_cs, SemanticPack& semantic_pack_cs)
{
  PerSpdLmtSemanticRepack(per_static_obj_cs, semantic_pack_cs);
  MapSemanticRepack(map_adapter_cs, semantic_pack_cs);
  SDSemanticRepack(navi_plus_cs, semantic_pack_cs);
}

void Preprocess::PerSpdLmtSemanticRepack(const StaticObjectData& per_static_obj_cs, SemanticPack& semantic_pack_cs)
{
  /** Repack speed limit semantic information. */
  /** Horizon static object information will be handled here. */
  /**set default value */
  semantic_pack_cs.traffic_spdlmt_sign_num_u8_ = UINT8_MIN;
  semantic_pack_cs.traffic_spdlmt_rev_sign_num_u8_ = UINT8_MIN;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < HORIZON_OBJECT_MAX_NUM; ++t_idx_u8)
  {
    semantic_pack_cs.traffic_spdlmt_sign_ar_[t_idx_u8] = StaticObject();
    semantic_pack_cs.traffic_spdlmt_rev_sign_ar_[t_idx_u8] = StaticObject();
  }
  if (input_diag_manager_ar_[kInputChkPerStaticObj].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < HORIZON_OBJECT_MAX_NUM; ++t_idx_u8)
    {
      if (per_static_obj_cs.objects_[t_idx_u8].type_ == 1)
      {
        if ((per_static_obj_cs.objects_[t_idx_u8].sub_type_ == 264 ||
             per_static_obj_cs.objects_[t_idx_u8].sub_type_ == 265) && 
             per_static_obj_cs.objects_[t_idx_u8].attr_.value_ >= 5.f && 
             per_static_obj_cs.objects_[t_idx_u8].attr_.value_ <= 120.f)
        {
          semantic_pack_cs.traffic_spdlmt_sign_ar_[semantic_pack_cs.traffic_spdlmt_sign_num_u8_] =
              per_static_obj_cs.objects_[t_idx_u8];
          semantic_pack_cs.traffic_spdlmt_sign_idx_ar_[semantic_pack_cs.traffic_spdlmt_sign_num_u8_] = t_idx_u8;
          semantic_pack_cs.traffic_spdlmt_sign_num_u8_++;
        }
        else if (per_static_obj_cs.objects_[t_idx_u8].sub_type_ == 300)
        {
          semantic_pack_cs.traffic_spdlmt_rev_sign_ar_[semantic_pack_cs.traffic_spdlmt_rev_sign_num_u8_] =
              per_static_obj_cs.objects_[t_idx_u8];
          semantic_pack_cs.traffic_spdlmt_rev_sign_idx_ar_[semantic_pack_cs.traffic_spdlmt_rev_sign_num_u8_] = t_idx_u8;
          semantic_pack_cs.traffic_spdlmt_rev_sign_num_u8_++;
        }
        else
        {
          /** Do nothing*/
        }
      }
      else
      {
        /** Do nothing*/
      }
    }
  }
  else
  {
    /** Do nothing*/
  }
}

void Preprocess::MapSemanticRepack(const EmAdapterHDmapData& map_adapter_cs, SemanticPack& semantic_pack_cs)
{
  semantic_pack_cs.map_geofence_ = map_adapter_cs.map_geofence_;
  semantic_pack_cs.map_guide_pts_ = map_adapter_cs.map_guide_pts_;
  semantic_pack_cs.map_car_position_ = map_adapter_cs.map_car_position_;
  semantic_pack_cs.relat_dest_lane_ = map_adapter_cs.relat_dest_lane_;
  semantic_pack_cs.global_lane_num_ = map_adapter_cs.global_lane_num_;
  semantic_pack_cs.host_lane_idx_ = map_adapter_cs.host_lane_idx_;
  semantic_pack_cs.main_lane_num_ = map_adapter_cs.main_lane_num_;
  semantic_pack_cs.timestamp_ = map_adapter_cs.timestamp_;
  semantic_pack_cs.tja_target_id_ = ld_veh_lst1_id_u16_;
  semantic_pack_cs.c0 = map_adapter_cs.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c0_position;
  semantic_pack_cs.c1 = map_adapter_cs.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c1_heading_angle;
  semantic_pack_cs.c2 = map_adapter_cs.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c2_curvature;
  semantic_pack_cs.c3 =
      map_adapter_cs.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c3_curvature_derivative;
  semantic_pack_cs.nearst_split_scenario_ = map_adapter_cs.nearst_split_scenario_;
  semantic_pack_cs.map_lane_marking_ar_ = map_adapter_cs.lane_marking_ar_;
  semantic_pack_cs.map_cross_stop_line_ar_ = map_adapter_cs.cross_stop_line_ar_;
  semantic_pack_cs.map_traffic_light_ar_ = map_adapter_cs.traffic_light_ar_;
  semantic_pack_cs.map_intersection_data_ = map_adapter_cs.intersection_data_;
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (map_adapter_cs.lanes_[t_lane_idx_u8].lane_valid_)
    {
      semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].valid_b_ = bc::true_v;
      semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].main_lane_idx_u8_ =
          map_adapter_cs.lanes_[t_lane_idx_u8].main_lane_idx_;
      semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].global_lane_idx_u32_ =
          map_adapter_cs.lanes_[t_lane_idx_u8].global_lane_idx_;
      for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
      {
        if (map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8].element_valid_)
        {
          ElementSemanticPack& elem_semantic =
              semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].lane_semantic_ar_[t_elem_idx_u8];
          ReferenceLine hdmap_ref_line =
              map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8].reference_line_;
          LaneElement map_element = map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8];
          elem_semantic.valid_b_ = bc::true_v;
          elem_semantic.is_on_route_segs_ = hdmap_ref_line.is_on_route_segs_;
          elem_semantic.is_in_intersection_segs_ = hdmap_ref_line.is_in_intersection_segs_;
          elem_semantic.speed_limit_segs_ =
              hdmap_ref_line.speed_limit_source_segs_[kHDMapSpdLmtSegIdx].speed_limit_segs_;
          elem_semantic.lane_type_segs_ = hdmap_ref_line.lane_type_segs_;
          elem_semantic.slope_segs_ = hdmap_ref_line.slope_segs_;
          elem_semantic.lane_transition_dir_segs_ = hdmap_ref_line.lane_transition_dir_segs_;
          elem_semantic.super_elevation_segs_ = hdmap_ref_line.super_elevation_segs_;
          // elem_semantic.curvature_segs_ = hdmap_ref_line.curvature_segs_;
          // elem_semantic.heading_segs_ = hdmap_ref_line.heading_segs_;
          // elem_semantic.width_segs_ = hdmap_ref_line.width_segs_;
          elem_semantic.arrow_segs_ = hdmap_ref_line.arrow_segs_;
          elem_semantic.special_segs_ = hdmap_ref_line.special_segs_;
          elem_semantic.route_left_dis_ = map_element.route_left_dis_;
          elem_semantic.is_dest_lane_ele_ = map_element.is_dest_lane_ele_;
          elem_semantic.lane_type_ = map_element.lane_type_;
          elem_semantic.element_id_ = map_element.element_id_;
          elem_semantic.recommend_level_ = map_element.recommend_level_;
          elem_semantic.relative_idx_ = map_element.relative_idx_;
          elem_semantic.merge_flag_ = map_element.merge_flag_;
          elem_semantic.direction_to_dest_ = map_element.direction_to_dest_;
          // elem_semantic.agents_projected_traj_ = hdmap_ref_line.agents_projected_traj_;}
          elem_semantic.left_segs_ = map_element.left_boundary_.segs_;
          elem_semantic.right_segs_ = map_element.right_boundary_.segs_;
        }
        else
        {
          semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8].lane_semantic_ar_[t_elem_idx_u8] = ElementSemanticPack();
        }
      }
    }
    else
    {
      semantic_pack_cs.map_semantic_ar_[t_lane_idx_u8] = LaneSemanticPack();
    }
  }
}

void Preprocess::SDSemanticRepack(const NavigationPlusData& navi_plus_cs, SemanticPack& semantic_pack_cs)
{
  /**set default value */
  semantic_pack_cs.max_normal_spdlmt_valid_b_ = bc::false_v;
  semantic_pack_cs.min_normal_spdlmt_valid_b_ = bc::false_v;
  semantic_pack_cs.max_dx_normal_speed_limit_u16_ = UINT16_MAX;
  semantic_pack_cs.min_dx_normal_speed_limit_u16_ = UINT16_MAX;
  semantic_pack_cs.max_vel_normal_speed_limit_u8_ = UINT8_MAX;
  semantic_pack_cs.min_vel_normal_speed_limit_u8_ = UINT8_MIN;
  semantic_pack_cs.ele_eye_spdlmt_valid_b_ = bc::false_v;
  semantic_pack_cs.vel_ele_eye_speed_limit_u8_ = UINT8_MAX;
  semantic_pack_cs.dx_ele_eye_speed_limit_u32_ = UINT32_MAX;
  semantic_pack_cs.intervel_spdlmt_valid_b_ = bc::false_v;
  semantic_pack_cs.dx_interval_starting_point_u16_ = UINT16_MAX;
  semantic_pack_cs.dx_interval_ending_point_u16_ = UINT16_MAX;
  semantic_pack_cs.vel_interval_speed_limit_u8_ = UINT8_MAX;
  /** Navi SpdLimit information will be handled here. */
  if (input_diag_manager_ar_[kInputChkNaviPlus].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    /**normal speed limit*/
    if (navi_plus_cs.speed_limit_board_.valid_ && navi_plus_cs.speed_limit_board_.limit_speed_ >= 5.f && 
        navi_plus_cs.speed_limit_board_.limit_speed_ <= 120.f)
    {
      semantic_pack_cs.max_normal_spdlmt_valid_b_ = bc::true_v;
      semantic_pack_cs.max_vel_normal_speed_limit_u8_ = navi_plus_cs.speed_limit_board_.limit_speed_;
      semantic_pack_cs.max_dx_normal_speed_limit_u16_ = navi_plus_cs.speed_limit_board_.distance_;
    }
    else
    {
      /** Do nothing*/
    }
    /**electronic eye speed limit*/
    if (navi_plus_cs.spd_lmt_ele_eye_.valid_ && navi_plus_cs.spd_lmt_ele_eye_.speed_value_ >= 5.f && 
        navi_plus_cs.spd_lmt_ele_eye_.speed_value_ <= 120.f)
    {
      semantic_pack_cs.ele_eye_spdlmt_valid_b_ = bc::true_v;
      semantic_pack_cs.vel_ele_eye_speed_limit_u8_ = navi_plus_cs.spd_lmt_ele_eye_.speed_value_;
      semantic_pack_cs.dx_ele_eye_speed_limit_u32_ = navi_plus_cs.spd_lmt_ele_eye_.distance_;
    }
    else
    {
      /** Do nothing*/
    }
    /**section speed limit*/
    if (navi_plus_cs.interval_camera_.valid_ && navi_plus_cs.interval_camera_.speed_value_ >= 5.f && 
        navi_plus_cs.interval_camera_.speed_value_ <= 120.f)
    {
      semantic_pack_cs.intervel_spdlmt_valid_b_ = bc::true_v;
      semantic_pack_cs.dx_interval_starting_point_u16_ = navi_plus_cs.interval_camera_.start_distance_;
      semantic_pack_cs.dx_interval_ending_point_u16_ = navi_plus_cs.interval_camera_.end_distance_;
      semantic_pack_cs.vel_interval_speed_limit_u8_ = navi_plus_cs.interval_camera_.speed_value_;
    }
    else
    {
      /** Do nothing*/
    }
  }
  else
  {
    /** Do nothing*/
  }
}

void Preprocess::StaticObjInfoTransmission(const StaticObjectData& per_static_obj_cs)
{
  if (input_diag_manager_ar_[kInputChkPerStaticObj].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    (*em_collection_ptr_).static_object_.timestamp_s64_ = per_static_obj_cs.exposure_time_stamp_;
    for (bc::uint8_t t_static_obj_index_u8 = 0; t_static_obj_index_u8 < kMaxStaticObjNum; ++t_static_obj_index_u8)
    {
      EmStaticObject& opt = (*em_collection_ptr_).static_object_.objects_[t_static_obj_index_u8];
      const StaticObject& ipt = per_static_obj_cs.objects_[t_static_obj_index_u8];
      opt.id_ = ipt.id_;
      opt.type_ = ipt.type_;
      opt.sub_type_ = ipt.sub_type_;
      opt.conf_ = ipt.conf_;
      opt.left_time_ = ipt.life_time_;
      opt.age_ = ipt.age_;
      opt.attr_.value_ = ipt.attr_.value_;
      opt.attr_.struct_type_ = ipt.attr_.struct_type_;
      opt.attr_.pole_lift_angle_ = ipt.attr_.pole_lift_angle_;
      opt.child_ids_ = ipt.child_ids_;
      opt.position_.x = ipt.position_.x_;
      opt.position_.y = ipt.position_.y_;
      opt.position_.z = ipt.position_.z_;
      opt.child_types_ = ipt.child_types_;
      opt.in_cur_lane_ = ipt.in_cur_lane_;
      opt.drive_start_up_ = ipt.drive_start_up_;
      opt.generation_time_ = ipt.generation_time_;
      for (bc::uint8_t t_border_pts_index_u8 = 0; t_border_pts_index_u8 < 4; ++t_border_pts_index_u8)
      {
        opt.border_.points_[t_border_pts_index_u8].x_ = ipt.border_.points_[t_border_pts_index_u8].x_;
        opt.border_.points_[t_border_pts_index_u8].y_ = ipt.border_.points_[t_border_pts_index_u8].y_;
        opt.border_.points_[t_border_pts_index_u8].z_ = ipt.border_.points_[t_border_pts_index_u8].z_;
        opt.border_.points_[t_border_pts_index_u8].cov_ = ipt.border_.points_[t_border_pts_index_u8].cov_;
      }
      opt.border_.edgeline_width_ = ipt.border_.edgeline_width_;
      opt.border_.normal_.x_ = ipt.border_.normal_.x_;
      opt.border_.normal_.y_ = ipt.border_.normal_.y_;
      opt.border_.normal_.z_ = ipt.border_.normal_.z_;
      opt.border_.normal_.cov_ = ipt.border_.normal_.cov_;
      opt.border_.orientation_.x_ = ipt.border_.orientation_.x_;
      opt.border_.orientation_.y_ = ipt.border_.orientation_.y_;
      opt.border_.orientation_.z_ = ipt.border_.orientation_.z_;
      opt.border_.orientation_.cov_ = ipt.border_.orientation_.cov_;
    }
  }
  else
  {
    (*em_collection_ptr_).static_object_ = EmStaticObjectData();
  }
}

void Preprocess::ValidityCheckPerObj(const ObjTracks& per_obj_cs, const bc::float32_t time_cycle_f,
                                     const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. if time stamp is stucked, set DiagStatus to kErrorWrn. */
  input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. if status is invalid, set DiagStatus to kErrorWrn.*/
}

void Preprocess::ValidityCheckPerLane(const LaneMarkings& per_lane_cs, const bc::float32_t time_cycle_f,
                                      const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckPerStaticObj(const StaticObjectData& per_static_obj_cs, const bc::float32_t time_cycle_f,
                                           const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckNavi(const NavigationData& navi_cs, const bc::float32_t time_cycle_f,
                                   const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkNavi].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckNaviPlus(const NavigationPlusData& navi_plus_cs, const bc::float32_t time_cycle_f,
                                       const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckMap(const EhrToEmData& ehr2em_cs, const bc::float32_t time_cycle_f,
                                  const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
  bc::bool_t t_hdmap_valid_b;
  if ((ehr2em_cs.return_code_[0] != 0 && ehr2em_cs.return_code_[0] != 0x80000000) ||
      ehr2em_cs.return_code_[1] >= (1 << 20) || ehr2em_cs.return_code_[2] != 0 ||
      (ehr2em_cs.return_code_[3] != 0 && ehr2em_cs.return_code_[3] != 0x10))
  {
    t_hdmap_valid_b = bc::false_v;
    // input_diag_manager_ar_[kInputChkMap].status_man_cs_.GetStatus() = DiagStatus::kErrCfm;
  }
  else
  {
    t_hdmap_valid_b = bc::true_v;
  }
  input_diag_manager_ar_[kInputChkMap].status_man_cs_.DiagStsMan(time_cycle_f, t_hdmap_valid_b);
}

void Preprocess::ValidityCheckEgoMotion(const EgoMotionData& ego_motion_cs, const bc::float32_t time_cycle_f,
                                        const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckPerFreeSpace(const FreespaceList& per_freespace_cs, const bc::float32_t& time_cycle_f,
                                           const bc::bool_t& input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  input_diag_manager_ar_[kInputChkPerFreeSp].timeout_man_cs_.DiagStsMan(time_cycle_f, input_update_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
}

void Preprocess::ValidityCheckWorldInfo(const WorkCondition& world_info_cs, const bc::float32_t time_cycle_f,
                                        const bc::bool_t input_update_b)
{
  /** 1. Check time stamp, valid if time stamp increases.  This check should be triggered by a
   * parameter. */
  bc::bool_t t_world_info_timeout_b;
  if (world_info_cs.common_data_header.seq <= lst_world_info_cs_.common_data_header.seq)
  {
    t_world_info_timeout_b = bc::false_v;
  }
  else
  {
    t_world_info_timeout_b = bc::true_v;
  }
  // input_diag_manager_ar_[kInputChkWorldInfo].timeout_man_cs_.DiagStsMan(1.f, t_world_info_timeout_b);
  /** 2. Check status, valid if status is ok. This check should be triggered by a
   * parameter. */
  bc::bool_t t_world_info_valid_b;
  if (!world_info_cs.weather_.available_ && !world_info_cs.light_.available_)
  {
    t_world_info_valid_b = bc::false_v;
  }
  else
  {
    t_world_info_valid_b = bc::true_v;
  }
  input_diag_manager_ar_[kInputChkWorldInfo].status_man_cs_.DiagStsMan(time_cycle_f, t_world_info_valid_b);
}

void Preprocess::WorldInfoProcess(const WorkCondition& world_info_cs, const EmData& em_output_lst1_cs)
{
  world_info_analy_b_ = bc::false_v;
  if (input_diag_manager_ar_[kInputChkWorldInfo].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkWorldInfo].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    lst_world_info_cs_ = world_info_cs;
    world_info_analy_b_ = bc::true_v;
  }
  else if (input_diag_manager_ar_[kInputChkWorldInfo].status_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
           input_diag_manager_ar_[kInputChkWorldInfo].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
  }
  else
  {
    world_info_analy_b_ = bc::true_v;
  }
  if (world_info_analy_b_)
  {
    if (lst_world_info_cs_.weather_.available_)
    {
      switch (lst_world_info_cs_.weather_.property_)
      {
        case 0:
          em_collection_ptr_->world_condition_.weather_ = Weather::kSunny;
          break;

        case 1:
          em_collection_ptr_->world_condition_.weather_ = Weather::kCloudy;
          break;

        case 2:
          em_collection_ptr_->world_condition_.weather_ = Weather::kRainy;
          break;

        case 3:
          em_collection_ptr_->world_condition_.weather_ = Weather::kSnowy;
          break;

        case 4:
          em_collection_ptr_->world_condition_.weather_ = Weather::kHeavyRain;
          break;

        case 5:
          em_collection_ptr_->world_condition_.weather_ = Weather::kOther;
          break;

        case 6:
          em_collection_ptr_->world_condition_.weather_ = Weather::kSmallRain;
          break;

        case 7:
          em_collection_ptr_->world_condition_.weather_ = Weather::kMediumRain;
          break;

        case 8:
          em_collection_ptr_->world_condition_.weather_ = Weather::kSmallSnow;
          break;

        case 9:
          em_collection_ptr_->world_condition_.weather_ = Weather::kMediumSnow;
          break;

        case 10:
          em_collection_ptr_->world_condition_.weather_ = Weather::kHeavySnow;
          break;

        case 11:
          em_collection_ptr_->world_condition_.weather_ = Weather::kSmallFog;
          break;

        case 12:
          em_collection_ptr_->world_condition_.weather_ = Weather::kMediumFog;
          break;

        case 13:
          em_collection_ptr_->world_condition_.weather_ = Weather::kHeavyFog;
          break;

        default:
          em_collection_ptr_->world_condition_.weather_ = Weather::kUnknown;
          break;
      }
    }
    else
    {
      em_collection_ptr_->world_condition_.weather_ = Weather::kUnknown;
    }
    if (lst_world_info_cs_.light_.available_)
    {
      switch (lst_world_info_cs_.light_.property_)
      {
        case 0:
          em_collection_ptr_->world_condition_.light_ = Light::kNatureLight;
          break;

        case 1:
          em_collection_ptr_->world_condition_.light_ = Light::kLampLight;
          break;

        case 2:
          em_collection_ptr_->world_condition_.light_ = Light::kHardLight;
          break;

        case 3:
          em_collection_ptr_->world_condition_.light_ = Light::kLowSun;
          break;

        case 4:
          em_collection_ptr_->world_condition_.light_ = Light::kDark;
          break;

        case 5:
          em_collection_ptr_->world_condition_.light_ = Light::kOther;
          break;

        default:
          em_collection_ptr_->world_condition_.light_ = Light::kUnknown;
          break;
      }
    }
    else
    {
      em_collection_ptr_->world_condition_.light_ = Light::kUnknown;
    }
  }
  else
  {
    em_collection_ptr_->world_condition_ = zone::data::em_data::WorldCondition();
  }
}

void Preprocess::TimeSyncObj() {}
void Preprocess::TimeSyncPerLane()
{
  /** 1. Time Sync for Per Lane based on discreted boundaries */
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_x_sync_ar;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_y_sync_ar;

  /** 2. Align to x_ref */
  // BoundaryPointAlignment(x_ref_ar_, t_x_sync_ar, t_y_sync_ar);
}

void Preprocess::PerLaneMapping(const LaneMarkings& per_lane_cs,
                                const bc::TCArray<LaneElementPoint, kMaxLaneNum>& ref_collection_ar,
                                const bc::uint8_t left_source_type_u8, const bc::uint8_t right_source_type_u8,
                                const bc::float32_t time_cycle_f)
{
  /** Mapping per lane info from input class to internal per_lane_collection */
  /** 0. Check perception lane validity.*/
  /** Curently consider 4 boundary lines: ll l r rr.*/
  bc::TCArray<bc::bool_t, 4> t_per_line_valid_ar = {bc::false_v, bc::false_v, bc::false_v, bc::false_v};
  LaneMarking t_l_line_cs = ::zone::fusion::sLaneMarking_t();
  LaneMarking t_r_line_cs = ::zone::fusion::sLaneMarking_t();
  LaneMarking t_ll_line_cs = ::zone::fusion::sLaneMarking_t();
  LaneMarking t_rr_line_cs = ::zone::fusion::sLaneMarking_t();
  for (uint8_t i = 0; i < kFusionLinesNum; ++i)
  {
    if (per_lane_cs.lines[i].line_position == 1)
    {
      t_l_line_cs = per_lane_cs.lines[i];
    }
    if (per_lane_cs.lines[i].line_position == 2)
    {
      t_r_line_cs = per_lane_cs.lines[i];
    }
    if (per_lane_cs.lines[i].line_position == 3)
    {
      t_ll_line_cs = per_lane_cs.lines[i];
    }
    if (per_lane_cs.lines[i].line_position == 4)
    {
      t_rr_line_cs = per_lane_cs.lines[i];
    }
    if (per_lane_cs.lines[i].confidence > FLOAT32_EPSILON && per_lane_cs.lines[i].lines_3d[0].t_max > FLOAT32_EPSILON)
    {
      new_per_life_time_ar_[i].line_id_s32_ = per_lane_cs.lines[i].line_id;
      new_per_life_time_ar_[i].initial_life_time_f_ = per_lane_cs.lines[i].life_time / 1000.f;
      new_per_life_time_ar_[i].valid_b_ = bc::true_v;
    }
    else
    {
      new_per_life_time_ar_[i] = PerLifeTimeCollection();
    }
    /// TODO: use horizon other lines
  }
  /// life time
  bc::bool_t t_find_relat_id_b = bc::false_v;
  for (uint8_t i = 0; i < kFusionLinesNum; ++i)
  {
    if (new_per_life_time_ar_[i].valid_b_)
    {
      t_find_relat_id_b = bc::false_v;
      for (uint8_t j = 0; j < kFusionLinesNum; ++j)
      {
        if (lst_per_life_time_ar_[j].valid_b_ &&
            new_per_life_time_ar_[i].line_id_s32_ == lst_per_life_time_ar_[j].line_id_s32_)
        {
          new_per_life_time_ar_[i].used_life_time_f_ = lst_per_life_time_ar_[j].used_life_time_f_ + time_cycle_f;
          new_per_life_time_ar_[i].initial_life_time_f_ = lst_per_life_time_ar_[j].initial_life_time_f_;
          t_find_relat_id_b = bc::true_v;
          break;
        }
      }
      if (t_find_relat_id_b == bc::false_v)
      {
        new_per_life_time_ar_[i].used_life_time_f_ =
            new_per_life_time_ar_[i].initial_life_time_f_ - new_per_life_time_ar_[i].initial_life_time_f_;
      }
    }
  }
  lst_per_life_time_ar_ = new_per_life_time_ar_;

  // 在特殊场景下(Y型分叉)不再收录转存感知新识别的车道线
  if (modified_ref_b)
  {
    if (ref_collection_ar[kLeftLane].lane_element_point_ar_[0].left_point_cs_.per_line_id_s32_ ==
        per_lane_cs.lines[0].line_id)
    {
      t_ll_line_cs = t_l_line_cs;
      t_l_line_cs = LaneMarking();
    }
    if (ref_collection_ar[kRightLane].lane_element_point_ar_[0].right_point_cs_.per_line_id_s32_ ==
        per_lane_cs.lines[1].line_id)
    {
      t_rr_line_cs = t_r_line_cs;
      t_r_line_cs = LaneMarking();
    }
  }
  // 在特殊场景下(超宽变双车道)修改Ref
  LaneBoundaryPoint& t_host_lane_cs =
      ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0];
  // 不需要对地图源场景识别
  if (t_host_lane_cs.valid_b_ && t_host_lane_cs.element_id_ == 0)
  {
    BoundaryPoint& t_left_boundary_cs = t_host_lane_cs.left_point_cs_;
    BoundaryPoint& t_right_boundary_cs = t_host_lane_cs.right_point_cs_;
    bc::int8_t t_adding_lane_direction_s8 = 0;  // left: -1, right: 1
    // 两边都是由真实车道线追踪得到
    if (t_left_boundary_cs.valid_b_ && t_right_boundary_cs.valid_b_ && t_left_boundary_cs.per_line_id_s32_ != -1 &&
        t_right_boundary_cs.per_line_id_s32_ != -1 &&
        t_left_boundary_cs.per_line_id_s32_ != t_right_boundary_cs.per_line_id_s32_)
    {
      if (t_left_boundary_cs.per_line_id_s32_ == t_ll_line_cs.line_id &&
          t_right_boundary_cs.per_line_id_s32_ == t_r_line_cs.line_id && t_ll_line_cs.line_id != 0 &&
          t_l_line_cs.line_id != 0 && t_r_line_cs.line_id != 0 &&
          t_ll_line_cs.lines_3d[0].start_pt.y - t_l_line_cs.lines_3d[0].start_pt.y > 2.f &&
          t_l_line_cs.lines_3d[0].start_pt.y - t_r_line_cs.lines_3d[0].start_pt.y > 2.f)
      {
        t_adding_lane_direction_s8 = -1;
        one_to_two_lane_ctn_u8_++;
      }
      else if (t_left_boundary_cs.per_line_id_s32_ == t_l_line_cs.line_id &&
               t_right_boundary_cs.per_line_id_s32_ == t_rr_line_cs.line_id && t_l_line_cs.line_id != 0 &&
               t_r_line_cs.line_id != 0 && t_rr_line_cs.line_id != 0 &&
               t_l_line_cs.lines_3d[0].start_pt.y - t_r_line_cs.lines_3d[0].start_pt.y > 2.f &&
               t_r_line_cs.lines_3d[0].start_pt.y - t_rr_line_cs.lines_3d[0].start_pt.y > 2.f)
      {
        t_adding_lane_direction_s8 = 1;
        one_to_two_lane_ctn_u8_++;
      }
      else
      {
        one_to_two_lane_ctn_u8_ = 0;
      }
      if (one_to_two_lane_ctn_u8_ == 5)
      {
        if (t_adding_lane_direction_s8 == -1)
        {
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLLLane] =
              ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane];
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].left_point_cs_ =
              t_left_boundary_cs;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].valid_b_ = bc::true_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].valid_b_ = bc::true_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].element_id_ = 0;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].right_point_cs_ =
              BoundaryPoint();
          ego_track_lane_collection_cs_.ego_track_point_ar_[kLeftLane].lane_element_point_ar_[0].map_unavailable_b_ =
              bc::false_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0].left_point_cs_ =
              BoundaryPoint();
        }
        else if (t_adding_lane_direction_s8 == 1)
        {
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRRLane] =
              ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane];
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].right_point_cs_ =
              t_right_boundary_cs;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].valid_b_ = bc::true_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].valid_b_ = bc::true_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].element_id_ = 0;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].left_point_cs_ =
              BoundaryPoint();
          ego_track_lane_collection_cs_.ego_track_point_ar_[kRightLane].lane_element_point_ar_[0].map_unavailable_b_ =
              bc::false_v;
          ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0].right_point_cs_ =
              BoundaryPoint();
        }
        one_to_two_lane_ctn_u8_ = 0;
      }
      // #ifdef STD_COUT_ENABLE
      //       std::cout << "one_to_two_lane_ctn_u8_: " << (bc::float32_t)one_to_two_lane_ctn_u8_ << std::endl;
      // #endif
    }
  }

  /**  consider per lines longth and overlapping .*/
  bc::int8_t t_ll_proj_boud_s8 = 0;
  bc::int8_t t_rr_proj_boud_s8 = 1;
  bc::bool_t t_predicted_b = bc::false_v;
  bc::bool_t t_stopline_b = bc::false_v;
  if (t_ll_line_cs.confidence > FLOAT32_EPSILON && t_ll_line_cs.lines_3d[0].t_max > FLOAT32_EPSILON)
  {
    // t_predicted_b = (t_ll_line_cs.line_source >> 12) & 1;
    // t_stopline_b = (t_ll_line_cs.line_source >> 25) & 1;
    if ((t_predicted_b == bc::false_v) && (t_stopline_b == bc::false_v))
    {
      t_per_line_valid_ar[0] = bc::true_v;
    }
  }
  if (t_l_line_cs.confidence > FLOAT32_EPSILON && t_l_line_cs.lines_3d[0].t_max > FLOAT32_EPSILON)
  {
    // t_predicted_b = (t_l_line_cs.line_source >> 12) & 1;
    // t_stopline_b = (t_l_line_cs.line_source >> 25) & 1;
    if ((t_predicted_b == bc::false_v) && (t_stopline_b == bc::false_v))
    {
      t_per_line_valid_ar[1] = bc::true_v;
    }
  }
  if (t_r_line_cs.confidence > FLOAT32_EPSILON && t_r_line_cs.lines_3d[0].t_max > FLOAT32_EPSILON)
  {
    // t_predicted_b = (t_r_line_cs.line_source >> 12) & 1;
    // t_stopline_b = (t_r_line_cs.line_source >> 25) & 1;
    if ((t_predicted_b == bc::false_v) && (t_stopline_b == bc::false_v))
    {
      t_per_line_valid_ar[2] = bc::true_v;
    }
  }
  if (t_rr_line_cs.confidence > FLOAT32_EPSILON && t_rr_line_cs.lines_3d[0].t_max > FLOAT32_EPSILON)
  {
    // t_predicted_b = (t_rr_line_cs.line_source >> 12) & 1;
    // t_stopline_b = (t_rr_line_cs.line_source >> 25) & 1;
    if ((t_predicted_b == bc::false_v) && (t_stopline_b == bc::false_v))
    {
      t_per_line_valid_ar[3] = bc::true_v;
    }
  }

  // 预处理感知车道线几何形状
  LaneMarking t_per_line_cs = ::zone::fusion::sLaneMarking_t();
  for (bc::uint8_t i = 0; i < kMaxPerBoundary; ++i)
  {
    per_mapping_collection_ar_[i] = ComparedBoundaryCollection();
    if (!t_per_line_valid_ar[i])
    {
      per_mapping_collection_ar_[i].valid_b_ = bc::false_v;
      continue;
    }
    else
    {
      if (i == 0)
      {
        t_per_line_cs = t_ll_line_cs;
      }
      else if (i == 1)
      {
        t_per_line_cs = t_l_line_cs;
      }
      else if (i == 2)
      {
        t_per_line_cs = t_r_line_cs;
      }
      else if (i == 3)
      {
        t_per_line_cs = t_rr_line_cs;
      }
      else
      {
        continue;
      }
      // 根据上一个周期计算得出的纵向间隔离散化感知车道线系数
      ClothoidModel t_per_coeff_cs;
      t_per_coeff_cs.c0_position = t_per_line_cs.lines_3d[0].y_coeffs[0];
      t_per_coeff_cs.c1_heading_angle = t_per_line_cs.lines_3d[0].y_coeffs[1];
      t_per_coeff_cs.c2_curvature = t_per_line_cs.lines_3d[0].y_coeffs[2];
      t_per_coeff_cs.c3_curvature_derivative = t_per_line_cs.lines_3d[0].y_coeffs[3];
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; ++t_idx_u8)
      {
        per_mapping_collection_ar_[i].dy_ar_[t_idx_u8] = t_per_coeff_cs.GetDyFromCloModel(x_ref_ar_[t_idx_u8]);
      }
      per_mapping_collection_ar_[i].valid_b_ = bc::true_v;
    }
    per_mapping_collection_ar_[i].per_x_f_ = t_per_line_cs.lines_3d[0].start_pt.x;
    per_mapping_collection_ar_[i].per_y_f_ = t_per_line_cs.lines_3d[0].start_pt.y;
    per_mapping_collection_ar_[i].per_length_f_ =
        t_per_line_cs.lines_3d[0].start_pt.x + t_per_line_cs.lines_3d[0].t_max;
  }

  // 对历史环境模型的车道线进行初筛, 用start_x对应的y作为准入条件选择最多kMaxComparedBoundary条和感知较贴合的线
  if (ref_collection_ar[kHostLane].lane_element_point_ar_[0].valid_b_ &&
      ((left_source_type_u8 != 0 && left_source_type_u8 != 4 && left_source_type_u8 != 8 &&
        left_source_type_u8 != 24) ||
       (right_source_type_u8 != 0 && right_source_type_u8 != 4 && right_source_type_u8 != 8 &&
        right_source_type_u8 != 24)))
  {
    for (bc::uint8_t i = 0; i < kMaxPerBoundary; ++i)
    {
      if (per_mapping_collection_ar_[i].valid_b_)
      {
        for (bc::uint8_t lane_idx = 0; lane_idx < kMaxLaneNum; ++lane_idx)
        {
          if (ref_collection_ar[lane_idx].valid_b_)
          {
            for (bc::uint8_t element_idx = 0; element_idx < kMaxElemNumInOneLane; ++element_idx)
            {
              if (ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].valid_b_)
              {
                //每条感知boundary最多对应三条上个cycle的em边线
                FilterSimilarBoundary(ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].left_point_cs_,
                                      lane_idx, element_idx, kLeftBoundary, per_mapping_collection_ar_[i]);
                FilterSimilarBoundary(ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].right_point_cs_,
                                      lane_idx, element_idx, kRightBoundary, per_mapping_collection_ar_[i]);
              }
            }
          }
        }
        // 计算筛选出的候选车道线与感知的likelihood, 选择likelihood最高的作为成功匹配车道线
        bc::float32_t t_matched_likelihood_f = 0.f;
        for (bc::uint8_t valid_idx = 0; valid_idx < per_mapping_collection_ar_[i].result_count_u8_; ++valid_idx)
        {
          PerMappingResult& t_cur_mapping_result_cs = per_mapping_collection_ar_[i].compared_boundary_ar_[valid_idx];
          bc::float32_t t_cur_likelihood_f = 0.f;
          if (t_cur_mapping_result_cs.boundary_position_u8_ == kLeftBoundary)
          {
            t_cur_likelihood_f =
                CheckMapPerLaneSimilarity(x_ref_ar_, per_mapping_collection_ar_[i].dy_ar_,
                                          ref_collection_ar[t_cur_mapping_result_cs.lane_idx_u8_]
                                              .lane_element_point_ar_[t_cur_mapping_result_cs.element_idx_u8_]
                                              .left_point_cs_.dy_ar_,
                                          per_mapping_collection_ar_[i].start_idx_u8_,
                                          std::min(ref_collection_ar[t_cur_mapping_result_cs.lane_idx_u8_]
                                                       .lane_element_point_ar_[t_cur_mapping_result_cs.element_idx_u8_]
                                                       .left_point_cs_.dy_end_idx_u8_,
                                                   per_mapping_collection_ar_[i].end_idx_u8_));
            t_cur_mapping_result_cs.likelihood_f_ = t_cur_likelihood_f;
          }
          else if (t_cur_mapping_result_cs.boundary_position_u8_ == kRightBoundary)
          {
            t_cur_likelihood_f =
                CheckMapPerLaneSimilarity(x_ref_ar_, per_mapping_collection_ar_[i].dy_ar_,
                                          ref_collection_ar[t_cur_mapping_result_cs.lane_idx_u8_]
                                              .lane_element_point_ar_[t_cur_mapping_result_cs.element_idx_u8_]
                                              .right_point_cs_.dy_ar_,
                                          per_mapping_collection_ar_[i].start_idx_u8_,
                                          std::min(ref_collection_ar[t_cur_mapping_result_cs.lane_idx_u8_]
                                                       .lane_element_point_ar_[t_cur_mapping_result_cs.element_idx_u8_]
                                                       .right_point_cs_.dy_end_idx_u8_,
                                                   per_mapping_collection_ar_[i].end_idx_u8_));
            t_cur_mapping_result_cs.likelihood_f_ = t_cur_likelihood_f;
          }
          if (t_cur_likelihood_f > t_matched_likelihood_f)
          {
            t_matched_likelihood_f = t_cur_likelihood_f;
            per_mapping_collection_ar_[i].max_likelihood_idx_u8_ = valid_idx;
          }
        }
      }
    }
    // 所有线形匹配完成后再做一次位置调整, 解决多个历史车道线对同一根感知车道线的匹配程度都很高的情况
    PerRefMatchingPostProcess(per_mapping_collection_ar_);
  }
  else
  {
    if (per_mapping_collection_ar_[0].valid_b_)
    {
      per_mapping_collection_ar_[0].target_lane_idx_u8_ = 1;
      per_mapping_collection_ar_[0].target_element_idx_u8_ = 0;
      per_mapping_collection_ar_[0].target_boundary_position_u8_ = kLeftBoundary;
    }
    if (per_mapping_collection_ar_[3].valid_b_)
    {
      per_mapping_collection_ar_[3].target_lane_idx_u8_ = 3;
      per_mapping_collection_ar_[3].target_element_idx_u8_ = 0;
      per_mapping_collection_ar_[3].target_boundary_position_u8_ = kRightBoundary;
    }
    if (per_mapping_collection_ar_[1].valid_b_ && per_mapping_collection_ar_[1].dy_ar_[20] >= 0.f)
    {
      per_mapping_collection_ar_[1].target_lane_idx_u8_ = 2;
      per_mapping_collection_ar_[1].target_element_idx_u8_ = 0;
      per_mapping_collection_ar_[1].target_boundary_position_u8_ = kLeftBoundary;
    }
    if (per_mapping_collection_ar_[2].valid_b_ && per_mapping_collection_ar_[2].dy_ar_[20] <= 0.f)
    {
      per_mapping_collection_ar_[2].target_lane_idx_u8_ = 2;
      per_mapping_collection_ar_[2].target_element_idx_u8_ = 0;
      per_mapping_collection_ar_[2].target_boundary_position_u8_ = kRightBoundary;
    }
    // 防止在十字路口左右转的时候用错误的左右边线构造自车道
    if (per_mapping_collection_ar_[1].valid_b_ && per_mapping_collection_ar_[1].dy_ar_[20] < 0.f)
    {
      per_mapping_collection_ar_[2] = ComparedBoundaryCollection();
    }
    if (per_mapping_collection_ar_[2].valid_b_ && per_mapping_collection_ar_[2].dy_ar_[20] > 0.f)
    {
      per_mapping_collection_ar_[1] = ComparedBoundaryCollection();
    }
    // 清空曾经tracking的车道宽等数据
    per_lane_width_lst1_f_ = kDefaultElementWidth;
    per_left_c0_lst_f_ = 0.f;
    per_right_c0_lst_f_ = 0.f;
    target_width_f_ = kDefaultElementWidth;
    per_lane_change_state_ = PerLaneChangeState::kDrvStraight;
    per_lane_width_filter_state_ar_ = {PerLaneWidthFilterState::kDefault};
  }

  // 将匹配的结果存储到per_lane_collection_cs_
  if (!((t_ll_line_cs.line_source >> 12) & 1) && per_mapping_collection_ar_[0].valid_b_ &&
      per_mapping_collection_ar_[0].target_boundary_position_u8_ != -1)
  {
    if (!ref_collection_ar[per_mapping_collection_ar_[0].target_lane_idx_u8_]
             .lane_element_point_ar_[per_mapping_collection_ar_[0].target_element_idx_u8_]
             .valid_b_)
    {
      StorePerLineToInternal(t_ll_line_cs, per_mapping_collection_ar_[0].target_lane_idx_u8_,
                             per_mapping_collection_ar_[0].target_element_idx_u8_,
                             per_mapping_collection_ar_[0].target_boundary_position_u8_);
    }
    else
    {
      StorePerLineToInternalWithMatchingId(t_ll_line_cs, per_mapping_collection_ar_[0].target_lane_idx_u8_,
                                           per_mapping_collection_ar_[0].target_element_idx_u8_,
                                           per_mapping_collection_ar_[0].target_boundary_position_u8_,
                                           ref_collection_ar);
    }
  }
  if (!((t_l_line_cs.line_source >> 12) & 1) && per_mapping_collection_ar_[1].valid_b_)
  {
    if (!ref_collection_ar[per_mapping_collection_ar_[1].target_lane_idx_u8_]
             .lane_element_point_ar_[per_mapping_collection_ar_[1].target_element_idx_u8_]
             .valid_b_)
    {
      StorePerLineToInternal(t_l_line_cs, per_mapping_collection_ar_[1].target_lane_idx_u8_,
                             per_mapping_collection_ar_[1].target_element_idx_u8_,
                             per_mapping_collection_ar_[1].target_boundary_position_u8_);
    }
    else
    {
      StorePerLineToInternalWithMatchingId(t_l_line_cs, per_mapping_collection_ar_[1].target_lane_idx_u8_,
                                           per_mapping_collection_ar_[1].target_element_idx_u8_,
                                           per_mapping_collection_ar_[1].target_boundary_position_u8_,
                                           ref_collection_ar);
    }
  }
  if (!((t_r_line_cs.line_source >> 12) & 1) && per_mapping_collection_ar_[2].valid_b_)
  {
    if (!ref_collection_ar[per_mapping_collection_ar_[2].target_lane_idx_u8_]
             .lane_element_point_ar_[per_mapping_collection_ar_[2].target_element_idx_u8_]
             .valid_b_)
    {
      StorePerLineToInternal(t_r_line_cs, per_mapping_collection_ar_[2].target_lane_idx_u8_,
                             per_mapping_collection_ar_[2].target_element_idx_u8_,
                             per_mapping_collection_ar_[2].target_boundary_position_u8_);
    }
    else
    {
      StorePerLineToInternalWithMatchingId(t_r_line_cs, per_mapping_collection_ar_[2].target_lane_idx_u8_,
                                           per_mapping_collection_ar_[2].target_element_idx_u8_,
                                           per_mapping_collection_ar_[2].target_boundary_position_u8_,
                                           ref_collection_ar);
    }
  }
  if (!((t_rr_line_cs.line_source >> 12) & 1) && per_mapping_collection_ar_[3].valid_b_ &&
      per_mapping_collection_ar_[3].target_boundary_position_u8_ != -1)
  {
    if (!ref_collection_ar[per_mapping_collection_ar_[3].target_lane_idx_u8_]
             .lane_element_point_ar_[per_mapping_collection_ar_[3].target_element_idx_u8_]
             .valid_b_)
    {
      StorePerLineToInternal(t_rr_line_cs, per_mapping_collection_ar_[3].target_lane_idx_u8_,
                             per_mapping_collection_ar_[3].target_element_idx_u8_,
                             per_mapping_collection_ar_[3].target_boundary_position_u8_);
    }
    else
    {
      StorePerLineToInternalWithMatchingId(t_rr_line_cs, per_mapping_collection_ar_[3].target_lane_idx_u8_,
                                           per_mapping_collection_ar_[3].target_element_idx_u8_,
                                           per_mapping_collection_ar_[3].target_boundary_position_u8_,
                                           ref_collection_ar);
    }
  }
}

void Preprocess::DoRefPerceptionMatch(const PerLaneCollection& per_collection_cs)
{
  /** 0. Find out and mark paralleled lane. */
  /// TODO: Use coeff or some math and statistic approach to guess the paralleled lane
  /// Only modify coeff if the left and right boundary are paralleled

  /** 1. Reconstruct perception lines relationship.
   *  应用于复杂场景新出现的车道线，比如Y型分叉的中间车道线
   *  TODO: 完成下方的raw code
  */
  if (ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0].element_id_ != 0 &&
      !per_mapping_collection_ar_[1].valid_b_ && per_mapping_collection_ar_[1].result_count_u8_ > 0)
  {
    // if (左边线和右边线是平行车道)
    //   1. 判断原来的左边线和现在的左边线是否有交叉
    //      若有交叉则将原来的左边线移至左车道右边线处
    //      否则移至左车道左边线处
    //      需要同时移动对应的ref
    //   2. 替换per_collection_cs自车道左边线
    //      同时清空对应的ref
    // else if (左边线和左左边线是平行车道)
    //   1. 原来的左边线是现在的左左边线则直接平移原来的左边线和对应的ref至左车道左边线处
    //   2. 暂时还没想到其他的case
    // else (default 左边线和右边线是平行车道)
  }
  if (ego_track_lane_collection_cs_.ego_track_point_ar_[kHostLane].lane_element_point_ar_[0].element_id_ != 0 &&
      !per_mapping_collection_ar_[2].valid_b_ && per_mapping_collection_ar_[2].result_count_u8_ > 0)
  {
    // 同上
  }

  /** 2. Clear Ref if from map and no perception is available. */
  bc::TCArray<LaneElementPoint, kMaxLaneNum>& t_track_point_ar = ego_track_lane_collection_cs_.ego_track_point_ar_;
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (t_track_point_ar[t_lane_idx_u8].valid_b_)
    {
      bc::bool_t t_all_element_invalid_b = bc::true_v;
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].valid_b_)
        {
          if (t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].map_unavailable_b_ &&
              !per_collection_cs.per_coeff_ar_[t_lane_idx_u8].valid_b_)
          {
            t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8] = LaneBoundaryPoint();
          }
          else
          {
            t_all_element_invalid_b = bc::false_v;
          }
        }
      }
      if (t_all_element_invalid_b)
      {
        t_track_point_ar[t_lane_idx_u8].valid_b_ = bc::false_v;
      }
    }
  }
  /** 3. Clear Ref if generated only from ref but position is too far away. */
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    if (t_track_point_ar[t_lane_idx_u8].valid_b_)
    {
      bc::bool_t t_all_element_invalid_b = bc::true_v;
      for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
      {
        if (t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].valid_b_)
        {
          BoundaryPoint& t_left_boundary_point_cs =
              t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].left_point_cs_;
          BoundaryPoint& t_right_boundary_point_cs =
              t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].right_point_cs_;
          bc::float32_t t_ego_width_f = t_left_boundary_point_cs.dy_ar_[20] - t_right_boundary_point_cs.dy_ar_[20];
          bc::float32_t t_end_width_f = t_left_boundary_point_cs.dy_ar_[t_left_boundary_point_cs.dy_end_idx_u8_] -
                                        t_right_boundary_point_cs.dy_ar_[t_right_boundary_point_cs.dy_end_idx_u8_];
          if (((t_lane_idx_u8 < kHostLane && t_left_boundary_point_cs.per_line_id_s32_ == -1) ||
               (t_lane_idx_u8 > kHostLane && t_right_boundary_point_cs.per_line_id_s32_ == -1)) &&
              t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8].element_id_ == 0 &&
              fabsf(t_ego_width_f - t_end_width_f) > 5.f)
          {
            t_track_point_ar[t_lane_idx_u8].lane_element_point_ar_[t_ele_idx_u8] = LaneBoundaryPoint();
          }
          else
          {
            t_all_element_invalid_b = bc::false_v;
          }
        }
      }
      if (t_all_element_invalid_b)
      {
        t_track_point_ar[t_lane_idx_u8].valid_b_ = bc::false_v;
      }
    }
  }
}

bc::float32_t Preprocess::CheckMapPerLaneSimilarity(const bc::TCArray<bc::float32_t, kMaxNumBoundary> cur_x_ref,
                                                    const bc::TCArray<bc::float32_t, kMaxNumBoundary>& per_boundary,
                                                    const bc::TCArray<bc::float32_t, kMaxNumBoundary>& ref_boundary,
                                                    const bc::uint8_t start_index, bc::uint8_t end_index)
{
  bc::float32_t t_map_sigma_f = 0.5f;
  bc::float32_t t_per_sigma_f = 0.5f;
  bc::float32_t t_sigma_f = sqrt(pow(t_map_sigma_f, 2) + pow(t_per_sigma_f, 2));
  // bc::float32_t t_sigma_f = 0.15f;
  bc::float32_t t_dy_diff_f = 0;
  bc::float32_t t_s_scale_f = 30;
  bc::float32_t t_likelihood_f = 0.f;
  bc::float32_t t_weight_f = 0.f;
  bc::float32_t t_weight_sum_f = 0.f;
  bc::float32_t t_weight_likelihood_sum_f = 0.f;
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_s_ar = {0.f};
  bc::float32_t t_dis_dead_zone = 0.1f;

  if (start_index >= kFrontStartIdx)
  {
    if (start_index == kFrontStartIdx)
    {
      t_s_ar[start_index] = 0.f;
    }
    else
    {
      t_s_ar[start_index] = sqrt(pow(ref_boundary[start_index] - ref_boundary[kFrontStartIdx], 2) +
                                 pow(cur_x_ref[start_index] - cur_x_ref[kFrontStartIdx], 2));
    }
    for (bc::uint8_t t_point_idx_u8 = start_index + 1; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 - 1] + sqrt(pow(ref_boundary[t_point_idx_u8 - 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(cur_x_ref[t_point_idx_u8 - 1] - cur_x_ref[t_point_idx_u8], 2));
    }
  }
  else if (end_index <= kFrontStartIdx)
  {
    if (end_index == kFrontStartIdx)
    {
      t_s_ar[end_index] = 0.f;
    }
    else
    {
      t_s_ar[end_index] = sqrt(pow(ref_boundary[end_index] - ref_boundary[kFrontStartIdx], 2) +
                               pow(cur_x_ref[end_index] - cur_x_ref[kFrontStartIdx], 2));
    }
    for (bc::int8_t t_point_idx_u8 = end_index - 1; t_point_idx_u8 >= start_index; --t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 + 1] + sqrt(pow(ref_boundary[t_point_idx_u8 + 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(cur_x_ref[t_point_idx_u8 + 1] - cur_x_ref[t_point_idx_u8], 2));
    }
  }
  else
  {
    t_s_ar[kFrontStartIdx] = 0.f;
    for (bc::uint8_t t_point_idx_u8 = kFrontStartIdx + 1; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 - 1] + sqrt(pow(ref_boundary[t_point_idx_u8 - 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(cur_x_ref[t_point_idx_u8 - 1] - cur_x_ref[t_point_idx_u8], 2));
    }
    for (bc::int8_t t_point_idx_u8 = kFrontStartIdx - 1; t_point_idx_u8 >= start_index; --t_point_idx_u8)
    {
      t_s_ar[t_point_idx_u8] =
          t_s_ar[t_point_idx_u8 + 1] + sqrt(pow(ref_boundary[t_point_idx_u8 + 1] - ref_boundary[t_point_idx_u8], 2) +
                                            pow(cur_x_ref[t_point_idx_u8 + 1] - cur_x_ref[t_point_idx_u8], 2));
    }
  }

  for (bc::uint8_t t_point_idx_u8 = start_index; t_point_idx_u8 <= end_index; ++t_point_idx_u8)
  {
    t_dy_diff_f = bc::abs(ref_boundary[t_point_idx_u8] - per_boundary[t_point_idx_u8]);
    t_dy_diff_f -= t_dis_dead_zone;
    if (t_dy_diff_f < 0.f)
    {
      t_dy_diff_f = 0.f;
    }
    t_likelihood_f = exp(-t_dy_diff_f * t_dy_diff_f / (2 * t_sigma_f * t_sigma_f));
    t_weight_f = exp(-t_s_ar[t_point_idx_u8] * t_s_ar[t_point_idx_u8] / (t_s_scale_f * t_s_scale_f));
    t_weight_sum_f += t_weight_f;
    t_weight_likelihood_sum_f += t_weight_f * t_likelihood_f;
  }
  t_weight_likelihood_sum_f = t_weight_likelihood_sum_f / t_weight_sum_f;

  return t_weight_likelihood_sum_f;
}

void Preprocess::PerRefMatchingPostProcess(bc::TCArray<ComparedBoundaryCollection, kMaxPerBoundary>& per_ref_mapping)
{
  // index: 左边线 1, 右边线 2, 左左边线 0, 右右边线 3
  // 优先匹配左右边线
  if (per_ref_mapping[1].max_likelihood_idx_u8_ == kMaxComparedBoundary)
  {
    per_ref_mapping[1].valid_b_ = bc::false_v;
  }
  if (per_ref_mapping[2].max_likelihood_idx_u8_ == kMaxComparedBoundary)
  {
    per_ref_mapping[2].valid_b_ = bc::false_v;
  }
  if (per_ref_mapping[1].valid_b_ && per_ref_mapping[2].valid_b_)
  {
    PerMappingResult& t_left_result_cs =
        per_ref_mapping[1].compared_boundary_ar_[per_ref_mapping[1].max_likelihood_idx_u8_];
    PerMappingResult& t_right_result_cs =
        per_ref_mapping[2].compared_boundary_ar_[per_ref_mapping[2].max_likelihood_idx_u8_];
    if ((t_left_result_cs.lane_idx_u8_ == t_right_result_cs.lane_idx_u8_) &&
        (t_left_result_cs.element_idx_u8_ == t_right_result_cs.element_idx_u8_) &&
        (t_left_result_cs.boundary_position_u8_ == t_right_result_cs.boundary_position_u8_))
    {
      if (t_left_result_cs.likelihood_f_ > t_right_result_cs.likelihood_f_)
      {
        per_ref_mapping[1].target_lane_idx_u8_ = t_left_result_cs.lane_idx_u8_;
        per_ref_mapping[1].target_element_idx_u8_ = t_left_result_cs.element_idx_u8_;
        per_ref_mapping[1].target_boundary_position_u8_ = t_left_result_cs.boundary_position_u8_;
        bc::float32_t t_secondary_likelihood_f = 0.f;
        bc::bool_t t_find_secondary_likelihood_b = bc::false_v;
        for (bc::uint8_t i = 0; i < per_ref_mapping[2].result_count_u8_; ++i)
        {
          PerMappingResult& t_cur_result_cs = per_ref_mapping[2].compared_boundary_ar_[i];
          if (i != per_ref_mapping[2].max_likelihood_idx_u8_ &&
              t_cur_result_cs.likelihood_f_ > t_secondary_likelihood_f)
          {
            t_secondary_likelihood_f = t_cur_result_cs.likelihood_f_;
            per_ref_mapping[2].target_lane_idx_u8_ = t_cur_result_cs.lane_idx_u8_;
            per_ref_mapping[2].target_element_idx_u8_ = t_cur_result_cs.element_idx_u8_;
            per_ref_mapping[2].target_boundary_position_u8_ = t_cur_result_cs.boundary_position_u8_;
            t_find_secondary_likelihood_b = bc::true_v;
          }
        }
        if (!t_find_secondary_likelihood_b && modified_ref_b)
        {
          if (per_ref_mapping[1].target_lane_idx_u8_ == kHostLane &&
              per_ref_mapping[1].target_boundary_position_u8_ == 1)
          {
            per_ref_mapping[2].target_lane_idx_u8_ = kRightLane;
            per_ref_mapping[2].target_element_idx_u8_ = 0;
            per_ref_mapping[2].target_boundary_position_u8_ = 1;
          }
        }
      }
      else
      {
        per_ref_mapping[2].target_lane_idx_u8_ = t_right_result_cs.lane_idx_u8_;
        per_ref_mapping[2].target_element_idx_u8_ = t_right_result_cs.element_idx_u8_;
        per_ref_mapping[2].target_boundary_position_u8_ = t_right_result_cs.boundary_position_u8_;
        bc::float32_t t_secondary_likelihood_f = 0.f;
        bc::bool_t t_find_secondary_likelihood_b = bc::false_v;
        for (bc::uint8_t i = 0; i < per_ref_mapping[1].result_count_u8_; ++i)
        {
          PerMappingResult& t_cur_result_cs = per_ref_mapping[1].compared_boundary_ar_[i];
          if (i != per_ref_mapping[1].max_likelihood_idx_u8_ &&
              t_cur_result_cs.likelihood_f_ > t_secondary_likelihood_f)
          {
            t_secondary_likelihood_f = t_cur_result_cs.likelihood_f_;
            per_ref_mapping[1].target_lane_idx_u8_ = t_cur_result_cs.lane_idx_u8_;
            per_ref_mapping[1].target_element_idx_u8_ = t_cur_result_cs.element_idx_u8_;
            per_ref_mapping[1].target_boundary_position_u8_ = t_cur_result_cs.boundary_position_u8_;
            t_find_secondary_likelihood_b = bc::true_v;
          }
        }
        if (!t_find_secondary_likelihood_b && modified_ref_b)
        {
          if (per_ref_mapping[2].target_lane_idx_u8_ == kHostLane &&
              per_ref_mapping[2].target_boundary_position_u8_ == 0)
          {
            per_ref_mapping[1].target_lane_idx_u8_ = kLeftLane;
            per_ref_mapping[1].target_element_idx_u8_ = 0;
            per_ref_mapping[1].target_boundary_position_u8_ = 0;
          }
        }
      }
    }
    else
    {
      per_ref_mapping[1].target_lane_idx_u8_ = t_left_result_cs.lane_idx_u8_;
      per_ref_mapping[1].target_element_idx_u8_ = t_left_result_cs.element_idx_u8_;
      per_ref_mapping[1].target_boundary_position_u8_ = t_left_result_cs.boundary_position_u8_;
      per_ref_mapping[2].target_lane_idx_u8_ = t_right_result_cs.lane_idx_u8_;
      per_ref_mapping[2].target_element_idx_u8_ = t_right_result_cs.element_idx_u8_;
      per_ref_mapping[2].target_boundary_position_u8_ = t_right_result_cs.boundary_position_u8_;
    }
  }
  else if (per_ref_mapping[1].valid_b_)
  {
    PerMappingResult& t_left_result_cs =
        per_ref_mapping[1].compared_boundary_ar_[per_ref_mapping[1].max_likelihood_idx_u8_];
    per_ref_mapping[1].target_lane_idx_u8_ = t_left_result_cs.lane_idx_u8_;
    per_ref_mapping[1].target_element_idx_u8_ = t_left_result_cs.element_idx_u8_;
    per_ref_mapping[1].target_boundary_position_u8_ = t_left_result_cs.boundary_position_u8_;
  }
  else if (per_ref_mapping[2].valid_b_)
  {
    PerMappingResult& t_right_result_cs =
        per_ref_mapping[2].compared_boundary_ar_[per_ref_mapping[2].max_likelihood_idx_u8_];
    per_ref_mapping[2].target_lane_idx_u8_ = t_right_result_cs.lane_idx_u8_;
    per_ref_mapping[2].target_element_idx_u8_ = t_right_result_cs.element_idx_u8_;
    per_ref_mapping[2].target_boundary_position_u8_ = t_right_result_cs.boundary_position_u8_;
  }
  // 匹配左左和右右边线
  if (per_ref_mapping[0].valid_b_)
  {
    bc::bool_t t_mapping_left_lane_left_line =
        per_ref_mapping[1].valid_b_ &&
        !(per_ref_mapping[1].target_lane_idx_u8_ == kLeftLane && per_ref_mapping[1].target_element_idx_u8_ == 0 &&
          per_ref_mapping[1].target_boundary_position_u8_ == 0);
    if (per_ref_mapping[0].max_likelihood_idx_u8_ == kMaxComparedBoundary)
    {
      if (t_mapping_left_lane_left_line)
      {
        per_ref_mapping[0].target_lane_idx_u8_ = kLeftLane;
        per_ref_mapping[0].target_element_idx_u8_ = 0;
        per_ref_mapping[0].target_boundary_position_u8_ = 0;
      }
    }
    else
    {
      PerMappingResult& t_ll_result_cs =
          per_ref_mapping[0].compared_boundary_ar_[per_ref_mapping[0].max_likelihood_idx_u8_];
      if ((t_ll_result_cs.lane_idx_u8_ != per_ref_mapping[1].target_lane_idx_u8_ ||
           t_ll_result_cs.element_idx_u8_ != per_ref_mapping[1].target_element_idx_u8_ ||
           t_ll_result_cs.boundary_position_u8_ != per_ref_mapping[1].target_boundary_position_u8_) &&
          (t_ll_result_cs.lane_idx_u8_ != per_ref_mapping[2].target_lane_idx_u8_ ||
           t_ll_result_cs.element_idx_u8_ != per_ref_mapping[2].target_element_idx_u8_ ||
           t_ll_result_cs.boundary_position_u8_ != per_ref_mapping[2].target_boundary_position_u8_))
      {
        per_ref_mapping[0].target_lane_idx_u8_ = t_ll_result_cs.lane_idx_u8_;
        per_ref_mapping[0].target_element_idx_u8_ = t_ll_result_cs.element_idx_u8_;
        per_ref_mapping[0].target_boundary_position_u8_ = t_ll_result_cs.boundary_position_u8_;
      }
      else
      {
        bc::float32_t t_secondary_likelihood_f = 0.f;
        bc::bool_t t_find_secondary_b = bc::false_v;
        for (bc::uint8_t i = 0; i < per_ref_mapping[0].result_count_u8_; ++i)
        {
          PerMappingResult& t_cur_result_cs = per_ref_mapping[0].compared_boundary_ar_[i];
          if (i != per_ref_mapping[0].max_likelihood_idx_u8_ &&
              t_cur_result_cs.likelihood_f_ > t_secondary_likelihood_f &&
              t_cur_result_cs.matching_id_u8_ != t_ll_result_cs.matching_id_u8_ && t_cur_result_cs.gap_y_f_ < 1.f)
          {
            t_secondary_likelihood_f = t_cur_result_cs.likelihood_f_;
            per_ref_mapping[0].target_lane_idx_u8_ = t_cur_result_cs.lane_idx_u8_;
            per_ref_mapping[0].target_element_idx_u8_ = t_cur_result_cs.element_idx_u8_;
            per_ref_mapping[0].target_boundary_position_u8_ = t_cur_result_cs.boundary_position_u8_;
            t_find_secondary_b = bc::true_v;
          }
        }
        if (!t_find_secondary_b && t_mapping_left_lane_left_line)
        {
          per_ref_mapping[0].target_lane_idx_u8_ = kLeftLane;
          per_ref_mapping[0].target_element_idx_u8_ = 0;
          per_ref_mapping[0].target_boundary_position_u8_ = 0;
        }
      }
    }
  }
  if (per_ref_mapping[3].valid_b_)
  {
    bc::bool_t t_mapping_right_lane_right_line =
        per_ref_mapping[2].valid_b_ &&
        !(per_ref_mapping[2].target_lane_idx_u8_ == kRightLane && per_ref_mapping[2].target_element_idx_u8_ == 0 &&
          per_ref_mapping[2].target_boundary_position_u8_ == 1);
    if (per_ref_mapping[3].max_likelihood_idx_u8_ == kMaxComparedBoundary)
    {
      if (t_mapping_right_lane_right_line)
      {
        per_ref_mapping[3].target_lane_idx_u8_ = kRightLane;
        per_ref_mapping[3].target_element_idx_u8_ = 0;
        per_ref_mapping[3].target_boundary_position_u8_ = 1;
      }
    }
    else
    {
      PerMappingResult& t_rr_result_cs =
          per_ref_mapping[3].compared_boundary_ar_[per_ref_mapping[3].max_likelihood_idx_u8_];
      if ((t_rr_result_cs.lane_idx_u8_ != per_ref_mapping[0].target_lane_idx_u8_ ||
           t_rr_result_cs.element_idx_u8_ != per_ref_mapping[0].target_element_idx_u8_ ||
           t_rr_result_cs.boundary_position_u8_ != per_ref_mapping[0].target_boundary_position_u8_) &&
          (t_rr_result_cs.lane_idx_u8_ != per_ref_mapping[1].target_lane_idx_u8_ ||
           t_rr_result_cs.element_idx_u8_ != per_ref_mapping[1].target_element_idx_u8_ ||
           t_rr_result_cs.boundary_position_u8_ != per_ref_mapping[1].target_boundary_position_u8_) &&
          (t_rr_result_cs.lane_idx_u8_ != per_ref_mapping[2].target_lane_idx_u8_ ||
           t_rr_result_cs.element_idx_u8_ != per_ref_mapping[2].target_element_idx_u8_ ||
           t_rr_result_cs.boundary_position_u8_ != per_ref_mapping[2].target_boundary_position_u8_))
      {
        per_ref_mapping[3].target_lane_idx_u8_ = t_rr_result_cs.lane_idx_u8_;
        per_ref_mapping[3].target_element_idx_u8_ = t_rr_result_cs.element_idx_u8_;
        per_ref_mapping[3].target_boundary_position_u8_ = t_rr_result_cs.boundary_position_u8_;
      }
      else
      {
        bc::float32_t t_secondary_likelihood_f = 0.f;
        bc::bool_t t_find_secondary_b = bc::false_v;
        for (bc::uint8_t i = 0; i < per_ref_mapping[3].result_count_u8_; ++i)
        {
          PerMappingResult& t_cur_result_cs = per_ref_mapping[3].compared_boundary_ar_[i];
          if (i != per_ref_mapping[3].max_likelihood_idx_u8_ &&
              t_cur_result_cs.likelihood_f_ > t_secondary_likelihood_f &&
              t_cur_result_cs.matching_id_u8_ != t_rr_result_cs.matching_id_u8_ && t_cur_result_cs.gap_y_f_ < 1.f)
          {
            t_secondary_likelihood_f = t_cur_result_cs.likelihood_f_;
            per_ref_mapping[3].target_lane_idx_u8_ = t_cur_result_cs.lane_idx_u8_;
            per_ref_mapping[3].target_element_idx_u8_ = t_cur_result_cs.element_idx_u8_;
            per_ref_mapping[3].target_boundary_position_u8_ = t_cur_result_cs.boundary_position_u8_;
            t_find_secondary_b = bc::true_v;
          }
        }
        if (!t_find_secondary_b && t_mapping_right_lane_right_line)
        {
          per_ref_mapping[3].target_lane_idx_u8_ = kRightLane;
          per_ref_mapping[3].target_element_idx_u8_ = 0;
          per_ref_mapping[3].target_boundary_position_u8_ = 1;
        }
      }
    }
  }
}

void Preprocess::FilterSimilarBoundary(const BoundaryPoint& lane_boundary, const bc::uint8_t lane_idx,
                                       const bc::uint8_t element_idx, const bc::uint8_t boundary_flag,
                                       ComparedBoundaryCollection& boundary_collection)
{
  if (lane_boundary.valid_b_)
  {
    bc::float32_t current_boundary_x_f = FLOAT32_MAX;
    bc::float32_t current_boundary_y_f = FLOAT32_MAX;
    bc::bool_t t_find_start_idx = bc::false_v;
    bc::bool_t t_find_end_idx = bc::false_v;
    for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < kMaxBoundaryPoint; ++t_point_idx_u8)
    {
      if (!t_find_start_idx && boundary_collection.per_x_f_ <= x_ref_ar_[t_point_idx_u8])
      {
        current_boundary_x_f = x_ref_ar_[t_point_idx_u8];
        current_boundary_y_f = lane_boundary.dy_ar_[t_point_idx_u8];
        boundary_collection.start_idx_u8_ = t_point_idx_u8;
        t_find_start_idx = bc::true_v;
      }
      else if ((boundary_collection.per_x_f_ + boundary_collection.per_length_f_) <= x_ref_ar_[t_point_idx_u8])
      {
        boundary_collection.end_idx_u8_ = t_point_idx_u8 - 1;
        t_find_end_idx = bc::true_v;
        break;
      }
    }
    if (!t_find_end_idx)
    {
      boundary_collection.end_idx_u8_ = kMaxBoundaryPoint - 1;
    }
    bc::float32_t t_gap_y_f = fabsf(current_boundary_y_f - boundary_collection.per_y_f_);
    if (t_gap_y_f < kMaxDistanceGap)
    {
      if (boundary_collection.result_count_u8_ == kMaxComparedBoundary)
      {
        if (t_gap_y_f < boundary_collection.max_gap_y_f_)
        {
          PerMappingResult& t_cur_mapping_result_cs =
              boundary_collection.compared_boundary_ar_[boundary_collection.max_gap_y_idx_u8_];
          t_cur_mapping_result_cs.lane_idx_u8_ = lane_idx;
          t_cur_mapping_result_cs.element_idx_u8_ = element_idx;
          t_cur_mapping_result_cs.per_line_id_s32_ = lane_boundary.per_line_id_s32_;
          t_cur_mapping_result_cs.matching_id_u8_ = lane_boundary.matching_id_u8_;
          t_cur_mapping_result_cs.boundary_position_u8_ = boundary_flag;
          t_cur_mapping_result_cs.history_x_f_ = current_boundary_x_f;
          t_cur_mapping_result_cs.history_y_f_ = current_boundary_y_f;
          t_cur_mapping_result_cs.gap_y_f_ = t_gap_y_f;
        }
        bc::uint8_t t_max_gap_idx_u8 = 0;
        bc::float32_t t_max_gap_f = boundary_collection.compared_boundary_ar_[t_max_gap_idx_u8].gap_y_f_;
        for (bc::uint8_t result_idx = 1; result_idx < kMaxComparedBoundary; result_idx++)
        {
          PerMappingResult& t_cur_mapping_result_cs = boundary_collection.compared_boundary_ar_[result_idx];
          if (t_cur_mapping_result_cs.gap_y_f_ >= t_max_gap_f)
          {
            t_max_gap_idx_u8 = result_idx;
            t_max_gap_f = t_cur_mapping_result_cs.gap_y_f_;
          }
        }
        boundary_collection.max_gap_y_f_ = t_max_gap_f;
        boundary_collection.max_gap_y_idx_u8_ = t_max_gap_idx_u8;
      }
      else
      {
        PerMappingResult& t_cur_mapping_result_cs =
            boundary_collection.compared_boundary_ar_[boundary_collection.result_count_u8_];
        t_cur_mapping_result_cs.valid_b_ = bc::true_v;
        t_cur_mapping_result_cs.lane_idx_u8_ = lane_idx;
        t_cur_mapping_result_cs.element_idx_u8_ = element_idx;
        t_cur_mapping_result_cs.per_line_id_s32_ = lane_boundary.per_line_id_s32_;
        t_cur_mapping_result_cs.matching_id_u8_ = lane_boundary.matching_id_u8_;
        t_cur_mapping_result_cs.boundary_position_u8_ = boundary_flag;
        t_cur_mapping_result_cs.history_x_f_ = current_boundary_x_f;
        t_cur_mapping_result_cs.history_y_f_ = current_boundary_y_f;
        t_cur_mapping_result_cs.gap_y_f_ = t_gap_y_f;
        if (t_gap_y_f > boundary_collection.max_gap_y_f_)
        {
          boundary_collection.max_gap_y_f_ = t_gap_y_f;
          boundary_collection.max_gap_y_idx_u8_ = boundary_collection.result_count_u8_;
        }
        boundary_collection.result_count_u8_++;
      }
    }
  }
}

void Preprocess::StorePerLineToInternalWithMatchingId(
    const LaneMarking& external_cs, const bc::uint8_t target_lane_u8, const bc::uint8_t target_element_u8,
    const bc::int8_t target_boundary_u8, const bc::TCArray<LaneElementPoint, kMaxLaneNum>& ref_collection_ar)
{
  bc::uint8_t t_matching_id_u8 = 0;
  if (target_boundary_u8 == kLeftBoundary)
  {
    t_matching_id_u8 =
        ref_collection_ar[target_lane_u8].lane_element_point_ar_[target_element_u8].left_point_cs_.matching_id_u8_;
  }
  else if (target_boundary_u8 == kRightBoundary)
  {
    t_matching_id_u8 =
        ref_collection_ar[target_lane_u8].lane_element_point_ar_[target_element_u8].right_point_cs_.matching_id_u8_;
  }
  for (bc::uint8_t lane_idx = 0; lane_idx < kMaxLaneNum; ++lane_idx)
  {
    if (ref_collection_ar[lane_idx].valid_b_)
    {
      for (bc::uint8_t element_idx = 0; element_idx < kMaxElemNumInOneLane; ++element_idx)
      {
        if (ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].valid_b_)
        {
          if (ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].left_point_cs_.valid_b_ &&
              ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].left_point_cs_.matching_id_u8_ ==
                  t_matching_id_u8)
          {
            StorePerLineToInternal(external_cs, lane_idx, element_idx, kLeftBoundary);
          }
          else if (ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].right_point_cs_.valid_b_ &&
                   ref_collection_ar[lane_idx].lane_element_point_ar_[element_idx].right_point_cs_.matching_id_u8_ ==
                       t_matching_id_u8)
          {
            StorePerLineToInternal(external_cs, lane_idx, element_idx, kRightBoundary);
          }
        }
      }
    }
  }
}

void Preprocess::DiscretePerLane()
{
  /** Discrete each valid per lane boundary according to x_ref, and put results into per_lane_collection*/
  for (bc::uint8_t t_lane_u8 = 0; t_lane_u8 < kMaxLaneNum; ++t_lane_u8)
  {
    if (per_lane_collection_cs_.per_coeff_ar_[t_lane_u8].valid_b_)
    {
      per_lane_collection_cs_.per_point_ar_[t_lane_u8].valid_b_ =
          per_lane_collection_cs_.per_coeff_ar_[t_lane_u8].valid_b_;
      const PerLaneBoundaryCoeff& t_l_per_coeff_cs = per_lane_collection_cs_.per_coeff_ar_[t_lane_u8].left_coeff_cs_;
      const PerLaneBoundaryCoeff& t_r_per_coeff_cs = per_lane_collection_cs_.per_coeff_ar_[t_lane_u8].right_coeff_cs_;
      BoundaryPoint& t_l_points_cs = per_lane_collection_cs_.per_point_ar_[t_lane_u8].left_point_cs_;
      BoundaryPoint& t_r_points_cs = per_lane_collection_cs_.per_point_ar_[t_lane_u8].right_point_cs_;
      if (t_l_per_coeff_cs.valid_b_)
      {
        DiscretePerLaneBoundary(t_l_per_coeff_cs, t_l_points_cs);
      }
      if (t_r_per_coeff_cs.valid_b_)
      {
        DiscretePerLaneBoundary(t_r_per_coeff_cs, t_r_points_cs);
      }
    }
  }
}

void Preprocess::ObjHistTrajTrack(const PerObjCollection& obj_collection_cs,
                                  const bc::TCArray<TrafficAgentData, kFusMaxObjNum>& last_em_agents)
{
  /** 1. Update object trajectory using current cycle perception objects */
  bc::uint8_t t_valid_object_ctns_u8 = obj_traj_collection_cs_.GetValidObjectTractoryCtns();
  bc::TCArray<bc::uint8_t, kFusMaxObjNum> t_object_indices_ar = obj_traj_collection_cs_.GetObjectIndices();
  TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();

  /** 2. Only update stored history trajectories in Error Confirm status */
  if (input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
      input_diag_manager_ar_[kInputChkPerObj].status_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
    if (t_valid_object_ctns_u8 > 0)
    {
      // Update all stored object trajectory points in current coordinate system
      obj_traj_collection_cs_.DoCoordTransToTrajectory(t_transform_st.rotation_f_, t_transform_st.translation_dx_f_,
                                                       t_transform_st.translation_dy_f_);
    }
    return;
  }

  /** 3. Use current or predicted object data to do trajectory tracking in Normal or Error Warn status */
  // Clear untracking object trajectory
  for (bc::uint8_t idx = 0; idx < t_valid_object_ctns_u8; ++idx)
  {
    if (!obj_collection_cs.objs_ar_[t_object_indices_ar[idx]].is_valid)
    {
      obj_traj_collection_cs_.ResetTrajectoryByIdx(t_object_indices_ar[idx]);
    }
  }
  // Update all stored object trajectory points in current coordinate system
  obj_traj_collection_cs_.DoCoordTransToTrajectory(t_transform_st.rotation_f_, t_transform_st.translation_dx_f_,
                                                   t_transform_st.translation_dy_f_);
  // Only obejcts that assigned to host lane can be used
  // Only objects that in a given range and not on the opposite direction can be added
  for (bc::uint8_t obj_idx = 0; obj_idx < kFusMaxObjNum; ++obj_idx)
  {
    auto iter = std::find_if(last_em_agents.begin(), last_em_agents.end(),
                             [&obj_collection_cs, obj_idx](const TrafficAgentData& agent) -> bool {
                               return agent.id_ == obj_collection_cs.objs_ar_[obj_idx].id;
                             });
    if (iter != last_em_agents.end() && iter->idx_assigned_lane_ != kHostLane)
    {
      obj_traj_collection_cs_.SetTrajectoryInValidityByIdx(obj_idx);
      obj_traj_collection_cs_.ResetTrajectoryByIdx(obj_idx);
      continue;
    }
    // Only track agents match a valid condition
    if (obj_collection_cs.objs_ar_[obj_idx].is_valid &&
        obj_collection_cs.objs_ar_[obj_idx].category >= ObjCategoryType::kPassengerCar &&
        obj_collection_cs.objs_ar_[obj_idx].category < ObjCategoryType::kUncertainVehicle &&
        (obj_collection_cs.objs_ar_[obj_idx].motion_pattern == MotionPattern::kDriving ||
         obj_collection_cs.objs_ar_[obj_idx].motion_pattern == MotionPattern::kDrivingStopped) &&
        obj_collection_cs.objs_ar_[obj_idx].x > 0.f && obj_collection_cs.objs_ar_[obj_idx].x < kObjMaxLongPosition &&
        fabsf(obj_collection_cs.objs_ar_[obj_idx].y) < kObjMaxLatPosition)
    {
      Point2D t_point_cs(obj_collection_cs.objs_ar_[obj_idx].x, obj_collection_cs.objs_ar_[obj_idx].y);
      obj_traj_collection_cs_.AddPointToTrajectoryByIdx(obj_idx, t_point_cs, obj_collection_cs.objs_ar_[obj_idx].id);
    }
    else if (obj_collection_cs.objs_ar_[obj_idx].is_valid &&
             obj_traj_collection_cs_.GetTrajectoryValidityByIdx(obj_idx))
    {
      obj_traj_collection_cs_.ResetTrajectoryByIdx(obj_idx);
    }
  }
}

void Preprocess::LdVehLaneGen(const EmData& em_output_lst1_cs, const PreProParam& prep_param_st,
                              const bc::float32_t time_cycle_f)
{
  bc::uint8_t t_valid_object_ctns_u8 = obj_traj_collection_cs_.GetValidObjectTractoryCtns();
  bc::TCArray<bc::uint8_t, kFusMaxObjNum> t_object_indices_ar = obj_traj_collection_cs_.GetObjectIndices();
  /** 1. Choose LdVeh from obj_traj_collection_cs_*/
  bc::bool_t t_find_ld_veh_b = bc::false_v;
  bc::uint8_t t_ld_veh_idx_u8 = kInvalidObjIdx;
  bc::float32_t t_admission_heading_f = bc::G_DEG2RAD * prep_param_st.ldveh_admission_heading_f;  /// 4deg to rad
  bc::float32_t t_exit_heading_f = bc::G_DEG2RAD * prep_param_st.ldveh_exit_heading_f;            /// 6deg
  FusionObj lst_obj_fus;
  TrafficAgentData lst_obj_em;
  // Keep using original leading vehicle trajectory if it is still available and stable
  if (ld_veh_lst1_id_u16_ != kInvalidObjectId && ld_veh_lst1_idx_u8_ != kInvalidObjIdx &&
      obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].is_valid &&
      obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].id == ld_veh_lst1_id_u16_ &&
      em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].idx_assigned_lane_ == kHostLane &&
      em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].lane_association_[kHostLane].probability_ >
          prep_param_st.ldveh_exit_prob_f &&
      fabsf(obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].heading) < t_exit_heading_f &&
      em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].pos_.x < prep_param_st.ldveh_exit_dis_f &&
      fabsf(em_output_lst1_cs.lanes_[kHostLane]
                .lane_elements_[0]
                .reference_line_.agents_projected_traj_[ld_veh_lst1_idx_u8_]
                .slt_pt_.l_) < prep_param_st.ldveh_exit_offset_f)
  {
    lv_dbg_cs_.lst_veh_cs_.prob_restrict_f_ = prep_param_st.ldveh_exit_prob_f;
    lv_dbg_cs_.lst_veh_cs_.veh_prob_f_ =
        em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].lane_association_[kHostLane].probability_;
    lv_dbg_cs_.lst_veh_cs_.satisfy_prob_b_ =
        lv_dbg_cs_.lst_veh_cs_.veh_prob_f_ > lv_dbg_cs_.lst_veh_cs_.prob_restrict_f_;
    lv_dbg_cs_.lst_veh_cs_.heading_restrict_f_ = t_exit_heading_f;
    lv_dbg_cs_.lst_veh_cs_.veh_heading_f_ = fabsf(obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].heading);
    lv_dbg_cs_.lst_veh_cs_.satisfy_heading_b_ =
        lv_dbg_cs_.lst_veh_cs_.veh_heading_f_ < lv_dbg_cs_.lst_veh_cs_.heading_restrict_f_;
    lv_dbg_cs_.lst_veh_cs_.long_distance_restrict_f_ = prep_param_st.ldveh_exit_dis_f;
    lv_dbg_cs_.lst_veh_cs_.veh_long_distance_f_ = em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].pos_.x;
    lv_dbg_cs_.lst_veh_cs_.satisfy_long_distance_b_ =
        lv_dbg_cs_.lst_veh_cs_.veh_long_distance_f_ < lv_dbg_cs_.lst_veh_cs_.long_distance_restrict_f_;
    lv_dbg_cs_.lst_veh_cs_.offset_restrict_f_ = prep_param_st.ldveh_exit_offset_f;
    lv_dbg_cs_.lst_veh_cs_.veh_offset_f_ = fabsf(em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].pos_.y);
    lv_dbg_cs_.lst_veh_cs_.satisfy_offset_b_ =
        lv_dbg_cs_.lst_veh_cs_.veh_offset_f_ < lv_dbg_cs_.lst_veh_cs_.offset_restrict_f_;
    lv_dbg_cs_.lst_veh_cs_.veh_id_u8_ = obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].id;

    t_find_ld_veh_b = bc::true_v;
    t_ld_veh_idx_u8 = ld_veh_lst1_idx_u8_;
    lv_dbg_cs_.follow_lst_cycle_b_ = bc::true_v;
    lv_dbg_cs_.final_veh_id_u8_ = obj_collection_cs_.objs_ar_[ld_veh_lst1_idx_u8_].id;
    lv_dbg_cs_.lst_veh_cs_.follow_veh_b_ = bc::true_v;
  }
  // Select a most trustable leading vehicle
  // todo: based on Similarity between object trajectory and lane line.
  // else
  // {
  bc::float32_t t_ld_veh_prob_f = 0.f;
  bc::float32_t t_ld_veh_dis_f = prep_param_st.ldveh_admission_dis_f;
  bc::float32_t t_ld_veh_offset_f = prep_param_st.ldveh_admission_offset_f;
  if (t_find_ld_veh_b)
  {
    t_ld_veh_prob_f = em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].lane_association_[kHostLane].probability_;
    t_ld_veh_dis_f = std::min(em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].pos_.x - kLongDisHysteresis,
                              prep_param_st.ldveh_admission_dis_f);
    t_ld_veh_offset_f =
        std::min(fabsf(em_output_lst1_cs.agents_[ld_veh_lst1_idx_u8_].pos_.y), prep_param_st.ldveh_admission_offset_f);
  }
  else
  {
    t_ld_veh_prob_f = 0.f;
    t_ld_veh_dis_f = prep_param_st.ldveh_admission_dis_f;
    t_ld_veh_offset_f = prep_param_st.ldveh_admission_offset_f;
  }

  bc::float32_t t_max_potential_ld_prob_f = t_ld_veh_prob_f;
  for (bc::uint8_t idx = 0; idx < t_valid_object_ctns_u8; ++idx)
  {
    bc::uint8_t t_object_idx = t_object_indices_ar[idx];
    if (t_object_idx >= kInvalidObjIdx)
    {
#ifdef STD_COUT_ENABLE
      std::cout << "Error occurs in LdVehLaneGen!!!" << std::endl;
#endif
      continue;
    }
    lst_obj_fus = obj_collection_cs_.objs_ar_[t_object_idx];
    lst_obj_em = em_output_lst1_cs.agents_[t_object_idx];
    if (lst_obj_em.idx_assigned_lane_ == kHostLane && lst_obj_fus.is_valid && lst_obj_fus.id == lst_obj_em.id_ &&
        (lst_obj_em.lane_association_[kHostLane].probability_ >= prep_param_st.ldveh_admission_prob_f) &&
        lst_obj_em.pos_.x < t_ld_veh_dis_f && fabsf(lst_obj_fus.heading) < t_admission_heading_f &&
        obj_traj_collection_cs_.GetTrajectoryPointNumByIdx(t_object_idx) >= kMinValidTrajPointNum &&
        fabsf(em_output_lst1_cs.lanes_[kHostLane]
                  .lane_elements_[0]
                  .reference_line_.agents_projected_traj_[t_object_idx]
                  .slt_pt_.l_) < t_ld_veh_offset_f)
    {
      if ((lst_obj_em.lane_association_[kHostLane].probability_ >= t_ld_veh_prob_f ||
           lst_obj_em.lane_association_[kHostLane].probability_ >= 0.9) &&
          lst_obj_em.lane_association_[kHostLane].probability_ >= t_max_potential_ld_prob_f)
      {
        t_max_potential_ld_prob_f = lst_obj_em.lane_association_[kHostLane].probability_;
        t_find_ld_veh_b = bc::true_v;
        t_ld_veh_idx_u8 = t_object_idx;
        lv_dbg_cs_.follow_change_b_ = bc::true_v;
        lv_dbg_cs_.final_veh_id_u8_ = obj_collection_cs_.objs_ar_[t_object_idx].id;
        // t_fol_veh_dbg.follow_veh_b_ = bc::true_v;
      }

      if (obj_traj_collection_cs_.GetTargetTrajectoryByIdx(t_object_idx).GetIsChoosedBool())
      {
      }
      else
      {
        obj_traj_collection_cs_.GetTargetTrajectoryByIdx(t_object_idx).SetIsChoosedBool(bc::true_v);
      }
    }
    if (obj_traj_collection_cs_.GetTargetTrajectoryByIdx(t_object_idx).GetIsChoosedBool() &&
        (lst_obj_em.idx_assigned_lane_ != kHostLane || !lst_obj_fus.is_valid ||
         lst_obj_em.lane_association_[kHostLane].probability_ < prep_param_st.ldveh_exit_prob_f ||
         lst_obj_em.pos_.x > prep_param_st.ldveh_exit_dis_f ||
         fabsf(lst_obj_fus.heading) > prep_param_st.ldveh_exit_heading_f ||
         fabsf(em_output_lst1_cs.lanes_[kHostLane]
                   .lane_elements_[0]
                   .reference_line_.agents_projected_traj_[t_object_idx]
                   .slt_pt_.l_) > prep_param_st.ldveh_exit_offset_f))
    {
      obj_traj_collection_cs_.GetTargetTrajectoryByIdx(t_object_idx).SetIsChoosedBool(bc::false_v);
      obj_traj_collection_cs_.GetTargetTrajectoryByIdx(t_object_idx).ResetAfterChoosedTotalPointNum();
    }
  }
  if (t_find_ld_veh_b)
  {
    ld_veh_lst1_id_u16_ = obj_collection_cs_.objs_ar_[t_ld_veh_idx_u8].id;
    ld_veh_lst1_idx_u8_ = t_ld_veh_idx_u8;
  }
  else
  {
    ld_veh_lst1_id_u16_ = kInvalidObjectId;
    ld_veh_lst1_idx_u8_ = kInvalidObjIdx;
  }
  // }
  // || obj_traj_collection_cs_.GetTrajectoryAfterChoosedPointNumByIdx(t_ld_veh_idx_u8) < 3
  /** 2. Consider the case that LdVeh trajectory is not available */
  if (!t_find_ld_veh_b)
  {
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].valid_b_ = bc::false_v;
    return;
  }

  /** 3. Align the trajectory of LdVeh to x_ref */
  // Calculate left and right boundary points of LdVeh with a reliable current lane width
  ReferenceLine t_host_ref_cs = em_output_lst1_cs.lanes_[kHostLane].lane_elements_[0].reference_line_;
  bc::float32_t t_lane_width_lst1_f = t_host_ref_cs.ref_line_pts_[t_host_ref_cs.current_point_idx_].lane_width_;
  bc::float32_t t_lane_width_f =
      LowPass(kDefaultLdVehLaneWidth, t_lane_width_lst1_f, kLaneWidthTimeFactor, time_cycle_f);
  TrajectoryBoundary t_traj_boundary_st =
      obj_traj_collection_cs_.GetTargetVehicleLaneBoundary(t_ld_veh_idx_u8, t_lane_width_f);
  bc::uint16_t t_traj_boundary_valid_point_ctns_u16 = t_traj_boundary_st.valid_point_ctns_u8_;

  // Align left and right boundary dys given x_ref_ar_ and find valid start/end index
  bc::uint8_t t_left_boundary_start_idx_u8 = kMaxNumBoundary;
  bc::uint8_t t_left_boundary_end_idx_u8 = kMaxNumBoundary - 1;  // todo: use crrent veh postion
  bc::uint8_t t_right_boundary_start_idx_u8 = kMaxNumBoundary;
  bc::uint8_t t_right_boundary_end_idx_u8 = kMaxNumBoundary - 1;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_left_y_sync_ar =
      BoundaryPointAlignment<bc::float32_t, kMaxNumBoundary, kMaxTrajectoryPoint>(
          x_ref_ar_, t_traj_boundary_st.left_x_ar_, t_traj_boundary_st.left_y_ar_, t_traj_boundary_valid_point_ctns_u16,
          kMaxNumBoundary, t_left_boundary_start_idx_u8, t_left_boundary_end_idx_u8);
  bc::TCArray<bc::float32_t, kMaxNumBoundary> t_right_y_sync_ar =
      BoundaryPointAlignment<bc::float32_t, kMaxNumBoundary, kMaxTrajectoryPoint>(
          x_ref_ar_, t_traj_boundary_st.right_x_ar_, t_traj_boundary_st.right_y_ar_,
          t_traj_boundary_valid_point_ctns_u16, kMaxNumBoundary, t_right_boundary_start_idx_u8,
          t_right_boundary_end_idx_u8);
  ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_end_idx_u8_ =
      ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_end_idx_u8_ =
          std::min(t_left_boundary_end_idx_u8, t_right_boundary_end_idx_u8);
  ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_start_idx_u8_ =
      ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_start_idx_u8_ =
          std::max(t_left_boundary_start_idx_u8, t_right_boundary_start_idx_u8);
  if (t_left_boundary_start_idx_u8 == kMaxNumBoundary || t_right_boundary_start_idx_u8 == kMaxNumBoundary ||
      (ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_end_idx_u8_ <=
       ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_start_idx_u8_) ||
      ((ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_end_idx_u8_ -
        ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_start_idx_u8_) < 5))
  {
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.valid_b_ = bc::false_v;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.valid_b_ = bc::false_v;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].valid_b_ = bc::false_v;
    ld_veh_lst1_id_u16_ = kInvalidObjectId;
    ld_veh_lst1_idx_u8_ = kInvalidObjIdx;
  }
  else
  {
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.valid_b_ = bc::true_v;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.valid_b_ = bc::true_v;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.per_line_id_s32_ =
        obj_collection_cs_.objs_ar_[t_ld_veh_idx_u8].id;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.per_line_id_s32_ =
        obj_collection_cs_.objs_ar_[t_ld_veh_idx_u8].id;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].valid_b_ = bc::true_v;
    ldveh_lane_collection_cs_.veh_id_u16_ = obj_collection_cs_.objs_ar_[t_ld_veh_idx_u8].id;
#ifdef STD_COUT_ENABLE
    std::cout << "LedVeh Id:  " << (bc::uint16_t)ldveh_lane_collection_cs_.veh_id_u16_ << std::endl;
#endif
    // Fill in ldveh_lane_collection_cs_ output
    for (bc::uint8_t i = 0; i < kMaxNumBoundary; ++i)
    {
      ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_ar_[i] = t_left_y_sync_ar[i];
      ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_ar_[i] = t_right_y_sync_ar[i];
    }
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dx_end_f_ =
        x_ref_ar_[ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_end_idx_u8_];
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dx_start_f_ =
        x_ref_ar_[ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_start_idx_u8_];
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dx_end_f_ =
        x_ref_ar_[ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_end_idx_u8_];
    ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dx_start_f_ =
        x_ref_ar_[ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_start_idx_u8_];
  }
}

void Preprocess::DynamicAdaptiveCoordinate(bc::float32_t& dx_f, bc::float32_t vel_bm_f)
{
  /** 0. Currently consider ego velocity. */
  bc::float32_t t_ego_vel_f64 = ego_pose_collection_cs_.GetCurrentEgoPose().GetLongVelocity();
  bc::float32_t t_scale_f = t_ego_vel_f64 / vel_bm_f;  ///< For scaling step.
  t_scale_f = ZONE_MIN(ZONE_MAX(0.5, t_scale_f), 1.5f);
  dx_f *= t_scale_f;
  /** 1. Also can consider scenarios later. */
  // todo:
}

void Preprocess::PerObjTrack(const bc::float32_t dt_f, FusionObj& obj_cs)
{
  /** Timesync Perception object */
  if (obj_cs.is_valid)
  {
    /** 0.Translation.*/
    const EgoPose& t_ego_pose_cs = ego_pose_collection_cs_.GetCurrentEgoPose();
    bc::float32_t t_ego_velx_f = ego_pose_collection_cs_.GetCurrentEgoPose().GetLongVelocity();
    bc::float32_t t_ego_vely_f = ego_pose_collection_cs_.GetCurrentEgoPose().GetLatVelocity();
    TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();
    bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
    bc::float32_t t_obj_rotation_f = 0.f;
    bc::float32_t t_obj_translation_x_f = 0.f;
    bc::float32_t t_obj_translation_y_f = 0.f;
    if (obj_cs.motion_pattern != MotionPattern::kUnknown && obj_cs.motion_pattern != MotionPattern::kStationary)
    {
      /** Predict object position based on old coordinate system.*/
      t_obj_rotation_f = obj_cs.yawrate * dt_f;
      t_obj_translation_x_f = obj_cs.long_velocity_abs * dt_f + 0.5 * obj_cs.long_acceleration_abs * dt_f * dt_f;
      t_obj_translation_y_f = obj_cs.lat_velocity_abs * dt_f + 0.5 * obj_cs.lat_acceleration_abs * dt_f * dt_f;
    }
    /** Coordinate transform based on new ego pose and new coordinate system.*/
    bc::float32_t t_obj_x = obj_cs.x + t_obj_translation_x_f;
    bc::float32_t t_obj_y = obj_cs.y + t_obj_translation_y_f;
    bc::float32_t t_obj_long_vel_abs_f = obj_cs.long_velocity_abs;
    bc::float32_t t_obj_lat_vel_abs_f = obj_cs.lat_velocity_abs;
    t_obj_x -= t_transform_st.translation_dx_f_;
    t_obj_y -= t_transform_st.translation_dy_f_;
    obj_cs.x = cosf(t_rotation_f) * t_obj_x + sinf(t_rotation_f) * t_obj_y;
    obj_cs.y = -sinf(t_rotation_f) * t_obj_x + cosf(t_rotation_f) * t_obj_y;
    if (std::isnan(obj_cs.x) || std::isnan(obj_cs.y))
    {
#ifdef STD_COUT_ENABLE
      std::cout << "Nan in Environment Model PerObjTrack!!!" << std::endl;
#endif
    }
    obj_cs.heading += t_obj_rotation_f - t_rotation_f;
    obj_cs.long_velocity_abs = (t_obj_long_vel_abs_f + obj_cs.long_acceleration_abs * dt_f) * cosf(t_rotation_f) +
                               (t_obj_lat_vel_abs_f + obj_cs.lat_acceleration_abs * dt_f) * sinf(t_rotation_f);
    obj_cs.lat_velocity_abs = -(t_obj_long_vel_abs_f + obj_cs.long_acceleration_abs * dt_f) * sinf(t_rotation_f) +
                              (t_obj_lat_vel_abs_f + obj_cs.lat_acceleration_abs * dt_f) * cosf(t_rotation_f);
    obj_cs.long_velocity_relative = obj_cs.long_velocity_abs - t_ego_velx_f;
    obj_cs.lat_velocity_relative = obj_cs.lat_velocity_abs - t_ego_vely_f;

    /** 1.Change Source.*/
    obj_cs.fusion_source = FusionSource::kEmInternalTracked;
  }
  else
  {
    /**Just keep status,but change their source type.*/
    // obj_cs.fusion_source = FusionSource::kEmInternalTracked;
  }
}

void Preprocess::StorePerObjsToInternal(const ObjTracks& per_obj_cs)
{
  // obj_collection_cs_.time_stamp_f_ = per_obj_cs.timeStamp;
  for (bc::uint8_t t_obj_u8 = 0; t_obj_u8 < kFusMaxObjNum; ++t_obj_u8)
  {
    /** Check object validity.*/
    if (per_obj_cs.objects[t_obj_u8].is_valid && per_obj_cs.objects[t_obj_u8].state.track_id != kInvalidObjectId &&
        per_obj_cs.objects[t_obj_u8].state.track_id < kMaxObjectId)
    {
      /** External data.*/
      const ObjTrackFusedState& t_obj_estimate_cs = per_obj_cs.objects[t_obj_u8].state;
      const ObjTrackFusedQuality& t_obj_property_cs = per_obj_cs.objects[t_obj_u8].quality;
      const ObjTrackNaturalAttr& t_obj_information_cs = per_obj_cs.objects[t_obj_u8].attr;
      const zone::fusion::ObjTrackCornerPoint& t_obj_corner_point_cs = per_obj_cs.objects[t_obj_u8].corner_point;
      const bc::bool_t t_obj_corner_point_valid_b = per_obj_cs.objects[t_obj_u8].corner_point_is_valid;
      /** Internal data.*/
      FusionObj& t_obj_cs = obj_collection_cs_.objs_ar_[t_obj_u8];
      /** Store.*/
      t_obj_cs.id = t_obj_estimate_cs.track_id;
      t_obj_cs.length = t_obj_information_cs.length;
      t_obj_cs.width = t_obj_information_cs.width;
      t_obj_cs.height = t_obj_information_cs.height;
      t_obj_cs.is_cipv = t_obj_information_cs.is_cipv;
      // t_obj_cs.distance_to_left_line = t_obj_information_cs.dist_to_left_near_lane;
      // t_obj_cs.distance_to_right_line = t_obj_information_cs.dist_to_right_near_lane;

      switch (t_obj_information_cs.obj_type)
      {
        case 0:
          t_obj_cs.category = ObjCategoryType::kUnknown;
          break;
        case 1:
          t_obj_cs.category = ObjCategoryType::kPedestrian;
          break;
        case 2:
          t_obj_cs.category = ObjCategoryType::kCyclist;
          break;
        case 3:
          t_obj_cs.category = ObjCategoryType::kPassengerCar;
          break;
        case 4:
          t_obj_cs.category = ObjCategoryType::kSmallBus;
          break;
        case 5:
          t_obj_cs.category = ObjCategoryType::kBigBus;
          break;
        case 6:
          t_obj_cs.category = ObjCategoryType::kLightTruck;
          break;
        case 7:
          t_obj_cs.category = ObjCategoryType::kHeavy_Truck;
          break;
        case 8:
          t_obj_cs.category = ObjCategoryType::kTricycle;
          break;
        case 9:
          t_obj_cs.category = ObjCategoryType::kVan;
          break;
        case 10:
          t_obj_cs.category = ObjCategoryType::kUncertainVehicle;
          break;
        case 11:
          t_obj_cs.category = ObjCategoryType::kTrafficCone;
          break;
        case 12:
          t_obj_cs.category = ObjCategoryType::kPole;
          break;
        default:
          break;
      }

      switch (t_obj_property_cs.fusion_source)
      {
        case 0:
          t_obj_cs.fusion_source = FusionSource::kUnknown;
          break;
        case 1:
          t_obj_cs.fusion_source = FusionSource::kVisionOnly;
          break;
        case 2:
          t_obj_cs.fusion_source = FusionSource::kRadarOnly;
          break;
        case 3:
          t_obj_cs.fusion_source = FusionSource::kFusedRadarVision;
          break;
        default:
          break;
      }

      switch (t_obj_information_cs.motion_pattern_current)
      {
        case 0:
          t_obj_cs.motion_pattern = MotionPattern::kUnknown;
          break;
        case 1:
          t_obj_cs.motion_pattern = MotionPattern::kStationary;
          break;
        case 2:
          t_obj_cs.motion_pattern = MotionPattern::kDriving;
          break;
        case 3:
          t_obj_cs.motion_pattern = MotionPattern::kOncoming;
          break;
        case 4:
          t_obj_cs.motion_pattern = MotionPattern::kDrivingStopped;
          break;
        case 5:
          t_obj_cs.motion_pattern = MotionPattern::kOncomingStopped;
          break;
        case 6:
          t_obj_cs.motion_pattern = MotionPattern::kCrossing;
          break;
        default:
          break;
      }

      switch (t_obj_information_cs.brake_light_status)
      {
        case 0:
          t_obj_cs.brake_light_status = BrakeLightStatus::kUnknown;
          break;
        case 1:
          t_obj_cs.brake_light_status = BrakeLightStatus::kOn;
          break;
        case 2:
          t_obj_cs.brake_light_status = BrakeLightStatus::kOff;
          break;
        default:
          break;
      }

      switch (t_obj_information_cs.turn_light_status)
      {
        case 0:
          t_obj_cs.turn_light_status = TurnLightStatus::kUnknown;
          break;
        case 1:
          t_obj_cs.turn_light_status = TurnLightStatus::kLeft;
          break;
        case 2:
          t_obj_cs.turn_light_status = TurnLightStatus::kRight;
          break;
        case 3:
          t_obj_cs.turn_light_status = TurnLightStatus::kBoth;
          break;
        case 4:
          t_obj_cs.turn_light_status = TurnLightStatus::kOff;
          break;
        default:
          break;
      }

      t_obj_cs.x = t_obj_estimate_cs.long_position;
      t_obj_cs.y = t_obj_estimate_cs.lat_position;
      if (std::isnan(t_obj_estimate_cs.long_position) || std::isnan(t_obj_estimate_cs.lat_position))
      {
#ifdef STD_COUT_ENABLE
        std::cout << "EM: Per Obj input has nan!! " << t_obj_estimate_cs.long_position << t_obj_estimate_cs.lat_position
                  << std::endl;
#endif
      }
      t_obj_cs.heading = t_obj_estimate_cs.heading_angle;
      t_obj_cs.yaw = t_obj_estimate_cs.yaw;
      t_obj_cs.yawrate = t_obj_estimate_cs.yaw_rate;
      t_obj_cs.long_velocity_abs = t_obj_estimate_cs.long_vel_abs;
      t_obj_cs.lat_velocity_abs = t_obj_estimate_cs.lat_vel_abs;
      t_obj_cs.long_velocity_relative = t_obj_estimate_cs.long_vel_relative;
      t_obj_cs.lat_velocity_relative = t_obj_estimate_cs.lat_vel_relative;
      t_obj_cs.long_acceleration_abs = t_obj_estimate_cs.long_acc_abs;
      t_obj_cs.lat_acceleration_abs = t_obj_estimate_cs.lat_acc_abs;
      t_obj_cs.long_acceleration_relative = t_obj_estimate_cs.long_acc_relative;
      t_obj_cs.lat_acceleration_relative = t_obj_estimate_cs.lat_acc_relative;
      t_obj_cs.long_position_std_dev = t_obj_property_cs.long_position_std_dev;
      t_obj_cs.lat_position_std_dev = t_obj_property_cs.lat_position_std_dev;
      t_obj_cs.is_valid = bc::true_v;

      /* corner point infor */

      t_obj_cs.corner_point_cs_.obj_cut_in_flag_ = t_obj_corner_point_cs.obj_cut_in_flag;
      t_obj_cs.corner_point_cs_.obj_cut_in_lane_ = t_obj_corner_point_cs.obj_cut_in_lane;
      t_obj_cs.corner_point_cs_.obj_corner_point_x_ = t_obj_corner_point_cs.obj_corner_point_x;
      t_obj_cs.corner_point_cs_.obj_corner_point_y_ = t_obj_corner_point_cs.obj_corner_point_y;
      t_obj_cs.corner_point_cs_.obj_dist_in_lane_ = t_obj_corner_point_cs.obj_dist_in_lane;
      t_obj_cs.corner_point_cs_.cut_in_speed_ = t_obj_corner_point_cs.cut_in_speed;
      t_obj_cs.corner_point_cs_.obj_yaw_relative_ = t_obj_corner_point_cs.obj_yaw_relative;
      t_obj_cs.corner_point_cs_.obj_yawrate_relative_ = t_obj_corner_point_cs.obj_yawrate_relative;
      t_obj_cs.corner_point_cs_.obj_yaw_info_state_ = t_obj_corner_point_cs.obj_yaw_info_state;
      t_obj_cs.corner_point_cs_.cut_in_speed_valid_ = t_obj_corner_point_cs.cut_in_speed_valid;
      t_obj_cs.corner_point_cs_.corner_point_valid_ = t_obj_corner_point_valid_b;
    }
  }
  for (bc::uint8_t t_cipv_id_idx_u8 = 0; t_cipv_id_idx_u8 < kMaxFusionCipvNum; ++t_cipv_id_idx_u8)
  {
    (*em_collection_ptr_).cipv_id_[t_cipv_id_idx_u8] = per_obj_cs.cipv_ids[t_cipv_id_idx_u8];
  }
}

void Preprocess::StorePerLineToInternal(const LaneMarking& external_cs, const bc::int8_t target_lane_u8,
                                        const bc::uint8_t target_element_u8, const bc::int8_t target_boundary_u8)
{
  if (target_lane_u8 < kLLLane || target_lane_u8 > kRRLane)
  {
    return;
  }
  if (target_boundary_u8 != kLeftBoundary && target_boundary_u8 != kRightBoundary)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Target Boundary Error in EM StorePerLineToInternal !!! " << target_boundary_u8 << std::endl;
#endif
    return;
  }
  /** Lane idx include 0,1,2,3,4; boundary idx include left 0 ,right 1.*/
  per_lane_collection_cs_.per_coeff_ar_[target_lane_u8].valid_b_ = bc::true_v;
  PerLaneBoundaryCoeff* t_internal_ptr = NULL;

  if (target_boundary_u8 == kLeftBoundary)
  {
    t_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[target_lane_u8].left_coeff_cs_);
  }
  else if (target_boundary_u8 == kRightBoundary)
  {
    t_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[target_lane_u8].right_coeff_cs_);
  }

  t_internal_ptr->coeff_cs_.c0_position = external_cs.lines_3d[0].y_coeffs[0];
  t_internal_ptr->coeff_cs_.c1_heading_angle = external_cs.lines_3d[0].y_coeffs[1];
  t_internal_ptr->coeff_cs_.c2_curvature = external_cs.lines_3d[0].y_coeffs[2];
  t_internal_ptr->coeff_cs_.c3_curvature_derivative = external_cs.lines_3d[0].y_coeffs[3];

  t_internal_ptr->dx_end_f_ = external_cs.lines_3d[0].start_pt.x + external_cs.lines_3d[0].t_max;
  t_internal_ptr->dx_start_f_ = external_cs.lines_3d[0].start_pt.x;
  // t_internal_ptr->std_c0_f_ = external_cs.lines_3d[0].latDistanceZeroOrderCoeffVariance;
  // t_internal_ptr->std_c1_f_ = external_cs.lines_3d[0].latDistanceFirstOrderCoeffVariance;
  // t_internal_ptr->std_c2_f_ = external_cs.lines_3d[0].latDistanceSecondOrderCoeffVariance;
  // t_internal_ptr->std_c3_f_ = external_cs.lines_3d[0].latDistanceThirdOrderCoeffVariance;
  // t_internal_ptr->prob_exist_f_ = external_cs.lines_3d[0].existConf;
  t_internal_ptr->prob_exist_f_ = external_cs.confidence;
  t_internal_ptr->per_line_id_s32_ = external_cs.line_id;
  if (!(external_cs.line_source >> 26) & 1)
  {
    t_internal_ptr->is_paralleled_b = bc::true_v;
  }
  t_internal_ptr->per_line_id_s32_ = external_cs.line_id;

  t_internal_ptr->valid_b_ = bc::true_v;

  /** Boundary type adaption */
  t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kUnknown;
  switch (external_cs.lines_3d[0].line_marking)
  {
    case 0:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kUnknown;
      break;
    case 1:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kSolid;
      break;
    case 2:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kDash;
      break;
    case 3:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kShortDash;
      break;
    case 4:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kDoubleSolid;
      break;
    case 5:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kDoubleDash;
      break;
    case 6:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kLeftSolidRightDash;
      break;
    case 7:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kRightSolidLeftDash;
      break;
    case 8:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kShadedArea;
      break;
    case 9:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kLaneVirtualMarking;
      break;
    case 10:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::KIntersectionVirutalMarking;
      break;
    case 11:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kCurbVirtualMarking;
      break;
    case 12:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kUnClosedRoad;
      break;
    case 13:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kRoadVirtualLine;
      break;
    case 14:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kDecelerationSolidLine;
      break;
    case 15:
      t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kDecelerationDashedLine;
      break;
    default:
      break;
  }
  if (external_cs.line_type == 64U)
  {
    t_internal_ptr->boundary_type_en_ = LaneBoundaryType::kPhysical;
  }
  /** Boundary color adaption */
  t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kUnknown;
  switch (external_cs.lines_3d[0].line_color)
  {
    case 0:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kUnknown;
      break;
    case 1:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kWhite;
      break;
    case 2:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kYellow;
      break;
    case 3:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kOrange;
      break;
    case 4:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kBlue;
      break;
    case 5:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kGreen;
      break;
    case 6:
      t_internal_ptr->boundary_color_en_ = LaneBoundaryColor::kGray;
      break;
    default:
      break;
  }
  t_internal_ptr->life_time_f_ = 0.f;
  for (uint8_t i = 0; i < kFusionLinesNum; ++i)
  {
    if (lst_per_life_time_ar_[i].line_id_s32_ == external_cs.line_id)
    {
      t_internal_ptr->life_time_f_ = lst_per_life_time_ar_[i].used_life_time_f_;
      break;
    }
  }
}

void Preprocess::DiscretePerLaneBoundary(const PerLaneBoundaryCoeff& per_coeff_cs, BoundaryPoint& per_points_cs)
{
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxNumBoundary; ++t_idx_u8)
  {
    per_points_cs.dy_ar_[t_idx_u8] = per_coeff_cs.coeff_cs_.GetDyFromCloModel(x_ref_ar_[t_idx_u8]);
    if (std::isnan(x_ref_ar_[t_idx_u8]) || std::isnan(per_points_cs.dy_ar_[t_idx_u8]))
    {
#ifdef STD_COUT_ENABLE
      std::cout << " DiscretePerLaneBoundary Nan in Environment Model TrackBoundaryPoints!!!" << std::endl;
#endif
    }
    if (x_ref_ar_[t_idx_u8] < per_coeff_cs.dx_start_f_ || x_ref_ar_[t_idx_u8] > per_coeff_cs.dx_end_f_)
    {
      /** Beyond line range, also valid.*/
    }
    if ((x_ref_ar_[t_idx_u8] < per_coeff_cs.dx_start_f_) ||
        fabsf(x_ref_ar_[t_idx_u8] - per_coeff_cs.dx_start_f_) < kDistanceTolerance)
    {
      per_points_cs.dy_start_idx_u8_ = t_idx_u8;
    }
    if ((x_ref_ar_[t_idx_u8] < per_coeff_cs.dx_end_f_) ||
        fabsf(x_ref_ar_[t_idx_u8] - per_coeff_cs.dx_end_f_) < kDistanceTolerance)
    {
      per_points_cs.dy_end_idx_u8_ = t_idx_u8;
    }
  }
  per_points_cs.boundary_type_en_ = per_coeff_cs.boundary_type_en_;
  per_points_cs.boundary_color_en_ = per_coeff_cs.boundary_color_en_;
  per_points_cs.dx_end_f_ = per_coeff_cs.dx_end_f_;
  per_points_cs.dx_start_f_ = per_coeff_cs.dx_start_f_;
  per_points_cs.per_line_id_s32_ = per_coeff_cs.per_line_id_s32_;
  per_points_cs.per_life_time_ = per_coeff_cs.life_time_f_;
  per_points_cs.valid_b_ = per_coeff_cs.valid_b_;
}

void Preprocess::TrackBoundaryPoints(bc::TCArray<Point3D, kMaxBoundaryPoint>& points_ar, Point2D& end_point,
                                     Point2D& start_point)
{
  TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::TCArray<bc::float32_t, 4> rot_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f), -sinf(t_rotation_f),
                                                 cosf(t_rotation_f)};
  /** Track boundary points with EgoPose.*/

  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
  {
    if (std::isnan(points_ar[t_idx_u8].x) || std::isnan(points_ar[t_idx_u8].y))
    {
#ifdef STD_COUT_ENABLE
      std::cout << "Input Nan in Environment Model TrackBoundaryPoints!!!" << std::endl;
#endif
    }
    bc::float32_t t_x_f = points_ar[t_idx_u8].x;
    bc::float32_t t_y_f = points_ar[t_idx_u8].y;
    t_x_f -= t_transform_st.translation_dx_f_;
    t_y_f -= t_transform_st.translation_dy_f_;
    points_ar[t_idx_u8].x = t_x_f * rot_matrix_ar[0] + t_y_f * rot_matrix_ar[1];
    points_ar[t_idx_u8].y = t_x_f * rot_matrix_ar[2] + t_y_f * rot_matrix_ar[3];
    if (std::isnan(points_ar[t_idx_u8].x) || std::isnan(points_ar[t_idx_u8].y))
    {
#ifdef STD_COUT_ENABLE
      std::cout << "Nan in Environment Model TrackBoundaryPoints!!!" << std::endl;
#endif
    }
  }
  bc::float32_t t_end_x_f = end_point.x - t_transform_st.translation_dx_f_;
  bc::float32_t t_end_y_f = end_point.y - t_transform_st.translation_dy_f_;
  end_point.x = t_end_x_f * rot_matrix_ar[0] + t_end_y_f * rot_matrix_ar[1];
  end_point.y = t_end_x_f * rot_matrix_ar[2] + t_end_y_f * rot_matrix_ar[3];
  bc::float32_t t_st_x_f = start_point.x - t_transform_st.translation_dx_f_;
  bc::float32_t t_st_y_f = start_point.y - t_transform_st.translation_dy_f_;
  start_point.x = t_st_x_f * rot_matrix_ar[0] + t_st_y_f * rot_matrix_ar[1];
  start_point.y = t_st_x_f * rot_matrix_ar[2] + t_st_y_f * rot_matrix_ar[3];
}

void Preprocess::TrackLstCrossPoint(JointPointData& joint_point)
{
  TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::TCArray<bc::float32_t, 4> rot_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f), -sinf(t_rotation_f),
                                                 cosf(t_rotation_f)};
  /** Track boundary points with EgoPose.*/

  bc::float32_t t_cross_x_f = joint_point.lst_cross_point_.x - t_transform_st.translation_dx_f_;
  bc::float32_t t_cross_y_f = joint_point.lst_cross_point_.y - t_transform_st.translation_dy_f_;
  joint_point.lst_cross_point_.x = t_cross_x_f * rot_matrix_ar[0] + t_cross_y_f * rot_matrix_ar[1];
  joint_point.lst_cross_point_.y = t_cross_x_f * rot_matrix_ar[2] + t_cross_y_f * rot_matrix_ar[3];
  joint_point.lst_cross_idx_ = FindXRefStartIdx(joint_point.lst_cross_point_.x);
  if (joint_point.lst_cross_point_.x < 0)
  {
    joint_point.still_exist_b_ = bc::false_v;
  }
  else
  {
    joint_point.still_exist_b_ = bc::true_v;
  }
}

void Preprocess::TrackSegsPoint(const LaneBoundarySegment& segment, float32_t& start_dx_f, float32_t& end_dx_f)
{
  TransForm t_transform_st = ego_pose_collection_cs_.GetCurrentEgoPose().GetTransformFromLastCycle();
  bc::float32_t t_rotation_f = t_transform_st.rotation_f_;
  bc::TCArray<bc::float32_t, 4> rot_matrix_ar = {cosf(t_rotation_f), sinf(t_rotation_f), -sinf(t_rotation_f),
                                                 cosf(t_rotation_f)};
  TrackPointDx(rot_matrix_ar, t_transform_st.translation_dx_f_, t_transform_st.translation_dy_f_, segment.start_point_,
               start_dx_f);
  TrackPointDx(rot_matrix_ar, t_transform_st.translation_dx_f_, t_transform_st.translation_dy_f_, segment.end_point_,
               end_dx_f);
}

void Preprocess::TrackPointDx(const bc::TCArray<bc::float32_t, 4> rot_matrix_ar, const bc::float32_t translation_dx_f_,
                              const float32_t translation_dy_f_, const Point2D& track_point, bc::float32_t& track_dx)
{
  bc::float32_t t_track_x_f = track_point.x - translation_dx_f_;
  bc::float32_t t_track_y_f = track_point.y - translation_dy_f_;
  track_dx = t_track_x_f * rot_matrix_ar[0] + t_track_y_f * rot_matrix_ar[1];
}
bc::bool_t Preprocess::StoreTrackBoundaryPoints(const bc::TCArray<Point3D, kMaxBoundaryPoint>& points_ar,
                                                BoundaryPoint& track_points_cs, const bc::uint8_t input_valid_num,
                                                const bc::uint8_t input_st_idx_u8,
                                                const bc::uint8_t ref_output_st_idx_u8,
                                                const bc::uint8_t ref_output_end_idx_u8, const bc::bool_t ref_source_b)
{
  /** Align to x_ref_ar and x_ref_rear_ar with linear polation and store.*/
  if (input_valid_num < 2U)
  {
    return bc::false_v;
  }
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_em_x_ar;
  bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_em_y_ar;
  for (bc::int8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
  {
    t_em_x_ar[t_idx_u8] = 0;
    t_em_y_ar[t_idx_u8] = 0;
  }
  bc::uint16_t t_max_em_point_u16 = input_valid_num;
  bc::uint8_t t_start_idx_u8 = kMaxBoundaryPoint;
  bc::uint8_t t_end_idx_u8 = kMaxBoundaryPoint;
  for (bc::int8_t t_idx_u8 = input_st_idx_u8; t_idx_u8 < input_st_idx_u8 + input_valid_num; ++t_idx_u8)
  {
    t_em_x_ar[t_idx_u8 - input_st_idx_u8] = points_ar[t_idx_u8].x;
    t_em_y_ar[t_idx_u8 - input_st_idx_u8] = points_ar[t_idx_u8].y;
  }
  track_points_cs.dy_ar_ = BoundaryPointAlignment<bc::float32_t, kMaxNumBoundary, kMaxBoundaryPoint>(
      x_ref_ar_, t_em_x_ar, t_em_y_ar, t_max_em_point_u16, kMaxNumBoundary, t_start_idx_u8, t_end_idx_u8);
  if (t_start_idx_u8 < kMaxNumBoundary && t_end_idx_u8 < kMaxNumBoundary)
  {
    track_points_cs.dy_start_idx_u8_ = ref_source_b ? ref_output_st_idx_u8 : t_start_idx_u8;
    track_points_cs.dy_end_idx_u8_ = ref_source_b ? ref_output_end_idx_u8 : t_end_idx_u8;
    // if (t_end_idx_u8 - t_start_idx_u8 + 1 != map_xref_valid_num)
    // {
    //   std::cout << "Error in StoreTrackBoundaryPoints!!!" << std::endl;
    // }
    // track_points_cs.dx_start_f_ = x_ref_ar_[track_points_cs.dy_start_idx_u8_];
    // track_points_cs.dx_end_f_ = x_ref_ar_[track_points_cs.dy_end_idx_u8_];
    track_points_cs.total_valid_num_u8_ = track_points_cs.dy_end_idx_u8_ - track_points_cs.dy_start_idx_u8_ + 1;
    return bc::true_v;
  }
  else
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Error ocurrs in EM StoreTrackBoundaryPoints" << std::endl;
#endif
    return bc::false_v;
  }
}

void Preprocess::IndividualBoundaryRepack(BoundaryPointPack& boundary_cs, bc::uint8_t lane_idx_u8,
                                          bc::uint8_t ele_idx_u8, bc::uint8_t l_or_r_u8, bc::bool_t near_intersection_b)
{
  /** 0. Repack perception lane.*/
  if (kMainLaneElement == ele_idx_u8 && per_lane_collection_cs_.per_point_ar_[lane_idx_u8].valid_b_ &&
      !near_intersection_b)
  {
    BoundaryPoint t_per_boundary_point_cs;
    PerLaneBoundaryCoeff t_per_boundary_coeff_cs;
    if (l_or_r_u8 == kLeftBoundary)
    {
      t_per_boundary_point_cs = per_lane_collection_cs_.per_point_ar_[lane_idx_u8].left_point_cs_;
      t_per_boundary_coeff_cs = per_lane_collection_cs_.per_coeff_ar_[lane_idx_u8].left_coeff_cs_;
    }
    if (l_or_r_u8 == kRightBoundary)
    {
      t_per_boundary_point_cs = per_lane_collection_cs_.per_point_ar_[lane_idx_u8].right_point_cs_;
      t_per_boundary_coeff_cs = per_lane_collection_cs_.per_coeff_ar_[lane_idx_u8].right_coeff_cs_;
    }

    if (t_per_boundary_point_cs.valid_b_ &&
        t_per_boundary_point_cs.dy_start_idx_u8_ < t_per_boundary_point_cs.dy_end_idx_u8_)
    {
      boundary_cs.per_cs_ = t_per_boundary_point_cs;
      boundary_cs.valid_b_ = bc::true_v;

      if (!boundary_cs.per_quality_cs_.valid_b_)
      {
        /** Perception lane quality translation.*/
        /** Avoid repeated calculate.*/
        boundary_cs.per_quality_cs_.std_c0_f_ = t_per_boundary_coeff_cs.std_c0_f_;
        boundary_cs.per_quality_cs_.std_c1_f_ = t_per_boundary_coeff_cs.std_c1_f_;
        boundary_cs.per_quality_cs_.std_c2_f_ = t_per_boundary_coeff_cs.std_c2_f_;
        boundary_cs.per_quality_cs_.std_c3_f_ = t_per_boundary_coeff_cs.std_c3_f_;
        boundary_cs.per_quality_cs_.prob_exist_f_ = t_per_boundary_coeff_cs.prob_exist_f_;
        boundary_cs.per_quality_cs_.coeff_ = t_per_boundary_coeff_cs.coeff_cs_;
        boundary_cs.per_quality_cs_.valid_b_ = t_per_boundary_coeff_cs.valid_b_;
        bc::float32_t t_per_exist_conf_f = t_per_boundary_coeff_cs.prob_exist_f_;
        bc::float32_t t_per_length_f = t_per_boundary_coeff_cs.dx_end_f_ - t_per_boundary_coeff_cs.dx_start_f_;
        bc::float32_t t_per_length_conf_f = LinearInterpolation(5, 0, 40, 1, t_per_length_f);
        boundary_cs.per_quality_cs_.prob_exist_f_ = 0.5 * 1 + 0.5 * 0.9;
      }
    }
  }
  /** 1. Repack leading vehicle lane.*/
  /** Only do when lane idx 2, element 0.*/
  /** ldveh_point_ar_[0] represent hostlane due to this array just one element.*/
  if (ele_idx_u8 == kMainLaneElement && lane_idx_u8 == 2 && ldveh_lane_collection_cs_.ldveh_point_ar_[0].valid_b_)
  {
    BoundaryPoint t_ldv_boundary_point_cs;
    if (l_or_r_u8 == kLeftBoundary)
    {
      t_ldv_boundary_point_cs = ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_;
    }
    if (l_or_r_u8 == kRightBoundary)
    {
      t_ldv_boundary_point_cs = ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_;
    }

    if (t_ldv_boundary_point_cs.valid_b_ &&
        t_ldv_boundary_point_cs.dy_start_idx_u8_ < t_ldv_boundary_point_cs.dy_end_idx_u8_)
    {
      boundary_cs.ldveh_cs_ = t_ldv_boundary_point_cs;
      boundary_cs.valid_b_ = bc::true_v;
    }
  }

  /** 2. Repack map lane.*/
  /** Repack all lanes and all elements.*/
  if (map_lane_collection_cs_.map_point_ar_[lane_idx_u8].valid_b_ &&
      map_lane_collection_cs_.map_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].valid_b_)
  {
    BoundaryPoint t_map_boundary_point_cs;
    if (l_or_r_u8 == kLeftBoundary)
    {
      t_map_boundary_point_cs =
          map_lane_collection_cs_.map_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].left_point_cs_;
    }
    if (l_or_r_u8 == kRightBoundary)
    {
      t_map_boundary_point_cs =
          map_lane_collection_cs_.map_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].right_point_cs_;
    }

    if (t_map_boundary_point_cs.valid_b_)
    {
      boundary_cs.map_cs_ = t_map_boundary_point_cs;
      boundary_cs.valid_b_ = bc::true_v;

      if (map_lane_collection_cs_.map_quality_ar_.valid_b_)
      {
        /** Map lane quality translation.*/
        /** Avoid repeated calculate.*/
        boundary_cs.map_quality_cs_ = map_lane_collection_cs_.map_quality_ar_;
      }
    }
  }

  /** 3. Repack tracked lane.*/
  if (ego_track_lane_collection_cs_.ego_track_point_ar_[lane_idx_u8].valid_b_ &&
      ego_track_lane_collection_cs_.ego_track_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].valid_b_)
  {
    BoundaryPoint t_etl_boundary_point_cs;
    if (l_or_r_u8 == kLeftBoundary)
    {
      BoundaryPoint& t_ego_left_points_cs = ego_track_lane_collection_cs_.ego_track_point_ar_[lane_idx_u8]
                                                .lane_element_point_ar_[ele_idx_u8]
                                                .left_point_cs_;
      t_etl_boundary_point_cs = t_ego_left_points_cs;
      if (map_lane_collection_cs_.map_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].element_id_ != 0 ||
          (t_ego_left_points_cs.valid_b_ &&
           (per_lane_collection_cs_.per_point_ar_[lane_idx_u8].left_point_cs_.per_line_id_s32_ ==
                t_ego_left_points_cs.per_line_id_s32_ ||
            t_ego_left_points_cs.per_line_id_s32_ == -1)))
      {
        /// TODO: 非平行假设相关函数
        // if (t_ego_left_points_cs.per_line_id_s32_ == -1)
        // {
        //   boundary_cs.per_cs_.valid_b_ = bc::false_v;
        //   boundary_cs.per_quality_cs_.valid_b_ = bc::false_v;
        // }
      }
      else if (per_lane_collection_cs_.per_point_ar_[lane_idx_u8].left_point_cs_.per_line_id_s32_ == -1 &&
               t_ego_left_points_cs.per_line_id_s32_ != -1)
      {
        boundary_cs.per_cs_.valid_b_ = bc::false_v;
        boundary_cs.per_quality_cs_.valid_b_ = bc::false_v;
      }
    }
    if (l_or_r_u8 == kRightBoundary)
    {
      BoundaryPoint& t_ego_right_points_cs = ego_track_lane_collection_cs_.ego_track_point_ar_[lane_idx_u8]
                                                 .lane_element_point_ar_[ele_idx_u8]
                                                 .right_point_cs_;
      t_etl_boundary_point_cs = t_ego_right_points_cs;
      if (map_lane_collection_cs_.map_point_ar_[lane_idx_u8].lane_element_point_ar_[ele_idx_u8].element_id_ != 0 ||
          (t_ego_right_points_cs.valid_b_ &&
           (per_lane_collection_cs_.per_point_ar_[lane_idx_u8].right_point_cs_.per_line_id_s32_ ==
                t_ego_right_points_cs.per_line_id_s32_ ||
            t_ego_right_points_cs.per_line_id_s32_ == -1)))
      {
        /// TODO: 非平行假设相关函数
        // if (t_ego_right_points_cs.per_line_id_s32_ == -1)
        // {
        //   boundary_cs.per_cs_.valid_b_ = bc::false_v;
        //   boundary_cs.per_quality_cs_.valid_b_ = bc::false_v;
        // }
      }
      else if (per_lane_collection_cs_.per_point_ar_[lane_idx_u8].right_point_cs_.per_line_id_s32_ == -1 &&
               t_ego_right_points_cs.per_line_id_s32_ != -1)
      {
        boundary_cs.per_cs_.valid_b_ = bc::false_v;
        boundary_cs.per_quality_cs_.valid_b_ = bc::false_v;
      }
    }

    if (t_etl_boundary_point_cs.valid_b_ &&
        t_etl_boundary_point_cs.dy_start_idx_u8_ < t_etl_boundary_point_cs.dy_end_idx_u8_)
    {
      boundary_cs.ref_cs_ = t_etl_boundary_point_cs;
      boundary_cs.valid_b_ = bc::true_v;
    }
  }

  // Check source pack validity
  if (!boundary_cs.per_cs_.valid_b_ && !boundary_cs.map_cs_.valid_b_ && !boundary_cs.ldveh_cs_.valid_b_ &&
      !boundary_cs.ref_cs_.valid_b_)
  {
    boundary_cs.valid_b_ = bc::false_v;
  }
}

void Preprocess::BoundaryJoint(const float32_t joint_similarity_threshold, BoundaryPack& boundary_pack_cs)
{
  if (boundary_pack_cs.lane_pack_ar_[kHostLane].valid_b_)
  {
    if (boundary_pack_cs.lane_pack_ar_[kHostLane].element_pack_ar_[0].valid_b_)
    {
      BoundaryPointPack& left = boundary_pack_cs.lane_pack_ar_[kHostLane].element_pack_ar_[0].left_pack_cs_;
      BoundaryPointPack& right = boundary_pack_cs.lane_pack_ar_[kHostLane].element_pack_ar_[0].right_pack_cs_;
      if (left.valid_b_ && left.ref_cs_.valid_b_ && (left.per_cs_.valid_b_ || left.ldveh_cs_.valid_b_))
      {
        IndividualBoundaryJoint(kLeftBoundary, joint_similarity_threshold, left);
      }
      else
      {
        joint_points_collection_cs_[0] = JointPointData();
        joint_points_collection_cs_[2] = JointPointData();
      }
      if (right.valid_b_ && right.ref_cs_.valid_b_ && (right.per_cs_.valid_b_ || right.ldveh_cs_.valid_b_))
      {
        IndividualBoundaryJoint(kRightBoundary, joint_similarity_threshold, right);
      }
      else
      {
        joint_points_collection_cs_[1] = JointPointData();
        joint_points_collection_cs_[3] = JointPointData();
      }
    }
    else
    {
      joint_points_collection_cs_[0] = JointPointData();
      joint_points_collection_cs_[1] = JointPointData();
      joint_points_collection_cs_[2] = JointPointData();
      joint_points_collection_cs_[3] = JointPointData();
    }
  }
  else
  {
    joint_points_collection_cs_[0] = JointPointData();
    joint_points_collection_cs_[1] = JointPointData();
    joint_points_collection_cs_[2] = JointPointData();
    joint_points_collection_cs_[3] = JointPointData();
  }
}
void Preprocess::IndividualBoundaryJoint(const bc::uint8_t dir_u8, const float32_t joint_similarity_threshold,
                                         BoundaryPointPack& boundary_point_pack)
{
  bc::uint8_t t_idx_u8 = dir_u8 + kSourcePer;
  if (t_idx_u8 < 4)
  {
    if (boundary_point_pack.per_cs_.per_line_id_s32_ == joint_points_collection_cs_[t_idx_u8].lst_id_ &&
        joint_points_collection_cs_[t_idx_u8].still_exist_b_)
    {
      if (joint_points_collection_cs_[t_idx_u8].lst_cross_idx_ > kFrontStartIdx)
      {
        for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < joint_points_collection_cs_[t_idx_u8].lst_cross_idx_;
             t_pt_idx_u8++)
        {
          boundary_point_pack.per_cs_.dy_ar_[t_pt_idx_u8] = boundary_point_pack.ref_cs_.dy_ar_[t_pt_idx_u8];
        }
      }
      else
      {
        joint_points_collection_cs_[t_idx_u8] = JointPointData();
      }
    }
    else
    {
      if (boundary_point_pack.per_cs_.valid_b_ && x_ref_ar_[boundary_point_pack.per_cs_.dy_start_idx_u8_] > 3 &&
          boundary_point_pack.ref_cs_.per_line_id_s32_ == -1 && boundary_point_pack.per_cs_.per_line_id_s32_ != -1)
      {
        bc::float32_t t_similarity_f = CheckMapPerLaneSimilarity(
            x_ref_ar_, boundary_point_pack.ref_cs_.dy_ar_, boundary_point_pack.per_cs_.dy_ar_,
            boundary_point_pack.per_cs_.dy_start_idx_u8_, boundary_point_pack.per_cs_.dy_end_idx_u8_);
        if (dir_u8 == kLeftBoundary)
        {
          left_per_ref_similarity_f_ = t_similarity_f;
        }
        else
        {
          right_per_ref_similarity_f_ = t_similarity_f;
        }
        if (t_similarity_f < joint_similarity_threshold)
        {
          SourceBoundaryJoint(dir_u8, kSourcePer, boundary_point_pack.ref_cs_, boundary_point_pack.per_cs_);
        }
        else
        {
          joint_points_collection_cs_[t_idx_u8] = JointPointData();
        }
      }
      else
      {
        joint_points_collection_cs_[t_idx_u8] = JointPointData();
      }
    }
  }

  t_idx_u8 = dir_u8 + kSourceLdveh;
  if (t_idx_u8 < 4)
  {

    if (boundary_point_pack.ldveh_cs_.per_line_id_s32_ == joint_points_collection_cs_[t_idx_u8].lst_id_ &&
        joint_points_collection_cs_[t_idx_u8].still_exist_b_)
    {
      if (joint_points_collection_cs_[t_idx_u8].lst_cross_idx_ > kFrontStartIdx)
      {
        for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < joint_points_collection_cs_[t_idx_u8].lst_cross_idx_;
             t_pt_idx_u8++)
        {
          boundary_point_pack.ldveh_cs_.dy_ar_[t_pt_idx_u8] = boundary_point_pack.ref_cs_.dy_ar_[t_pt_idx_u8];
        }
      }
      else
      {
        joint_points_collection_cs_[t_idx_u8] = JointPointData();
      }
    }
    else
    {
      if (boundary_point_pack.ldveh_cs_.valid_b_ && x_ref_ar_[boundary_point_pack.ldveh_cs_.dy_start_idx_u8_] > 3)
      {
        SourceBoundaryJoint(dir_u8, kSourceLdveh, boundary_point_pack.ref_cs_, boundary_point_pack.ldveh_cs_);
      }
      else
      {
        joint_points_collection_cs_[t_idx_u8] = JointPointData();
      }
    }
  }
}
void Preprocess::SourceBoundaryJoint(const bc::uint8_t dir_u8, const bc::uint8_t source_u8,
                                     const BoundaryPoint& ref_boundary, BoundaryPoint& tar_boundary)
{
  bc::uint8_t t_idx_u8 = dir_u8 + source_u8;
  if (t_idx_u8 < 4)
  {
    bc::uint8_t t_start_idx_u8 = 0;
    bc::uint8_t t_end_idx_u8 = 0;
    Point2D t_cross_point;
    bc::uint8_t t_cross_idx = FindCrossingPoint(ref_boundary, tar_boundary, t_cross_point);
    if (t_cross_idx != kMaxBoundaryPoint && t_cross_idx > kFrontStartIdx)
    {
      joint_points_collection_cs_[t_idx_u8].lst_id_ = tar_boundary.per_line_id_s32_;
      joint_points_collection_cs_[t_idx_u8].dir_u8_ = dir_u8;
      joint_points_collection_cs_[t_idx_u8].source_u8_ = source_u8;
      joint_points_collection_cs_[t_idx_u8].joint_b_ = bc::true_v;
      joint_points_collection_cs_[t_idx_u8].lst_cross_idx_ = t_cross_idx;
      joint_points_collection_cs_[t_idx_u8].lst_cross_point_ = t_cross_point;
      joint_points_collection_cs_[t_idx_u8].still_exist_b_ = bc::false_v;
      for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < t_cross_idx; t_idx_u8++)
      {
        tar_boundary.dy_ar_[t_idx_u8] = ref_boundary.dy_ar_[t_idx_u8];
      }
    }
    else
    {
      joint_points_collection_cs_[t_idx_u8] = JointPointData();
    }
  }

  //   if (ref_boundary.dy_end_idx_u8_ < tar_boundary.dy_start_idx_u8_)
  //   {
  //     t_start_idx_u8 = ref_boundary.dy_end_idx_u8_;
  //     t_end_idx_u8 = tar_boundary.dy_start_idx_u8_;
  //   }
  //   else if (ref_boundary.dy_end_idx_u8_ >= tar_boundary.dy_start_idx_u8_ &&
  //            ref_boundary.dy_end_idx_u8_ <= tar_boundary.dy_end_idx_u8_)
  //   {
  //     t_start_idx_u8 = tar_boundary.dy_start_idx_u8_;
  //     if (x_ref_ar_[ref_boundary.dy_end_idx_u8_] - x_ref_ar_[t_start_idx_u8] > 10)
  //     {
  //       t_end_idx_u8 = ref_boundary.dy_end_idx_u8_;
  //     }
  //     else
  //     {
  //       for (bc::uint8_t t_idx_u8 = ref_boundary.dy_end_idx_u8_; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  //       {
  //         if (x_ref_ar_[t_idx_u8] - x_ref_ar_[t_start_idx_u8] > 10)
  //         {
  //           t_end_idx_u8 = t_idx_u8;
  //           break;
  //         }
  //       }
  //     }
  //   }
  //   else
  //   {
  //     t_start_idx_u8 = tar_boundary.dy_start_idx_u8_;
  //     for (bc::uint8_t t_idx_u8 = t_start_idx_u8; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  //     {
  //       if (x_ref_ar_[t_idx_u8] - x_ref_ar_[t_start_idx_u8] > 10)
  //       {
  //         t_end_idx_u8 = t_idx_u8;
  //         break;
  //       }
  //     }
  //   }

  // for(bc::uint8_t t_pt_idx_u8 =0; t_pt_idx_u8<=t_start_idx_u8;t_pt_idx_u8++){
  //   tar_boundary.dy_ar_[t_pt_idx_u8] =   ref_boundary.dy_ar_[t_pt_idx_u8] ;
  // }
  // for( bc::uint8_t t_pt_idx_u8 =t_start_idx_u8; t_pt_idx_u8<=t_end_idx_u8;t_pt_idx_u8++){
  //   tar_boundary.dy_ar_[t_pt_idx_u8] =
  //       LinearInterpolation(x_ref_ar_[t_start_idx_u8], ref_boundary.dy_ar_[t_start_idx_u8], x_ref_ar_[t_end_idx_u8],
  //                           tar_boundary.dy_ar_[t_end_idx_u8], x_ref_ar_[t_pt_idx_u8]);
  // }
  // bc::uint8_t t_st_idx_u8 = ((t_start_idx_u8- 5) < kMaxBoundaryPoint)? (t_start_idx_u8- 5 ):0;
  // bc::uint8_t t_ed_idx_u8 = ((t_end_idx_u8 + 5 )< kMaxBoundaryPoint) ?( t_end_idx_u8 + 5) : (kMaxBoundaryPoint-1);
  // bc::uint8_t t_valid_num = t_ed_idx_u8 - t_st_idx_u8 + 1;
  // bc::TCArray<bc::float32_t, kMaxBoundaryPoint> t_out_pts_y = BoundarySmoother<bc::float32_t, kMaxBoundaryPoint>(
  // t_valid_num, t_st_idx_u8, 8.0, 10, x_ref_ar_,tar_boundary.dy_ar_);
  // for (bc::uint8_t t_idx_u8 = t_st_idx_u8; t_idx_u8 <= t_ed_idx_u8; t_idx_u8++)
  // {
  //   tar_boundary.dy_ar_[t_idx_u8] = t_out_pts_y[t_idx_u8];
  // }
}

bc::uint8_t Preprocess::FindCrossingPoint(const BoundaryPoint& ref_boundary, BoundaryPoint& tar_boundary,
                                          Point2D& cross_point)
{
  bc::int8_t t_lst_compare_s8 = 0;
  bc::int8_t t_this_compare_s8 = 0;
  bc::TCArray<bc::int8_t, kMaxBoundaryPoint> compare_s8_ar{0};
  bc::uint8_t t_cross_idx_u8 = 100;
  bc::float32_t t_diff_f;
  bc::bool_t t_find_cross_b = bc::false_v;
  /// 如果相交点不止一个，先找在start_end_idx范围内的交叉点，以target boundary的start_end_idx为基准，其后是reference
  /// boundary
  /// set diff compare
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
  {
    t_diff_f = ref_boundary.dy_ar_[t_idx_u8] - tar_boundary.dy_ar_[t_idx_u8];
    if (fabsf(t_diff_f) < 0.01)
    {
      compare_s8_ar[t_idx_u8] = 0;
    }
    else if (t_diff_f < 0)
    {
      compare_s8_ar[t_idx_u8] = -1;
    }
    else
    {
      compare_s8_ar[t_idx_u8] = 1;
    }
  }

  for (bc::uint8_t t_idx_u8 = tar_boundary.dy_start_idx_u8_; t_idx_u8 <= tar_boundary.dy_end_idx_u8_; t_idx_u8++)
  {
    if (compare_s8_ar[t_idx_u8] == 0)
    {
      t_cross_idx_u8 = t_idx_u8;
      cross_point.x = x_ref_ar_[t_idx_u8];
      cross_point.y = tar_boundary.dy_ar_[t_idx_u8];
      t_find_cross_b = bc::true_v;
      break;
    }
    else
    {
      if (compare_s8_ar[t_idx_u8 - 1] * compare_s8_ar[t_idx_u8] == -1)
      {
        t_cross_idx_u8 = t_idx_u8;
        Point2D p_A(x_ref_ar_[t_idx_u8 - 1], ref_boundary.dy_ar_[t_idx_u8 - 1]);
        Point2D p_B(x_ref_ar_[t_idx_u8], ref_boundary.dy_ar_[t_idx_u8]);
        Point2D p_C(x_ref_ar_[t_idx_u8 - 1], tar_boundary.dy_ar_[t_idx_u8 - 1]);
        Point2D p_D(x_ref_ar_[t_idx_u8], tar_boundary.dy_ar_[t_idx_u8]);
        cross_point = FindCrossPoint(p_A, p_B, p_C, p_D);
        t_find_cross_b = bc::true_v;
        break;
      }
    }
  }

  if (!t_find_cross_b)
  {
    if (ref_boundary.dy_start_idx_u8_ < tar_boundary.dy_start_idx_u8_)
    {
      for (bc::int8_t t_idx_s8 = tar_boundary.dy_start_idx_u8_; t_idx_s8 >= ref_boundary.dy_start_idx_u8_; t_idx_s8--)
      {
        if (compare_s8_ar[t_idx_s8] == 0)
        {
          t_cross_idx_u8 = t_idx_s8;
          cross_point.x = x_ref_ar_[t_idx_s8];
          cross_point.y = tar_boundary.dy_ar_[t_idx_s8];
          t_find_cross_b = bc::true_v;
          break;
        }
        else
        {
          if (t_idx_s8 != 0)
          {
            if (compare_s8_ar[t_idx_s8 - 1] * compare_s8_ar[t_idx_s8] == -1)
            {
              t_cross_idx_u8 = t_idx_s8;
              Point2D p_A(x_ref_ar_[t_idx_s8 - 1], ref_boundary.dy_ar_[t_idx_s8 - 1]);
              Point2D p_B(x_ref_ar_[t_idx_s8], ref_boundary.dy_ar_[t_idx_s8]);
              Point2D p_C(x_ref_ar_[t_idx_s8 - 1], tar_boundary.dy_ar_[t_idx_s8 - 1]);
              Point2D p_D(x_ref_ar_[t_idx_s8], tar_boundary.dy_ar_[t_idx_s8]);
              cross_point = FindCrossPoint(p_A, p_B, p_C, p_D);
              t_find_cross_b = bc::true_v;
              break;
            }
          }
        }
      }
    }
    if (ref_boundary.dy_end_idx_u8_ > tar_boundary.dy_end_idx_u8_)
    {
      for (bc::uint8_t t_idx_u8 = tar_boundary.dy_end_idx_u8_; t_idx_u8 <= ref_boundary.dy_end_idx_u8_; t_idx_u8++)
      {
        if (compare_s8_ar[t_idx_u8] == 0)
        {
          t_cross_idx_u8 = t_idx_u8;
          cross_point.x = x_ref_ar_[t_idx_u8];
          cross_point.y = tar_boundary.dy_ar_[t_idx_u8];
          t_find_cross_b = bc::true_v;
          break;
        }
        else
        {
          if (compare_s8_ar[t_idx_u8 - 1] * compare_s8_ar[t_idx_u8] == -1)
          {
            t_cross_idx_u8 = t_idx_u8;
            Point2D p_A(x_ref_ar_[t_idx_u8 - 1], ref_boundary.dy_ar_[t_idx_u8 - 1]);
            Point2D p_B(x_ref_ar_[t_idx_u8], ref_boundary.dy_ar_[t_idx_u8]);
            Point2D p_C(x_ref_ar_[t_idx_u8 - 1], tar_boundary.dy_ar_[t_idx_u8 - 1]);
            Point2D p_D(x_ref_ar_[t_idx_u8], tar_boundary.dy_ar_[t_idx_u8]);
            cross_point = FindCrossPoint(p_A, p_B, p_C, p_D);
            t_find_cross_b = bc::true_v;
            break;
          }
        }
      }
    }
  }

  if (!t_find_cross_b)
  {
    if (tar_boundary.dy_start_idx_u8_ > 0 && ref_boundary.dy_start_idx_u8_ > 0)
    {
      for (bc::int8_t t_idx_s8 = std::min(tar_boundary.dy_start_idx_u8_, ref_boundary.dy_start_idx_u8_); t_idx_s8 >= 0;
           t_idx_s8--)
      {
        if (compare_s8_ar[t_idx_s8] == 0)
        {
          t_cross_idx_u8 = t_idx_s8;
          cross_point.x = x_ref_ar_[t_idx_s8];
          cross_point.y = tar_boundary.dy_ar_[t_idx_s8];
          t_find_cross_b = bc::true_v;
          break;
        }
        else
        {
          if (t_idx_s8 != 0)
          {
            if (compare_s8_ar[t_idx_s8 - 1] * compare_s8_ar[t_idx_s8] == -1)
            {
              t_cross_idx_u8 = t_idx_s8;
              Point2D p_A(x_ref_ar_[t_idx_s8 - 1], ref_boundary.dy_ar_[t_idx_s8 - 1]);
              Point2D p_B(x_ref_ar_[t_idx_s8], ref_boundary.dy_ar_[t_idx_s8]);
              Point2D p_C(x_ref_ar_[t_idx_s8 - 1], tar_boundary.dy_ar_[t_idx_s8 - 1]);
              Point2D p_D(x_ref_ar_[t_idx_s8], tar_boundary.dy_ar_[t_idx_s8]);
              cross_point = FindCrossPoint(p_A, p_B, p_C, p_D);
              t_find_cross_b = bc::true_v;
              break;
            }
          }
        }
      }
    }
  }
  if (!t_find_cross_b)
  {
    if (tar_boundary.dy_end_idx_u8_ < kMaxBoundaryPoint - 1 && ref_boundary.dy_end_idx_u8_ < kMaxBoundaryPoint - 1)
    {
      for (bc::uint8_t t_idx_u8 = std::max(tar_boundary.dy_end_idx_u8_, ref_boundary.dy_end_idx_u8_);
           t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
      {
        if (compare_s8_ar[t_idx_u8] == 0)
        {
          t_cross_idx_u8 = t_idx_u8;
          cross_point.x = x_ref_ar_[t_idx_u8];
          cross_point.y = tar_boundary.dy_ar_[t_idx_u8];
          t_find_cross_b = bc::true_v;
          break;
        }
        else
        {
          if (compare_s8_ar[t_idx_u8 - 1] * compare_s8_ar[t_idx_u8] == -1)
          {
            t_cross_idx_u8 = t_idx_u8;
            Point2D p_A(x_ref_ar_[t_idx_u8 - 1], ref_boundary.dy_ar_[t_idx_u8 - 1]);
            Point2D p_B(x_ref_ar_[t_idx_u8], ref_boundary.dy_ar_[t_idx_u8]);
            Point2D p_C(x_ref_ar_[t_idx_u8 - 1], tar_boundary.dy_ar_[t_idx_u8 - 1]);
            Point2D p_D(x_ref_ar_[t_idx_u8], tar_boundary.dy_ar_[t_idx_u8]);
            cross_point = FindCrossPoint(p_A, p_B, p_C, p_D);
            t_find_cross_b = bc::true_v;
            break;
          }
        }
      }
    }
  }
  return t_cross_idx_u8;
}

Point2D Preprocess::FindCrossPoint(const Point2D& p_A, const Point2D& p_B, const Point2D& p_C, const Point2D& p_D)
{
  Point2D vec_AB(p_B.x - p_A.x, p_B.y - p_A.y);
  Point2D vec_AC(p_C.x - p_A.x, p_C.y - p_A.y);
  Point2D vec_AD(p_D.x - p_A.x, p_D.y - p_A.y);
  Point2D p_P;
  bc::float32_t norm_vec_AC_AB_f =
      sqrt(vec_AC.x * vec_AB.x * vec_AC.x * vec_AB.x + vec_AC.y * vec_AB.y * vec_AC.y * vec_AB.y);
  bc::float32_t norm_vec_AD_AB_f =
      sqrt(vec_AD.x * vec_AB.x * vec_AD.x * vec_AB.x + vec_AD.y * vec_AB.y * vec_AD.y * vec_AB.y);
  bc::float32_t t_lamda_f = norm_vec_AC_AB_f / norm_vec_AD_AB_f;
  p_P.x = (p_C.x + t_lamda_f * p_D.x) / (1 + t_lamda_f);
  p_P.y = (p_C.y + t_lamda_f * p_D.y) / (1 + t_lamda_f);
  return p_P;
}

void Preprocess::DeductPerBoundary(const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                                   const bc::float32_t time_cycle_f, const bc::float32_t ego_vel_f)
{
  PerLaneCoeff* t_internal_ptr = nullptr;
  /** 0. HostLane.*/
  t_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[kHostLane]);

  // judge whether lane change happens
  per_lane_change_state_ = PerLaneChangeState::kDrvStraight;
  // lane keep
  if ((t_internal_ptr->left_coeff_cs_.valid_b_ &&
       t_internal_ptr->left_coeff_cs_.per_line_id_s32_ ==
           per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_) &&
      (t_internal_ptr->right_coeff_cs_.valid_b_ &&
       t_internal_ptr->right_coeff_cs_.per_line_id_s32_ ==
           per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_))
  {
    // do nothing
  }
  // right lane split
  else if ((t_internal_ptr->left_coeff_cs_.valid_b_ &&
            t_internal_ptr->left_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_) &&
           (per_lane_collection_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.valid_b_ &&
            per_lane_collection_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_))
  {
    per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
  }
  // left lane split
  else if ((t_internal_ptr->right_coeff_cs_.valid_b_ &&
            t_internal_ptr->right_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_) &&
           (per_lane_collection_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.valid_b_ &&
            per_lane_collection_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_))
  {
    per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
  }
  // crossing right line
  else if ((((t_internal_ptr->left_coeff_cs_.valid_b_ &&
              t_internal_ptr->left_coeff_cs_.per_line_id_s32_ ==
                  per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_) ||
             (per_lane_collection_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.valid_b_ &&
              per_lane_collection_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.per_line_id_s32_ ==
                  per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_)) &&
            (t_internal_ptr->right_coeff_cs_.per_line_id_s32_ !=
             per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_)))
  {
    per_lane_change_state_ = PerLaneChangeState::kDrvRight;
    per_lane_width_filter_state_ar_[kLeftBoundary] = per_lane_width_filter_state_ar_[1];
    per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
    per_left_c0_lst_f_ = per_right_c0_lst_f_;
  }
  // crossing left line
  else if ((((t_internal_ptr->right_coeff_cs_.valid_b_ &&
              t_internal_ptr->right_coeff_cs_.per_line_id_s32_ ==
                  per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_) ||
             (per_lane_collection_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.valid_b_ &&
              per_lane_collection_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.per_line_id_s32_ ==
                  per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_)) &&
            (t_internal_ptr->left_coeff_cs_.per_line_id_s32_ !=
             per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_)))
  {
    per_lane_change_state_ = PerLaneChangeState::kDrvLeft;
    per_lane_width_filter_state_ar_[kRightBoundary] = per_lane_width_filter_state_ar_[0];
    per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
    per_right_c0_lst_f_ = per_left_c0_lst_f_;
  }
  // host lane merges into left lane
  else if ((t_internal_ptr->right_coeff_cs_.valid_b_ &&
            t_internal_ptr->right_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.per_line_id_s32_) &&
           (t_internal_ptr->left_coeff_cs_.valid_b_ &&
            per_lane_collection_lst_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.per_line_id_s32_ ==
                t_internal_ptr->left_coeff_cs_.per_line_id_s32_))
  {
    per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
    per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
    per_left_c0_lst_f_ = per_lane_collection_cs_.per_coeff_ar_[kLeftLane].left_coeff_cs_.coeff_cs_.c0_position;
    per_right_c0_lst_f_ = per_lane_collection_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.coeff_cs_.c0_position;
  }
  // host lane merges into right lane
  else if ((t_internal_ptr->left_coeff_cs_.valid_b_ &&
            t_internal_ptr->left_coeff_cs_.per_line_id_s32_ ==
                per_lane_collection_lst_cs_.per_coeff_ar_[kHostLane].left_coeff_cs_.per_line_id_s32_) &&
           (t_internal_ptr->right_coeff_cs_.valid_b_ &&
            per_lane_collection_lst_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.per_line_id_s32_ ==
                t_internal_ptr->right_coeff_cs_.per_line_id_s32_))
  {
    per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
    per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
    per_right_c0_lst_f_ = per_lane_collection_cs_.per_coeff_ar_[kRightLane].right_coeff_cs_.coeff_cs_.c0_position;
    per_left_c0_lst_f_ = per_lane_collection_cs_.per_coeff_ar_[kHostLane].right_coeff_cs_.coeff_cs_.c0_position;
  }
  // line ids are irregular
  else
  {
    // do nothing
  }

  // calc width change rate upon different ego vel
  /*
  here should be acceptable ref c0 ramp rate.
  1. one boundary ramp, ref ramp rate = 0.5*c0 ramp rate. In this case, width ramp rate = c0 ramp rate.
  2. two boundary ramp, ref ramp rate = either 1*c0 ramp rate or 0. In this case, width ramp rate = either 0 or 2*c0
  ramp rate, respectively.
      width ramp rate = 2*c0 ramp rate is not a problem because in this situation, ref ramp rate = 0.
  */
  bc::float32_t t_c0_ramp_rate_f = 0.f;
  t_c0_ramp_rate_f = LinearInterpolation(kEgoLowSpeedBoundary, 0.f, 16.f, 1.f, ego_vel_f, Lin_Interp_Method::kFlat);
  bc::float32_t t_c0_ramp_step_f = t_c0_ramp_rate_f * time_cycle_f;
  bc::float32_t t_c0_left_new_f = t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position;
  bc::float32_t t_c0_right_new_f = t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position;
  bc::float32_t t_c0_left_target_f = t_c0_left_new_f;
  bc::float32_t t_c0_right_target_f = t_c0_right_new_f;
  bc::float32_t t_width_pred_f = kDefaultElementWidth;

  if (t_internal_ptr->left_coeff_cs_.valid_b_ == bc::true_v && t_internal_ptr->right_coeff_cs_.valid_b_ == bc::true_v)
  {
    /* both valid, keep default value, following code is for understanding purpose only.
    t_c0_left_target_f = t_c0_left_new_f;
    t_c0_right_target_f = t_c0_right_new_f;
    */
    per_lane_width_lst1_f_ = fabsf(t_c0_left_target_f - t_c0_right_target_f);
  }
  else if (t_internal_ptr->left_coeff_cs_.valid_b_ == bc::true_v)
  {
    /* left valid, right invalid */
    t_c0_right_target_f = t_c0_left_target_f - t_width_pred_f;
  }
  else if (t_internal_ptr->right_coeff_cs_.valid_b_ == bc::true_v)
  {
    /* left invalid, right valid */
    t_c0_left_target_f = t_c0_right_target_f + t_width_pred_f;
  }
  else
  {
    /* both invalid, keep default value, following code is for understanding purpose only.
    t_c0_left_target_f = t_c0_left_new_f;
    t_c0_right_target_f = t_c0_right_new_f;
    */
  }

  if (t_internal_ptr->left_coeff_cs_.valid_b_ == bc::false_v && t_internal_ptr->right_coeff_cs_.valid_b_ == bc::false_v)
  {
    per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
    per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
  }
  else
  {
    /* there is at least one valid boundary */
    /* can change following code to be a function*/
    bc::bool_t t_left_boundary_valid_b = t_internal_ptr->left_coeff_cs_.valid_b_;
    switch (per_lane_width_filter_state_ar_[kLeftBoundary])
    {
      case PerLaneWidthFilterState::kDefault:
      {
        if (t_left_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kMeasured;
        }
        else
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kPredicted;
        }
        break;
      }

      case PerLaneWidthFilterState::kMeasured:
      {
        if (t_left_boundary_valid_b == bc::false_v)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kMeaToPred;
        }
        break;
      }

      case PerLaneWidthFilterState::kMeaToPred:
      {
        if (t_left_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kPredToMea;
        }
        else if (fabsf(t_c0_left_target_f - per_left_c0_lst_f_) < t_c0_ramp_step_f)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kPredicted;
        }
        break;
      }

      case PerLaneWidthFilterState::kPredicted:
      {
        if (t_left_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kPredToMea;
        }
        break;
      }

      case PerLaneWidthFilterState::kPredToMea:
      {
        if (t_left_boundary_valid_b == bc::false_v)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kMeaToPred;
        }
        else if (fabsf(t_c0_left_target_f - per_left_c0_lst_f_) < t_c0_ramp_step_f)
        {
          per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kMeasured;
        }
        break;
      }
      default:
      {
        per_lane_width_filter_state_ar_[kLeftBoundary] = PerLaneWidthFilterState::kDefault;
        break;
      }
    }

    bc::bool_t t_right_boundary_valid_b = t_internal_ptr->right_coeff_cs_.valid_b_;
    switch (per_lane_width_filter_state_ar_[kRightBoundary])
    {
      case PerLaneWidthFilterState::kDefault:
      {
        if (t_right_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kMeasured;
        }
        else
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kPredicted;
        }
        break;
      }

      case PerLaneWidthFilterState::kMeasured:
      {
        if (t_right_boundary_valid_b == bc::false_v)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kMeaToPred;
        }
        break;
      }

      case PerLaneWidthFilterState::kMeaToPred:
      {
        if (t_right_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kPredToMea;
        }
        else if (fabsf(t_c0_right_target_f - per_right_c0_lst_f_) < t_c0_ramp_step_f)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kPredicted;
        }
        break;
      }

      case PerLaneWidthFilterState::kPredicted:
      {
        if (t_right_boundary_valid_b == bc::true_v)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kPredToMea;
        }
        break;
      }

      case PerLaneWidthFilterState::kPredToMea:
      {
        if (t_right_boundary_valid_b == bc::false_v)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kMeaToPred;
        }
        else if (fabsf(t_c0_right_target_f - per_right_c0_lst_f_) < t_c0_ramp_step_f)
        {
          per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kMeasured;
        }
        break;
      }
      default:
      {
        per_lane_width_filter_state_ar_[kRightBoundary] = PerLaneWidthFilterState::kDefault;
        break;
      }
    }
  }

  switch (per_lane_width_filter_state_ar_[kLeftBoundary])
  {
    case PerLaneWidthFilterState::kDefault:
    case PerLaneWidthFilterState::kMeasured:
    {
      t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = t_c0_left_target_f;
      break;
    }

    case PerLaneWidthFilterState::kPredicted:
    {
      t_internal_ptr->left_coeff_cs_ = t_internal_ptr->right_coeff_cs_;
      t_internal_ptr->left_coeff_cs_.per_line_id_s32_ = -1;
      t_internal_ptr->left_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
      t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = t_c0_left_target_f;
      break;
    }

    case PerLaneWidthFilterState::kMeaToPred:
    {
      t_internal_ptr->left_coeff_cs_ = t_internal_ptr->right_coeff_cs_;
      t_internal_ptr->left_coeff_cs_.per_line_id_s32_ = -1;
      t_internal_ptr->left_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
      bc::float32_t t_c0_diff_f = t_c0_left_target_f - per_left_c0_lst_f_;
      if (fabs(t_c0_diff_f) < t_c0_ramp_step_f)
      {
        t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = t_c0_left_target_f;
      }
      else
      {
        if (t_c0_diff_f > 0)
        {
          if (t_c0_diff_f > 1.f)
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f;
          }
        }
        else
        {
          if (t_c0_diff_f < -1.f)
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ - t_c0_ramp_step_f;
          }
        }
      }
      break;
    }

    case PerLaneWidthFilterState::kPredToMea:
    {
      bc::float32_t t_c0_diff_f = t_c0_left_target_f - per_left_c0_lst_f_;
      if (fabs(t_c0_diff_f) < t_c0_ramp_step_f)
      {
        t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = t_c0_left_target_f;
      }
      else
      {
        if (t_c0_diff_f > 0)
        {
          if (t_c0_diff_f > 1.f)
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f;
          }
        }
        else
        {
          if (t_c0_diff_f < -1.f)
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position = per_left_c0_lst_f_ - t_c0_ramp_step_f;
          }
        }
      }
      break;
    }
  }

  switch (per_lane_width_filter_state_ar_[kRightBoundary])
  {
    case PerLaneWidthFilterState::kDefault:
    case PerLaneWidthFilterState::kMeasured:
    {
      t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = t_c0_right_target_f;
      break;
    }

    case PerLaneWidthFilterState::kPredicted:
    {
      t_internal_ptr->right_coeff_cs_ = t_internal_ptr->left_coeff_cs_;
      t_internal_ptr->right_coeff_cs_.per_line_id_s32_ = -1;
      t_internal_ptr->right_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
      t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = t_c0_right_target_f;
      break;
    }

    case PerLaneWidthFilterState::kMeaToPred:
    {
      t_internal_ptr->right_coeff_cs_ = t_internal_ptr->left_coeff_cs_;
      t_internal_ptr->right_coeff_cs_.per_line_id_s32_ = -1;
      t_internal_ptr->right_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
      bc::float32_t t_c0_diff_f = t_c0_right_target_f - per_right_c0_lst_f_;
      if (fabs(t_c0_diff_f) < t_c0_ramp_step_f)
      {
        t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = t_c0_right_target_f;
      }
      else
      {
        if (t_c0_diff_f > 0)
        {
          if (t_c0_diff_f > 1.f)
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
                per_right_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = per_right_c0_lst_f_ + t_c0_ramp_step_f;
          }
        }
        else
        {
          if (t_c0_diff_f < -1.f)
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
                per_right_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = per_right_c0_lst_f_ - t_c0_ramp_step_f;
          }
        }
      }
      break;
    }

    case PerLaneWidthFilterState::kPredToMea:
    {
      bc::float32_t t_c0_diff_f = t_c0_right_target_f - per_right_c0_lst_f_;
      if (fabs(t_c0_diff_f) < t_c0_ramp_step_f)
      {
        t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = t_c0_right_target_f;
      }
      else
      {
        if (t_c0_diff_f > 0)
        {
          if (t_c0_diff_f > 1.f)
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
                per_right_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = per_right_c0_lst_f_ + t_c0_ramp_step_f;
          }
        }
        else
        {
          if (t_c0_diff_f < -1.f)
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
                per_right_c0_lst_f_ + t_c0_ramp_step_f * t_c0_diff_f;
          }
          else
          {
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position = per_right_c0_lst_f_ - t_c0_ramp_step_f;
          }
        }
      }
      break;
    }
  }

  // store per c0 & lane width of last cycle
  per_left_c0_lst_f_ = t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position;
  per_right_c0_lst_f_ = t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position;
  if (t_internal_ptr->left_coeff_cs_.valid_b_ == bc::false_v && t_internal_ptr->right_coeff_cs_.valid_b_ == bc::false_v)
  {
    per_lane_width_lst1_f_ = kDefaultElementWidth;
  }

  // debug data
  (*em_collection_ptr_).reserve_[69] = t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position;
  (*em_collection_ptr_).reserve_[70] = t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position;
  (*em_collection_ptr_).reserve_[90] = (bc::float32_t)per_lane_width_filter_state_ar_[kLeftBoundary];
  (*em_collection_ptr_).reserve_[91] = (bc::float32_t)per_lane_width_filter_state_ar_[kRightBoundary];
  (*em_collection_ptr_).reserve_[92] = (bc::float32_t)per_lane_change_state_;
  (*em_collection_ptr_).reserve_[93] = (bc::float32_t)per_lane_width_lst1_f_;
  (*em_collection_ptr_).reserve_[94] = per_lane_collection_cs_.per_coeff_ar_[1].left_coeff_cs_.per_line_id_s32_;
  (*em_collection_ptr_).reserve_[95] = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.per_line_id_s32_;
  (*em_collection_ptr_).reserve_[96] = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.per_line_id_s32_;
  (*em_collection_ptr_).reserve_[97] = per_lane_collection_cs_.per_coeff_ar_[3].right_coeff_cs_.per_line_id_s32_;

  if (per_lane_width_lst1_f_ < 1.f)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "HostLaneWidth Error in DeductPerBoundary!!!" << std::endl;
#endif
  }

  /** 1. OtherLane.*/
  for (bc::uint8_t t_lane_idx_u8 = 0U; t_lane_idx_u8 < kMaxLaneNum; ++t_lane_idx_u8)
  {
    if (t_lane_idx_u8 == kHostLane)
    {
      continue;
    }
    t_internal_ptr = &(per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8]);
    if (t_internal_ptr->valid_b_)
    {
      const LaneElement& t_left_ele_lst1_cs = em_lane_lst1_ar[t_lane_idx_u8].lane_elements_[0];
      /** firstly consider current host lane width, then last_em_data left lane width.*/
      if (t_internal_ptr->left_coeff_cs_.valid_b_ && (t_internal_ptr->right_coeff_cs_.valid_b_ == bc::false_v))
      {
        if (t_lane_idx_u8 < kHostLane)
        {
          if (per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 + 1].valid_b_)
          {
            t_internal_ptr->right_coeff_cs_ = per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 + 1].left_coeff_cs_;
            // t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
            //     per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 + 1].left_coeff_cs_.coeff_cs_.c0_position;
          }
          else
          {
            t_internal_ptr->right_coeff_cs_ = t_internal_ptr->left_coeff_cs_;
            t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position =
                t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position - per_lane_width_lst1_f_;
          }
          t_internal_ptr->right_coeff_cs_.per_line_id_s32_ = -1;
          t_internal_ptr->right_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
        }
        else if (t_lane_idx_u8 > kHostLane)
        {
          // t_internal_ptr->valid_b_ = bc::false_v;
        }
      }
      else if ((t_internal_ptr->left_coeff_cs_.valid_b_ == bc::false_v) && t_internal_ptr->right_coeff_cs_.valid_b_)
      {
        if (t_lane_idx_u8 > kHostLane)
        {
          if (per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 - 1].valid_b_)
          {
            t_internal_ptr->left_coeff_cs_ = per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 - 1].right_coeff_cs_;
            // t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position =
            //     per_lane_collection_cs_.per_coeff_ar_[t_lane_idx_u8 - 1].right_coeff_cs_.coeff_cs_.c0_position;
          }
          else
          {
            t_internal_ptr->left_coeff_cs_ = t_internal_ptr->right_coeff_cs_;
            t_internal_ptr->left_coeff_cs_.coeff_cs_.c0_position =
                t_internal_ptr->right_coeff_cs_.coeff_cs_.c0_position + per_lane_width_lst1_f_;
          }
          t_internal_ptr->left_coeff_cs_.per_line_id_s32_ = -1;
          t_internal_ptr->left_coeff_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
        }
        else if (t_lane_idx_u8 < kHostLane)
        {
          t_internal_ptr->valid_b_ = bc::false_v;
        }
      }
      else if ((t_internal_ptr->left_coeff_cs_.valid_b_ == bc::false_v) &&
               (t_internal_ptr->right_coeff_cs_.valid_b_ == bc::false_v))
      {
        // t_internal_ptr->valid_b_ = bc::false_v;
      }
      else
      {
        /** Nothing. */
      }
    }
    else
    {
      /** Nothing. */
    }
  }
  per_lane_collection_lst_cs_ = per_lane_collection_cs_;
}

void Preprocess::DeductOneLaneBoundary(BoundaryPack& boundary_pack_cs, bc::uint8_t lane_idx)
{
  bc::bool_t t_suceess_b = bc::false_v;
  /** 0. Try copy valid neignbor lane info, which close to hostlane has priority.*/
  if (lane_idx < kHostLane)
  {
    if (lane_idx + 1 < kMaxLaneNum && boundary_pack_cs.lane_pack_ar_[lane_idx + 1].valid_b_)
    {
      boundary_pack_cs.lane_pack_ar_[lane_idx] = boundary_pack_cs.lane_pack_ar_[lane_idx + 1];
      t_suceess_b = bc::true_v;
    }
    else if (lane_idx - 1 >= 0 && boundary_pack_cs.lane_pack_ar_[lane_idx - 1].valid_b_)
    {
      boundary_pack_cs.lane_pack_ar_[lane_idx] = boundary_pack_cs.lane_pack_ar_[lane_idx - 1];
      t_suceess_b = bc::true_v;
    }
  }
  else if (lane_idx >= kHostLane)
  {
    if (lane_idx - 1 >= 0 && boundary_pack_cs.lane_pack_ar_[lane_idx - 1].valid_b_)
    {
      boundary_pack_cs.lane_pack_ar_[lane_idx] = boundary_pack_cs.lane_pack_ar_[lane_idx - 1];
      t_suceess_b = bc::true_v;
    }
    else if (lane_idx + 1 < kMaxLaneNum && boundary_pack_cs.lane_pack_ar_[lane_idx + 1].valid_b_)
    {
      boundary_pack_cs.lane_pack_ar_[lane_idx] = boundary_pack_cs.lane_pack_ar_[lane_idx + 1];
      t_suceess_b = bc::true_v;
    }
  }

  /** 1. Set lane boundary type is virtual for all sources.*/
  /** Dont care whether this lane element boudary is valid, bacause copy directly.*/
  if (t_suceess_b)
  {
    for (bc::uint8_t t_element_u8 = 0; t_element_u8 < kMaxElemNumInOneLane; ++t_element_u8)
    {
      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].left_pack_cs_.per_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;
      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].right_pack_cs_.per_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;

      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].left_pack_cs_.map_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;
      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].right_pack_cs_.map_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;

      boundary_pack_cs.lane_pack_ar_[lane_idx]
          .element_pack_ar_[t_element_u8]
          .left_pack_cs_.ldveh_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;
      boundary_pack_cs.lane_pack_ar_[lane_idx]
          .element_pack_ar_[t_element_u8]
          .right_pack_cs_.ldveh_cs_.boundary_type_en_ = LaneBoundaryType::kVirtual;

      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].left_pack_cs_.ref_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;
      boundary_pack_cs.lane_pack_ar_[lane_idx].element_pack_ar_[t_element_u8].right_pack_cs_.ref_cs_.boundary_type_en_ =
          LaneBoundaryType::kVirtual;
    }
  }
}

void Preprocess::TransformMapdata(const bc::float32_t time_cycle_f, EmAdapterHDmapData& map_em_data)
{
  /** map invalid or coeff invalid
   * if coeff invalid and map valid , hlmf will use raw map data */
  if (!map_em_data.lanes_[2].lane_elements_[0].reference_line_.available_ ||
      !map_em_data.lanes_[2].lane_elements_[0].reference_line_.coeff_.valid_)
  {
    filtered_delta_heading_ = 0;
    filtered_delta_offset_ = 0;
  }
  else
  {
    bc::float32_t t_cur_delta_heading_f = 0;
    bc::float32_t t_cur_delta_offset_f = 0;
    const DiagStatus& t_lane_diag_en =
        input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.GetStatus();  ///< Perception lane status
    if (t_lane_diag_en != DiagStatus::kNormal)
    {
      t_cur_delta_heading_f = 0;
      t_cur_delta_offset_f = 0;
    }
    else
    {
      bc::bool_t t_l_val_b = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.valid_b_;
      bc::bool_t t_r_val_b = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.valid_b_;
      if (!t_l_val_b || !t_r_val_b)
      {
        t_cur_delta_heading_f = 0;
        t_cur_delta_offset_f = 0;
      }
      else
      {
        LaneBoundary map_left = map_em_data.lanes_[2].lane_elements_[0].left_boundary_;
        LaneBoundary map_right = map_em_data.lanes_[2].lane_elements_[0].right_boundary_;
        bc::uint8_t t_l_map_ego_idx_u8 = map_left.ego_point_idx_;
        bc::uint8_t t_r_map_ego_idx_u8 = map_right.ego_point_idx_;
        bc::float32_t t_l_map_c0_f = map_left.pts_[t_l_map_ego_idx_u8].y;
        bc::float32_t t_r_map_c0_f = map_right.pts_[t_r_map_ego_idx_u8].y;
        bc::float32_t t_l_map_length_f = map_left.pts_[map_left.start_idx_ + map_left.total_valid_number_ - 1].x -
                                         map_left.pts_[map_left.start_idx_].x;
        bc::float32_t t_r_map_length_f = map_right.pts_[map_right.start_idx_ + map_right.total_valid_number_ - 1].x -
                                         map_right.pts_[map_right.start_idx_].x;
        bc::float32_t t_l_map_c1_f, t_r_map_c1_f;
        CalcC1(map_left, t_l_map_ego_idx_u8, t_l_map_c1_f);
        CalcC1(map_right, t_r_map_ego_idx_u8, t_r_map_c1_f);
        bc::float32_t t_l_per_c0_f = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.coeff_cs_.c0_position;
        bc::float32_t t_r_per_c0_f = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.coeff_cs_.c0_position;
        bc::float32_t t_l_per_c1_f = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.coeff_cs_.c1_heading_angle;
        bc::float32_t t_r_per_c1_f =
            per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.coeff_cs_.c1_heading_angle;
        bc::float32_t t_l_per_length_f = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.dx_end_f_ -
                                         per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.dx_start_f_;
        bc::float32_t t_r_per_length_f = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.dx_end_f_ -
                                         per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.dx_start_f_;
        bc::float32_t t_c1_var_f = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.std_c1_f_ +
                                   per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.std_c1_f_;
        bc::float32_t t_c1_ratio_f = 0.5;
        if (t_c1_var_f > FLOAT32_ZERO)
        {
          t_c1_ratio_f = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.std_c1_f_ / t_c1_var_f;
        }
        bc::float32_t t_aver_c0_f = 0.5 * (t_l_per_c0_f + t_r_per_c0_f);
        // bc::float32_t t_aver_c1_f = t_c1_ratio_f * t_l_per_c1_f + (1 - t_c1_ratio_f) * t_r_per_c1_f;
        bc::float32_t t_aver_c1_f =
            0.5 * (t_l_per_length_f * 2.f / (t_l_per_length_f * 2.f + t_l_map_length_f) * t_l_per_c1_f +
                   t_l_map_length_f / (t_l_per_length_f * 2.f + t_l_map_length_f) * t_l_map_c1_f) +
            0.5 * (t_r_per_length_f * 2.f / (t_r_per_length_f * 2.f + t_r_map_length_f) * t_r_per_c1_f +
                   t_r_map_length_f / (t_r_per_length_f * 2.f + t_r_map_length_f) * t_r_map_c1_f);
        t_cur_delta_heading_f = t_aver_c1_f - 0.5 * (t_l_map_c1_f + t_r_map_c1_f);
        t_cur_delta_offset_f = t_aver_c0_f - 0.5 * (t_l_map_c0_f + t_r_map_c0_f);
        // t_cur_delta_heading_f =
        //     t_aver_c1_f - map_em_data.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c1_heading_angle;
        // t_cur_delta_offset_f =
        //     t_aver_c0_f - map_em_data.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c0_position;
      }
    }
    filtered_delta_heading_ = t_cur_delta_heading_f;
    filtered_delta_offset_ = t_cur_delta_offset_f;
    // filtered_delta_heading_ = LowPass(t_cur_delta_heading_f, filtered_delta_heading_, 0.5, time_cycle_f);
    // filtered_delta_offset_ = LowPass(t_cur_delta_offset_f, filtered_delta_offset_, 0.5, time_cycle_f);
    for (bc::uint8_t lane_idx = 0; lane_idx < kMaxLaneNum; ++lane_idx)
    {
      for (bc::uint8_t ele_idx = 0; ele_idx < kMaxElemNumInOneLane; ++ele_idx)
      {
        LaneBoundary& left = map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].left_boundary_;
        LaneBoundary& right = map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].right_boundary_;
        ReferenceLine& ref = map_em_data.lanes_[lane_idx].lane_elements_[ele_idx].reference_line_;
        for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; ++t_pt_idx_u8)
        {
          bc::float32_t temp_x_f = left.pts_[t_pt_idx_u8].x;
          bc::float32_t temp_y_f = left.pts_[t_pt_idx_u8].y + filtered_delta_offset_;
          left.pts_[t_pt_idx_u8].x =
              cosf(filtered_delta_heading_) * temp_x_f - sinf(filtered_delta_heading_) * temp_y_f;
          left.pts_[t_pt_idx_u8].y =
              sinf(filtered_delta_heading_) * temp_x_f + cosf(filtered_delta_heading_) * temp_y_f;
        }
        for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; ++t_pt_idx_u8)
        {
          bc::float32_t temp_x_f = right.pts_[t_pt_idx_u8].x;
          bc::float32_t temp_y_f = right.pts_[t_pt_idx_u8].y + filtered_delta_offset_;
          right.pts_[t_pt_idx_u8].x =
              cosf(filtered_delta_heading_) * temp_x_f - sinf(filtered_delta_heading_) * temp_y_f;
          right.pts_[t_pt_idx_u8].y =
              sinf(filtered_delta_heading_) * temp_x_f + cosf(filtered_delta_heading_) * temp_y_f;
        }
        for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint; ++t_pt_idx_u8)
        {
          bc::float32_t temp_x_f = ref.ref_line_pts_[t_pt_idx_u8].pos_.x;
          bc::float32_t temp_y_f = ref.ref_line_pts_[t_pt_idx_u8].pos_.y + filtered_delta_offset_;
          ref.ref_line_pts_[t_pt_idx_u8].pos_.x =
              cosf(filtered_delta_heading_) * temp_x_f - sinf(filtered_delta_heading_) * temp_y_f;
          ref.ref_line_pts_[t_pt_idx_u8].pos_.y =
              sinf(filtered_delta_heading_) * temp_x_f + cosf(filtered_delta_heading_) * temp_y_f;
          ref.ref_line_pts_[t_pt_idx_u8].heading_ += filtered_delta_heading_;
        }
      }
    }
  }
  temp_map_adapter_ = map_em_data;
}

void Preprocess::CalcC1(const LaneBoundary& boundary, const bc::uint8_t& ego_idx, bc::float32_t& t_c1)
{
  if (boundary.start_idx_ < ego_idx && (boundary.start_idx_ + boundary.total_valid_number_) > ego_idx)
  {
    if (boundary.pts_[ego_idx + 1].x > boundary.pts_[ego_idx - 1].x)
    {
      t_c1 = (boundary.pts_[ego_idx + 1].y - boundary.pts_[ego_idx - 1].y) /
             (boundary.pts_[ego_idx + 1].x - boundary.pts_[ego_idx - 1].x);
    }
    else
    {
#ifdef STD_COUT_ENABLE
      std::cout << "X_ref ERROR!!!!" << std::endl;
#endif
    }
  }
  else if (boundary.start_idx_ == ego_idx && (boundary.start_idx_ + boundary.total_valid_number_) > (ego_idx + 1))
  {
    if (boundary.pts_[ego_idx + 2].x > boundary.pts_[ego_idx].x)
    {
      t_c1 = (boundary.pts_[ego_idx + 2].y - boundary.pts_[ego_idx].y) /
             (boundary.pts_[ego_idx + 2].x - boundary.pts_[ego_idx].x);
    }
    else
    {
#ifdef STD_COUT_ENABLE
      std::cout << "X_ref ERROR!!!!" << std::endl;
#endif
    }
  }
  else if ((boundary.start_idx_ == ego_idx && (boundary.start_idx_ + boundary.total_valid_number_) == (ego_idx + 1)))
  {
    if (boundary.pts_[ego_idx + 1].x > boundary.pts_[ego_idx].x)
    {
      t_c1 = (boundary.pts_[ego_idx + 1].y - boundary.pts_[ego_idx].y) /
             (boundary.pts_[ego_idx + 1].x - boundary.pts_[ego_idx].x);
    }
    else
    {
#ifdef STD_COUT_ENABLE
      std::cout << "X_ref ERROR!!!!" << std::endl;
#endif
    }
  }
}

void Preprocess::TimeSyncMap(const LaneMarkings& per_lane_cs, const EgoMotionData& ego_motion_data_st,
                             EmAdapterHDmapData& map_em_data)
{
  bc::float64_t t_delta_timestamp_f = per_lane_cs.time_stamp - map_em_data.map_car_position_.timestamp_;
  if (fabsf(t_delta_timestamp_f) >= kMaxDeltaTimeStamp)
  {
    return;
  }
  if (fabsf(t_delta_timestamp_f) >= kWarningDeltaTimeStamp)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "Warning: Big Time Stamp Gap between Per & Map!!!!" << std::endl;
#endif
  }
  if (input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      input_diag_manager_ar_[kInputChkEgoMotion].status_man_cs_.GetStatus() != DiagStatus::kErrCfm &&
      input_diag_manager_ar_[kInputChkPerLane].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    time_sync_cs_.SetDeltaTime(t_delta_timestamp_f);
    time_sync_cs_.CalcEgoTransform(ego_motion_data_st.twist.linear.x, ego_motion_data_st.kappa);
    for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < kMaxLaneNum; ++t_laneidx_u8)
    {
      if (map_em_data.lanes_[t_laneidx_u8].lane_valid_ == bc::false_v)
      {
        /** Do nothing*/
      }
      else
      {
        for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < kMaxElemNumInOneLane; ++t_elementidx_u8)
        {
          LaneElement& t_map_lane_element_cs = map_em_data.lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
          if (t_map_lane_element_cs.element_valid_ == bc::false_v)
          {
            /** Do nothing*/
          }
          else
          {
            LaneBoundary& t_left_boundary_cs = t_map_lane_element_cs.left_boundary_;
            LaneBoundary& t_right_boundary_cs = t_map_lane_element_cs.right_boundary_;
            if (t_left_boundary_cs.existence_)
            {
              for (bc::uint8_t t_pointidx_u8 = t_left_boundary_cs.start_idx_;
                   t_pointidx_u8 < t_left_boundary_cs.start_idx_ + t_left_boundary_cs.total_valid_number_;
                   ++t_pointidx_u8)
              {
                time_sync_cs_.SetTargetX(t_left_boundary_cs.pts_[t_pointidx_u8].x);
                time_sync_cs_.SetTargetY(t_left_boundary_cs.pts_[t_pointidx_u8].y);
                time_sync_cs_.TimeSyncMain(TargetMotionModel::kStatic);
                t_left_boundary_cs.pts_[t_pointidx_u8].x = time_sync_cs_.GetTargetX();
                t_left_boundary_cs.pts_[t_pointidx_u8].y = time_sync_cs_.GetTargetY();
              }
            }
            else
            {
              t_left_boundary_cs = LaneBoundary();
            }
            if (t_right_boundary_cs.existence_)
            {
              for (bc::uint8_t t_pointidx_u8 = t_right_boundary_cs.start_idx_;
                   t_pointidx_u8 < t_right_boundary_cs.start_idx_ + t_right_boundary_cs.total_valid_number_;
                   ++t_pointidx_u8)
              {
                time_sync_cs_.SetTargetX(t_right_boundary_cs.pts_[t_pointidx_u8].x);
                time_sync_cs_.SetTargetY(t_right_boundary_cs.pts_[t_pointidx_u8].y);
                time_sync_cs_.TimeSyncMain(TargetMotionModel::kStatic);
                t_right_boundary_cs.pts_[t_pointidx_u8].x = time_sync_cs_.GetTargetX();
                t_right_boundary_cs.pts_[t_pointidx_u8].y = time_sync_cs_.GetTargetY();
              }
            }
            else
            {
              t_right_boundary_cs = LaneBoundary();
            }
          }
        }
      }
    }
  }
  else
  {
    /** Do nothing*/
  }
}

bc::bool_t Preprocess::MapDataAdapter(const LaneMarkings& per_lane_cs, const bc::float32_t time_cycle_f,
                                      const EgoMotionData& ego_motion_data_st, EmAdapterHDmapData& map_em_data)
{
  /// TODO: adapter map source data
  /// HMap Data adapter
  // EhrDataHandle* pDataInstance = Singleton<EhrDataHandle>::GetInstance();
  ehrdatahandle_.getEhrToEmData(map_em_data);
  ehrdatahandle_.getLeftMergeEhrToEmData(map_em_data);
  ehrdatahandle_.getRightMergeEhrToEmData(map_em_data);
  map_dbg_data_.map_host_ele_[0] = map_em_data.lanes_[2].lane_elements_[0];
  TimeSyncMap(per_lane_cs, ego_motion_data_st, map_em_data);
  map_dbg_data_.map_host_ele_[1] = map_em_data.lanes_[2].lane_elements_[0];
  if (!map_em_data.lanes_[2].lane_elements_[0].reference_line_.available_)
  {
#ifdef STD_COUT_ENABLE
    std::cout << "ERROR in Preprocess::MapDataAdapter: Map host lane refline invalid!" << std::endl;
#endif
  }
  MapScenarioDistinction(map_em_data);
  // ehr_adapter_dbg_ = map_em_data;
  for (bc::uint8_t i = 0; i < kMaxLaneNum; i++)
  {
    for (bc::uint8_t j = 0; j < kMaxElemNumInOneLane; j++)
    {
      bc::bool_t t_lane_valid = map_em_data.lanes_[i].lane_valid_;
      bc::bool_t t_element_valid = map_em_data.lanes_[i].lane_elements_[j].element_valid_;

      if ((t_lane_valid == bc::true_v) && (t_element_valid == bc::true_v))
      {
        LaneElement& t_element = map_em_data.lanes_[i].lane_elements_[j];
        LaneElement t_element_new = t_element;

        // left boundary start
        bc::uint16_t t_idx_boundary_lower = kIdxPointInvalid;
        bc::uint16_t t_idx_boundary_upper = 0;
        // bc::uint16_t t_idx_edge_lower = kIdxPointInvalid;
        // bc::uint16_t t_idx_edge_upper = 0;
        bc::uint16_t t_idx_ref_lower = kIdxPointInvalid;
        bc::uint16_t t_idx_ref_upper = 0;
        bc::uint16_t t_idx_interp = 0;
        bc::uint16_t t_ego_point_idx_ = 0;
        bc::uint16_t t_mono_valid_num_ = 0;
        bc::bool_t t_boundary_valid = t_element.left_boundary_.existence_;
        // bc::bool_t t_edge_valid = t_element.left_road_edge_.existence_;
        bc::bool_t t_ref_valid = t_element.reference_line_.available_;
        // check map size
        bc::uint8_t t_map_end_idx = kMaxBoundaryPoint - 1;
        bc::uint8_t t_map_start_idx = 0;
        bc::uint8_t t_mono_start_idx_u8 = 0;
        bc::uint8_t t_mono_end_idx_u8 = 0;

        if (t_boundary_valid)
        {
          /// Find mono segs with ego point
          t_mono_start_idx_u8 = t_element.left_boundary_.start_idx_;
          t_mono_end_idx_u8 = t_element.left_boundary_.start_idx_ + t_element.left_boundary_.total_valid_number_ - 1;
          if (t_element.left_boundary_.ego_point_idx_ != kMaxBoundaryPoint)
          {
            for (bc::uint8_t t_pt_idx_u8 = t_element.left_boundary_.ego_point_idx_;
                 t_pt_idx_u8 < t_element.left_boundary_.start_idx_ + t_element.left_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.left_boundary_.pts_[t_pt_idx_u8 + 1].x > t_element.left_boundary_.pts_[t_pt_idx_u8].x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
            for (bc::int8_t t_pt_idx_s8 = t_element.left_boundary_.ego_point_idx_ - 1; t_pt_idx_s8 >= 0; t_pt_idx_s8--)
            {
              if (t_element.left_boundary_.pts_[t_pt_idx_s8 + 1].x > t_element.left_boundary_.pts_[t_pt_idx_s8].x)
              {
                continue;
              }
              else
              {
                t_mono_start_idx_u8 = t_pt_idx_s8 + 1;
                break;
              }
            }
          }
          else
          {
            bc::float32_t t_min_x_f = t_element.left_boundary_.pts_[t_element.left_boundary_.start_idx_].x;
            for (bc::uint8_t t_pt_idx_u8 = t_element.left_boundary_.start_idx_ + 1;
                 t_pt_idx_u8 < t_element.left_boundary_.start_idx_ + t_element.left_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.left_boundary_.pts_[t_pt_idx_u8].x < t_min_x_f)
              {
                t_min_x_f = t_element.left_boundary_.pts_[t_pt_idx_u8].x;
                t_mono_start_idx_u8 = t_pt_idx_u8;
              }
            }
            for (bc::uint8_t t_pt_idx_u8 = t_mono_start_idx_u8;
                 t_pt_idx_u8 < t_element.left_boundary_.start_idx_ + t_element.left_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.left_boundary_.pts_[t_pt_idx_u8 + 1].x > t_element.left_boundary_.pts_[t_pt_idx_u8].x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
          }
          t_mono_valid_num_ = t_mono_end_idx_u8 - t_mono_start_idx_u8 + 1;
          if (t_mono_valid_num_ < kMaxBoundaryPoint)
          // && t_element.left_boundary_.pts_[t_mono_valid_num_ - 1].x < x_ref_ar_[kMaxBoundaryPoint - 1])
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
            {
              /// find map start idx
              /// because end idx bigger than start idx, so start idx does not need break
              if (t_pt_idx_u8 == 0 && t_element.left_boundary_.pts_[t_mono_start_idx_u8].x > x_ref_ar_[t_pt_idx_u8])
              {
                t_map_start_idx = t_pt_idx_u8;
              }
              else if (t_element.left_boundary_.pts_[t_mono_start_idx_u8].x > x_ref_ar_[t_pt_idx_u8] &&
                       t_element.left_boundary_.pts_[t_mono_start_idx_u8].x < x_ref_ar_[t_pt_idx_u8 + 1])
              {
                t_map_start_idx = t_pt_idx_u8 + 1;
              }
              else if (t_element.left_boundary_.pts_[t_mono_end_idx_u8].x > x_ref_ar_[t_pt_idx_u8] &&
                       (t_element.left_boundary_.pts_[t_mono_end_idx_u8].x < x_ref_ar_[t_pt_idx_u8 + 1] ||
                        fabsf(t_element.left_boundary_.pts_[t_mono_end_idx_u8].x - x_ref_ar_[t_pt_idx_u8]) < 0.1))
              {
                t_map_end_idx = t_pt_idx_u8;
                break;
              }
              else
              {
                continue;
              }
            }
          }
        }

        else if (t_ref_valid)
        {
          t_mono_start_idx_u8 = t_element.reference_line_.ref_start_idx_;
          t_mono_end_idx_u8 =
              t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_ - 1;
          if (t_element.reference_line_.current_point_idx_ != kMaxBoundaryPoint)
          {
            for (bc::uint8_t t_pt_idx_u8 = t_element.reference_line_.current_point_idx_;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
            for (bc::int8_t t_pt_idx_s8 = t_element.reference_line_.current_point_idx_ - 1; t_pt_idx_s8 >= 0;
                 t_pt_idx_s8--)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_s8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_s8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_start_idx_u8 = t_pt_idx_s8 + 1;
                break;
              }
            }
          }
          else
          {
            bc::float32_t t_min_x_f =
                t_element.reference_line_.ref_line_pts_[t_element.reference_line_.ref_start_idx_].pos_.x;
            for (bc::uint8_t t_pt_idx_u8 = t_element.reference_line_.ref_start_idx_ + 1;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x < t_min_x_f)
              {
                t_min_x_f = t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x;
                t_mono_start_idx_u8 = t_pt_idx_u8;
              }
            }
            for (bc::uint8_t t_pt_idx_u8 = t_mono_start_idx_u8;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
          }
          t_mono_valid_num_ = t_mono_end_idx_u8 - t_mono_start_idx_u8 + 1;
          if (t_mono_valid_num_ < kMaxRefLinePtsNum)
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
            {
              /// find map start idx
              /// because end idx bigger than start idx, so start idx does not need break
              if (t_element.reference_line_.ref_line_pts_[t_mono_start_idx_u8].pos_.x > x_ref_ar_[t_pt_idx_u8] &&
                  t_element.reference_line_.ref_line_pts_[t_mono_start_idx_u8].pos_.x < x_ref_ar_[t_pt_idx_u8 + 1])
              {
                t_map_start_idx = t_pt_idx_u8 + 1;
              }
              else if (t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x > x_ref_ar_[t_pt_idx_u8] &&
                       (t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x <
                            x_ref_ar_[t_pt_idx_u8 + 1] ||
                        fabsf(t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x -
                              x_ref_ar_[t_pt_idx_u8]) < 0.1))
              {
                t_map_end_idx = t_pt_idx_u8;
                break;
              }
              else
              {
                continue;
              }
            }
          }
        }
        // check map size end
        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
        {
          bc::float32_t t_x = x_ref_ar_[t_idx_u8];
          if (t_boundary_valid == bc::true_v)
          {
            for (bc::uint16_t t_idx = t_mono_start_idx_u8; t_idx < t_mono_valid_num_; t_idx++)
            {
              if (t_element.left_boundary_.pts_[t_idx].x > t_x)
              {
                t_idx_boundary_upper = t_idx;
                break;
              }
              else if (t_idx == t_mono_valid_num_ - 1)
              {
                /* last point is still behind t_x, set the last point as lower
                 * point, and upper point invalid */
                t_idx_boundary_upper = t_idx;
                break;
              }
              else
              {
                /* do nothing, check next point */
              }
            }
            if (t_idx_boundary_upper == 0)
            {
              t_idx_boundary_lower = kIdxPointInvalid;
            }
            else if (t_idx_boundary_upper != kIdxPointInvalid)
            {
              t_idx_boundary_lower = t_idx_boundary_upper - 1;
            }
            else
            {
              /* do nothing */
            }
          }
          else
          {
            t_idx_boundary_lower = kIdxPointInvalid;
            t_idx_boundary_upper = kIdxPointInvalid;
          }

          if (t_ref_valid == bc::true_v)
          {
            for (bc::uint16_t t_idx = t_mono_start_idx_u8; t_idx < t_mono_valid_num_; t_idx++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_idx].pos_.x > t_x)
              {
                t_idx_ref_upper = t_idx;
                break;
              }
              else if (t_idx == t_mono_valid_num_ - 1)
              {
                /* last point is still behind t_x, set the last point as lower
                 * point, and upper point invalid */
                t_idx_ref_upper = t_idx;
                break;
              }
              else
              {
                /* do nothing, check next point */
              }
            }
            if (t_idx_ref_upper == 0)
            {
              t_idx_ref_lower = kIdxPointInvalid;
            }
            else if (t_idx_ref_upper != kIdxPointInvalid)
            {
              t_idx_ref_lower = t_idx_ref_upper - 1;
            }
            else
            {
              /* do nothing */
            }
          }
          else
          {
            t_idx_ref_lower = kIdxPointInvalid;
            t_idx_ref_upper = kIdxPointInvalid;
          }

          /// lower point
          Point3D t_point_lower;
          t_point_lower.x = -65535;
          t_point_lower.y = 0;
          t_point_lower.z = -1;  // z as validity
          bc::bool_t t_lower_point_found = bc::false_v;

          if (t_idx_boundary_lower != kIdxPointInvalid)
          {
            if (bc::abs(t_element.left_boundary_.pts_[t_idx_boundary_lower].x - t_x) < 5 ||
                (bc::abs(t_element.left_boundary_.pts_[t_idx_boundary_lower].x - t_x) >= 5 &&
                 t_element.left_boundary_.existence_))
            {
              t_lower_point_found = bc::true_v;
            }

            t_point_lower.x = t_element.left_boundary_.pts_[t_idx_boundary_lower].x;
            t_point_lower.y = t_element.left_boundary_.pts_[t_idx_boundary_lower].y;
            t_point_lower.z = 1;
          }

          if (t_lower_point_found == bc::false_v)
          {
            if (t_idx_ref_lower != kIdxPointInvalid)
            {
              bc::float32_t t_x_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].pos_.x;
              bc::float32_t t_y_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].pos_.y;
              bc::float32_t t_heading_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].heading_;
              bc::float32_t t_half_width = 0.5 * t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].lane_width_;
              t_half_width = bc::min(t_half_width, 4.f / 2.f);
              t_half_width = bc::max(t_half_width, 2.8f / 2.f);

              t_x_ref = t_x_ref - t_half_width * std::sin(t_heading_ref);
              t_y_ref = t_y_ref + t_half_width * std::cos(t_heading_ref);
              if (t_x_ref > t_point_lower.x)
              {
                t_point_lower.x = t_x_ref;
                t_point_lower.y = t_y_ref;
                t_point_lower.z = 1;
              }
            }
          }

          /// upper point
          Point3D t_point_upper;
          t_point_upper.x = 65535;
          t_point_upper.y = 0;
          t_point_upper.z = -1;
          bc::bool_t t_upper_point_found = bc::false_v;

          if (t_idx_boundary_upper != kIdxPointInvalid)
          {
            if (bc::abs(t_element.left_boundary_.pts_[t_idx_boundary_upper].x - t_x) < 5 ||
                (bc::abs(t_element.left_boundary_.pts_[t_idx_boundary_upper].x - t_x) >= 5 &&
                 t_element.left_boundary_.existence_))
            {
              t_upper_point_found = bc::true_v;
            }

            t_point_upper.x = t_element.left_boundary_.pts_[t_idx_boundary_upper].x;
            t_point_upper.y = t_element.left_boundary_.pts_[t_idx_boundary_upper].y;
            t_point_upper.z = 1;
          }

          if (t_upper_point_found == bc::false_v)
          {
            if (t_idx_ref_upper != kIdxPointInvalid)
            {
              bc::float32_t t_x_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].pos_.x;
              bc::float32_t t_y_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].pos_.y;
              bc::float32_t t_heading_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].heading_;
              bc::float32_t t_half_width = 0.5 * t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].lane_width_;
              t_half_width = bc::min(t_half_width, 4.f / 2.f);
              t_half_width = bc::max(t_half_width, 2.8f / 2.f);
              t_x_ref = t_x_ref - t_half_width * std::sin(t_heading_ref);
              t_y_ref = t_y_ref + t_half_width * std::cos(t_heading_ref);
              if (t_x_ref < t_point_upper.x)
              {
                t_point_upper.x = t_x_ref;
                t_point_upper.y = t_y_ref;
                t_point_upper.z = 1;
              }
            }
          }

          // interpolation
          bc::float32_t t_y_interp = 0;
          bc::float32_t t_z_interp = 0;
          if ((t_point_lower.z > 0) && (t_point_upper.z > 0))
          {
            if (t_point_lower.x > t_point_upper.x)
            {
              Point3D t_point_swap = t_point_upper;
              t_point_upper = t_point_lower;
              t_point_lower = t_point_swap;
            }
            if (t_x > t_point_upper.x)
            {
              t_y_interp = LinearInterpolation(t_point_lower.x, t_point_lower.y, t_point_upper.x, t_point_upper.y, t_x,
                                               Lin_Interp_Method::kExtrapolate);
              t_z_interp = 0.f;
            }
            else
            {
              t_y_interp = LinearInterpolation(t_point_lower.x, t_point_lower.y, t_point_upper.x, t_point_upper.y, t_x);
              t_z_interp = bc::min(bc::abs(t_x - t_point_lower.x), bc::abs(t_x - t_point_upper.x));
              t_z_interp = bc::exp(-t_z_interp);
            }
          }
          else if (t_point_lower.z > 0)
          {
            t_y_interp = t_point_lower.y;
          }
          else if (t_point_upper.z > 0)
          {
            t_y_interp = t_point_upper.y;
          }
          else
          {
            t_y_interp = 0;
          }
          t_element_new.left_boundary_.pts_[t_idx_interp].x = t_x;
          t_element_new.left_boundary_.pts_[t_idx_interp].y = t_y_interp;
          t_element_new.left_boundary_.pts_[t_idx_interp].z = t_z_interp;

          if ((t_x > -0.5) && (t_x < 0.5))
          {
            t_ego_point_idx_ = t_idx_interp;
          }
          t_idx_interp++;
        }
        t_element_new.left_boundary_.ego_point_idx_ = t_ego_point_idx_;
        t_element_new.left_boundary_.total_valid_number_ = t_map_end_idx - t_map_start_idx + 1;
        t_element_new.left_boundary_.start_idx_ = t_map_start_idx;
        t_element_new.left_boundary_.reserve_[0] = t_map_start_idx;
        t_element_new.left_boundary_.existence_ = bc::true_v;

        // left boundary end

        // right boundary start
        t_idx_boundary_lower = kIdxPointInvalid;
        t_idx_boundary_upper = 0;
        // t_idx_edge_lower = kIdxPointInvalid;
        // t_idx_edge_upper = 0;
        t_idx_ref_lower = kIdxPointInvalid;
        t_idx_ref_upper = 0;
        t_idx_interp = 0;
        t_ego_point_idx_ = 0;
        t_mono_valid_num_ = 0;
        t_boundary_valid = t_element.right_boundary_.existence_;
        // t_edge_valid = t_element.right_road_edge_.existence_;
        t_ref_valid = t_element.reference_line_.available_;
        // check map size
        t_map_end_idx = kMaxBoundaryPoint - 1;
        t_map_start_idx = 0;
        t_mono_start_idx_u8 = 0;
        t_mono_end_idx_u8 = 0;
        if (t_boundary_valid)
        {
          t_mono_start_idx_u8 = t_element.right_boundary_.start_idx_;
          t_mono_end_idx_u8 = t_element.right_boundary_.start_idx_ + t_element.right_boundary_.total_valid_number_ - 1;
          if (t_element.right_boundary_.ego_point_idx_ != kMaxBoundaryPoint)
          {
            for (bc::uint8_t t_pt_idx_u8 = t_element.right_boundary_.ego_point_idx_;
                 t_pt_idx_u8 < t_element.right_boundary_.start_idx_ + t_element.right_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.right_boundary_.pts_[t_pt_idx_u8 + 1].x > t_element.right_boundary_.pts_[t_pt_idx_u8].x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
            for (bc::int8_t t_pt_idx_s8 = t_element.right_boundary_.ego_point_idx_ - 1; t_pt_idx_s8 >= 0; t_pt_idx_s8--)
            {
              if (t_element.right_boundary_.pts_[t_pt_idx_s8 + 1].x > t_element.right_boundary_.pts_[t_pt_idx_s8].x)
              {
                continue;
              }
              else
              {
                t_mono_start_idx_u8 = t_pt_idx_s8 + 1;
                break;
              }
            }
          }
          else
          {
            bc::float32_t t_min_x_f = t_element.right_boundary_.pts_[t_element.right_boundary_.start_idx_].x;
            for (bc::uint8_t t_pt_idx_u8 = t_element.right_boundary_.start_idx_ + 1;
                 t_pt_idx_u8 < t_element.right_boundary_.start_idx_ + t_element.right_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.right_boundary_.pts_[t_pt_idx_u8].x < t_min_x_f)
              {
                t_min_x_f = t_element.right_boundary_.pts_[t_pt_idx_u8].x;
                t_mono_start_idx_u8 = t_pt_idx_u8;
              }
            }
            for (bc::uint8_t t_pt_idx_u8 = t_mono_start_idx_u8;
                 t_pt_idx_u8 < t_element.right_boundary_.start_idx_ + t_element.right_boundary_.total_valid_number_;
                 t_pt_idx_u8++)
            {
              if (t_element.right_boundary_.pts_[t_pt_idx_u8 + 1].x > t_element.right_boundary_.pts_[t_pt_idx_u8].x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
          }
          t_mono_valid_num_ = t_mono_end_idx_u8 - t_mono_start_idx_u8 + 1;
          if (t_mono_valid_num_ < kMaxBoundaryPoint)
          // && t_element.right_boundary_.pts_[t_mono_valid_num_ - 1].x < x_ref_ar_[kMaxBoundaryPoint - 1])
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
            {
              /// find map start idx
              /// because end idx bigger than start idx, so start idx does not need break
              if (t_element.right_boundary_.pts_[t_mono_start_idx_u8].x > x_ref_ar_[t_pt_idx_u8] &&
                  t_element.right_boundary_.pts_[t_mono_start_idx_u8].x < x_ref_ar_[t_pt_idx_u8 + 1])
              {
                t_map_start_idx = t_pt_idx_u8 + 1;
              }
              else if (t_element.right_boundary_.pts_[t_mono_end_idx_u8].x > x_ref_ar_[t_pt_idx_u8] &&
                       (t_element.right_boundary_.pts_[t_mono_end_idx_u8].x < x_ref_ar_[t_pt_idx_u8 + 1] ||
                        fabsf(t_element.right_boundary_.pts_[t_mono_end_idx_u8].x - x_ref_ar_[t_pt_idx_u8]) < 0.1))
              {
                t_map_end_idx = t_pt_idx_u8;
                break;
              }
              else
              {
                continue;
              }
            }
          }
        }

        else if (t_ref_valid)
        {
          t_mono_start_idx_u8 = t_element.reference_line_.ref_start_idx_;
          t_mono_end_idx_u8 =
              t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_ - 1;
          if (t_element.reference_line_.current_point_idx_ != kMaxBoundaryPoint)
          {
            for (bc::uint8_t t_pt_idx_u8 = t_element.reference_line_.current_point_idx_;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
            for (bc::int8_t t_pt_idx_s8 = t_element.reference_line_.current_point_idx_ - 1; t_pt_idx_s8 >= 0;
                 t_pt_idx_s8--)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_s8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_s8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_start_idx_u8 = t_pt_idx_s8 + 1;
                break;
              }
            }
          }
          else
          {
            bc::float32_t t_min_x_f =
                t_element.reference_line_.ref_line_pts_[t_element.reference_line_.ref_start_idx_].pos_.x;
            for (bc::uint8_t t_pt_idx_u8 = t_element.reference_line_.ref_start_idx_ + 1;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x < t_min_x_f)
              {
                t_min_x_f = t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x;
                t_mono_start_idx_u8 = t_pt_idx_u8;
              }
            }
            for (bc::uint8_t t_pt_idx_u8 = t_mono_start_idx_u8;
                 t_pt_idx_u8 < t_element.reference_line_.ref_start_idx_ + t_element.reference_line_.ref_line_valid_pts_;
                 t_pt_idx_u8++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_pt_idx_u8 + 1].pos_.x >
                  t_element.reference_line_.ref_line_pts_[t_pt_idx_u8].pos_.x)
              {
                continue;
              }
              else
              {
                t_mono_end_idx_u8 = t_pt_idx_u8;
                break;
              }
            }
          }
          t_mono_valid_num_ = t_mono_end_idx_u8 - t_mono_start_idx_u8 + 1;
          if (t_mono_valid_num_ < kMaxRefLinePtsNum)
          {
            for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kMaxBoundaryPoint - 1; t_pt_idx_u8++)
            {
              /// find map start idx
              /// because end idx bigger than start idx, so start idx does not need break
              if (t_element.reference_line_.ref_line_pts_[t_mono_start_idx_u8].pos_.x > x_ref_ar_[t_pt_idx_u8] &&
                  t_element.reference_line_.ref_line_pts_[t_mono_start_idx_u8].pos_.x < x_ref_ar_[t_pt_idx_u8 + 1])
              {
                t_map_start_idx = t_pt_idx_u8 + 1;
              }
              else if (t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x > x_ref_ar_[t_pt_idx_u8] &&
                       (t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x <
                            x_ref_ar_[t_pt_idx_u8 + 1] ||
                        fabsf(t_element.reference_line_.ref_line_pts_[t_mono_end_idx_u8].pos_.x -
                              x_ref_ar_[t_pt_idx_u8]) < 0.1))
              {
                t_map_end_idx = t_pt_idx_u8;
                break;
              }
              else
              {
                continue;
              }
            }
          }
        }
        // check map size end

        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; t_idx_u8++)
        {
          bc::float32_t t_x = x_ref_ar_[t_idx_u8];
          if (t_boundary_valid == bc::true_v)
          {
            for (bc::uint16_t t_idx = t_mono_start_idx_u8; t_idx < t_mono_valid_num_; t_idx++)
            {
              if (t_element.right_boundary_.pts_[t_idx].x > t_x)
              {
                t_idx_boundary_upper = t_idx;
                break;
              }
              else if (t_idx == t_mono_valid_num_ - 1)
              {
                /* last point is still behind t_x, set the last point as lower
                 * point, and upper point invalid */
                t_idx_boundary_upper = t_idx;
                break;
              }
              else
              {
                /* do nothing, check next point */
              }
            }
            if (t_idx_boundary_upper == 0)
            {
              t_idx_boundary_lower = kIdxPointInvalid;
            }
            else if (t_idx_boundary_upper != kIdxPointInvalid)
            {
              t_idx_boundary_lower = t_idx_boundary_upper - 1;
            }
            else
            {
              /* do nothing */
            }
          }
          else
          {
            t_idx_boundary_lower = kIdxPointInvalid;
            t_idx_boundary_upper = kIdxPointInvalid;
          }

          // if (t_edge_valid == bc::true_v)
          // {
          //   for (bc::uint16_t t_idx = t_mono_start_idx_u8; t_idx < t_mono_valid_num_; t_idx++)
          //   {
          //     if (t_element.right_road_edge_.pts_[t_idx].x > t_x)
          //     {
          //       t_idx_edge_upper = t_idx;
          //       break;
          //     }
          //     else if (t_idx == t_mono_valid_num_ - 1)
          //     {
          //       /* last point is still behind t_x, set the last point as lower
          //        * point, and upper point invalid */
          //       t_idx_edge_upper = t_idx;
          //       break;
          //     }
          //     else
          //     {
          //       /* do nothing, check next point */
          //     }
          //   }
          //   if (t_idx_edge_upper == 0)
          //   {
          //     t_idx_edge_lower = kIdxPointInvalid;
          //   }
          //   else if (t_idx_edge_upper != kIdxPointInvalid)
          //   {
          //     t_idx_edge_lower = t_idx_edge_upper - 1;
          //   }
          //   else
          //   {
          //     /* do nothing */
          //   }
          // }
          // else
          // {
          //   t_idx_edge_lower = kIdxPointInvalid;
          //   t_idx_edge_upper = kIdxPointInvalid;
          // }

          if (t_ref_valid == bc::true_v)
          {
            for (bc::uint16_t t_idx = t_mono_start_idx_u8; t_idx < t_mono_valid_num_; t_idx++)
            {
              if (t_element.reference_line_.ref_line_pts_[t_idx].pos_.x > t_x)
              {
                t_idx_ref_upper = t_idx;
                break;
              }
              else if (t_idx == t_mono_valid_num_ - 1)
              {
                /* last point is still behind t_x, set the last point as lower
                 * point, and upper point invalid */
                t_idx_ref_upper = t_idx;
                break;
              }
              else
              {
                /* do nothing, check next point */
              }
            }
            if (t_idx_ref_upper == 0)
            {
              t_idx_ref_lower = kIdxPointInvalid;
            }
            else if (t_idx_ref_upper != kIdxPointInvalid)
            {
              t_idx_ref_lower = t_idx_ref_upper - 1;
            }
            else
            {
              /* do nothing */
            }
          }
          else
          {
            t_idx_ref_lower = kIdxPointInvalid;
            t_idx_ref_upper = kIdxPointInvalid;
          }

          /// lower point
          Point3D t_point_lower;
          t_point_lower.x = -65535;
          t_point_lower.y = 0;
          t_point_lower.z = -1;  // z as validity
          bc::bool_t t_lower_point_found = bc::false_v;

          if (t_idx_boundary_lower != kIdxPointInvalid)
          {
            if (bc::abs(t_element.right_boundary_.pts_[t_idx_boundary_lower].x - t_x) < 5 ||
                (bc::abs(t_element.right_boundary_.pts_[t_idx_boundary_lower].x - t_x) >= 5 &&
                 t_element.right_boundary_.existence_))
            {
              t_lower_point_found = bc::true_v;
            }

            t_point_lower.x = t_element.right_boundary_.pts_[t_idx_boundary_lower].x;
            t_point_lower.y = t_element.right_boundary_.pts_[t_idx_boundary_lower].y;
            t_point_lower.z = 1;
          }
          // if (t_lower_point_found == bc::false_v)
          // {
          //   if (t_idx_edge_lower != kIdxPointInvalid)
          //   {
          //     if (bc::abs(t_element.right_road_edge_.pts_[t_idx_edge_lower].x - t_x) < 5)
          //     {
          //       t_lower_point_found = bc::true_v;
          //       t_point_lower.x = t_element.right_road_edge_.pts_[t_idx_edge_lower].x;
          //       t_point_lower.y = t_element.right_road_edge_.pts_[t_idx_edge_lower].y;
          //       t_point_lower.z = 1;
          //     }
          //     else
          //     {
          //       if (t_element.right_road_edge_.pts_[t_idx_edge_lower].x > t_point_lower.x)
          //       {
          //         t_point_lower.x = t_element.right_road_edge_.pts_[t_idx_edge_lower].x;
          //         t_point_lower.y = t_element.right_road_edge_.pts_[t_idx_edge_lower].y;
          //         t_point_lower.z = 1;
          //       }
          //     }
          //   }
          // }
          if (t_lower_point_found == bc::false_v)
          {
            if (t_idx_ref_lower != kIdxPointInvalid)
            {
              bc::float32_t t_x_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].pos_.x;
              bc::float32_t t_y_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].pos_.y;
              bc::float32_t t_heading_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].heading_;
              bc::float32_t t_half_width = 0.5 * t_element.reference_line_.ref_line_pts_[t_idx_ref_lower].lane_width_;
              t_half_width = bc::min(t_half_width, 4.f / 2.f);
              t_half_width = bc::max(t_half_width, 2.8f / 2.f);
              t_x_ref = t_x_ref + t_half_width * std::sin(t_heading_ref);
              t_y_ref = t_y_ref - t_half_width * std::cos(t_heading_ref);
              if (t_x_ref > t_point_lower.x)
              {
                t_point_lower.x = t_x_ref;
                t_point_lower.y = t_y_ref;
                t_point_lower.z = 1;
              }
            }
          }

          /// upper point
          Point3D t_point_upper;
          t_point_upper.x = 65535;
          t_point_upper.y = 0;
          t_point_upper.z = -1;
          bc::bool_t t_upper_point_found = bc::false_v;

          if (t_idx_boundary_upper != kIdxPointInvalid)
          {
            if (bc::abs(t_element.right_boundary_.pts_[t_idx_boundary_upper].x - t_x) < 5 ||
                (bc::abs(t_element.right_boundary_.pts_[t_idx_boundary_upper].x - t_x) >= 5 &&
                 t_element.right_boundary_.existence_))
            {
              t_upper_point_found = bc::true_v;
            }

            t_point_upper.x = t_element.right_boundary_.pts_[t_idx_boundary_upper].x;
            t_point_upper.y = t_element.right_boundary_.pts_[t_idx_boundary_upper].y;
            t_point_upper.z = 1;
          }
          // if (t_upper_point_found == bc::false_v)
          // {
          //   if (t_idx_edge_upper != kIdxPointInvalid)
          //   {
          //     if (bc::abs(t_element.right_road_edge_.pts_[t_idx_edge_upper].x - t_x) < 5)
          //     {
          //       t_upper_point_found = bc::true_v;
          //       t_point_upper.x = t_element.right_road_edge_.pts_[t_idx_edge_upper].x;
          //       t_point_upper.y = t_element.right_road_edge_.pts_[t_idx_edge_upper].y;
          //       t_point_upper.z = 1;
          //     }
          //     else
          //     {
          //       if (t_element.right_road_edge_.pts_[t_idx_edge_upper].x < t_point_upper.x)
          //       {
          //         t_point_upper.x = t_element.right_road_edge_.pts_[t_idx_edge_upper].x;
          //         t_point_upper.y = t_element.right_road_edge_.pts_[t_idx_edge_upper].y;
          //         t_point_upper.z = 1;
          //       }
          //     }
          //   }
          // }
          if (t_upper_point_found == bc::false_v)
          {
            if (t_idx_ref_upper != kIdxPointInvalid)
            {
              bc::float32_t t_x_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].pos_.x;
              bc::float32_t t_y_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].pos_.y;
              bc::float32_t t_heading_ref = t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].heading_;
              bc::float32_t t_half_width = 0.5 * t_element.reference_line_.ref_line_pts_[t_idx_ref_upper].lane_width_;
              t_half_width = bc::min(t_half_width, 4.f / 2.f);
              t_half_width = bc::max(t_half_width, 2.8f / 2.f);
              t_x_ref = t_x_ref + t_half_width * std::sin(t_heading_ref);
              t_y_ref = t_y_ref - t_half_width * std::cos(t_heading_ref);
              if (t_x_ref < t_point_upper.x)
              {
                t_point_upper.x = t_x_ref;
                t_point_upper.y = t_y_ref;
                t_point_upper.z = 1;
              }
            }
          }

          // interpolation
          bc::float32_t t_y_interp = 0;
          bc::float32_t t_z_interp = 0;
          if ((t_point_lower.z > 0) && (t_point_upper.z > 0))
          {
            if (t_point_lower.x > t_point_upper.x)
            {
              Point3D t_point_swap = t_point_upper;
              t_point_upper = t_point_lower;
              t_point_lower = t_point_swap;
            }
            if (t_x > t_point_upper.x)
            {
              t_y_interp = LinearInterpolation(t_point_lower.x, t_point_lower.y, t_point_upper.x, t_point_upper.y, t_x,
                                               Lin_Interp_Method::kExtrapolate);
              t_z_interp = 0.f;
            }
            else
            {
              t_y_interp = LinearInterpolation(t_point_lower.x, t_point_lower.y, t_point_upper.x, t_point_upper.y, t_x);
              t_z_interp = bc::min(bc::abs(t_x - t_point_lower.x), bc::abs(t_x - t_point_upper.x));
              t_z_interp = bc::exp(-t_z_interp);
            }
          }
          else if (t_point_lower.z > 0)
          {
            t_y_interp = t_point_lower.y;
          }
          else if (t_point_upper.z > 0)
          {
            t_y_interp = t_point_upper.y;
          }
          else
          {
            t_y_interp = 0;
          }
          t_element_new.right_boundary_.pts_[t_idx_interp].x = t_x;
          t_element_new.right_boundary_.pts_[t_idx_interp].y = t_y_interp;
          t_element_new.right_boundary_.pts_[t_idx_interp].z = t_z_interp;

          if ((t_x > -0.5) && (t_x < 0.5))
          {
            t_ego_point_idx_ = t_idx_interp;
          }
          t_idx_interp++;
        }
        t_element_new.right_boundary_.ego_point_idx_ = t_ego_point_idx_;
        t_element_new.right_boundary_.total_valid_number_ = t_map_end_idx - t_map_start_idx + 1;
        t_element_new.right_boundary_.start_idx_ = t_map_start_idx;
        t_element_new.right_boundary_.reserve_[0] = t_map_start_idx;
        t_element_new.right_boundary_.existence_ = bc::true_v;
        t_element = t_element_new;

        // Fix reference line not at the middle of the two boundaries
        if (t_element.reference_line_.available_ &&
            t_element.reference_line_.current_point_idx_ < (uint32_t)kMaxRefLinePtsNum)
        {
          bc::uint8_t left_start_idx = t_element.left_boundary_.start_idx_;
          bc::uint8_t left_end_idx = left_start_idx + t_element.left_boundary_.total_valid_number_ - 1;
          for (bc::int8_t i = t_element.left_boundary_.ego_point_idx_; i > 0; --i)
          {
            if (t_element.left_boundary_.pts_[i - 1].z < 0.01f)
            {
              left_start_idx = i;
              break;
            }
          }
          for (bc::uint8_t i = t_element.left_boundary_.ego_point_idx_; i < kMaxBoundaryPoint - 1; ++i)
          {
            if (t_element.left_boundary_.pts_[i + 1].z < 0.01f)
            {
              left_end_idx = i;
              break;
            }
          }
          bc::uint8_t right_start_idx = t_element.right_boundary_.start_idx_;
          bc::uint8_t right_end_idx = right_start_idx + t_element.right_boundary_.total_valid_number_ - 1;
          for (bc::int8_t i = t_element.right_boundary_.ego_point_idx_; i > 0; --i)
          {
            if (t_element.right_boundary_.pts_[i - 1].z < 0.01f)
            {
              right_start_idx = i;
              break;
            }
          }
          for (bc::uint8_t i = t_element.right_boundary_.ego_point_idx_; i < kMaxBoundaryPoint - 1; ++i)
          {
            if (t_element.right_boundary_.pts_[i + 1].z < 0.01f)
            {
              right_end_idx = i;
              break;
            }
          }
          bc::uint8_t start_idx = std::max(left_start_idx, right_start_idx);
          bc::uint8_t end_idx = std::min(left_end_idx, right_end_idx);
          // bc::uint8_t end_end_idx = end_idx;
          if (start_idx < end_idx)
          {
            bc::TCArray<Point2D, kMaxRefLinePtsNum> points;
            bc::uint8_t valid_pts = 0;
            for (bc::uint8_t i = start_idx; i <= end_idx; ++i)
            {
              Point2D current_point(0.5 * (t_element.left_boundary_.pts_[i].x + t_element.right_boundary_.pts_[i].x),
                                    0.5 * (t_element.left_boundary_.pts_[i].y + t_element.right_boundary_.pts_[i].y));
              points[valid_pts] = current_point;
              valid_pts++;
            }
            PolynomialRegression poly_fit;
            Eigen::Vector4d local_coeff{0.f, 0.f, 0.f, 0.f};
            if (valid_pts >= 4)
            {
              poly_fit.SetPara(points, valid_pts, true);
              local_coeff = poly_fit.PolyGen();

              t_element.reference_line_.coeff_.clothoid_.c0_position = local_coeff[0];
              t_element.reference_line_.coeff_.clothoid_.c1_heading_angle = local_coeff[1];
              t_element.reference_line_.coeff_.clothoid_.c2_curvature = local_coeff[2];
              t_element.reference_line_.coeff_.clothoid_.c3_curvature_derivative = local_coeff[3];

              t_element.reference_line_.coeff_.clothoid_.model_degree = 3U;
              t_element.reference_line_.coeff_.start_x_ = points[0].x;
              t_element.reference_line_.coeff_.end_x_ = points[valid_pts - 1].x;
              t_element.reference_line_.coeff_.valid_ = bc::true_v;
            }
            else
            {
              t_element.reference_line_.coeff_.valid_ = bc::false_v;
            }
          }
          else
          {
            t_element.reference_line_.coeff_.valid_ = bc::false_v;
          }
        }
      }
    }
  }
  return bc::true_v;
}

void Preprocess::StoreHDMapLaneInfoToInternal(const EmAdapterHDmapData& map_adapter_cs)
{
  bc::bool_t t_success_b = bc::false_v;
  for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
  {
    for (bc::uint8_t t_elem_idx_u8 = 0; t_elem_idx_u8 < kMaxElemNumInOneLane; t_elem_idx_u8++)
    {
      const LaneElement& t_inp_elem_cs = map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_elem_idx_u8];
      LaneBoundaryPoint& t_otp_elem_cs =
          map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8].lane_element_point_ar_[t_elem_idx_u8];
      if (t_inp_elem_cs.element_valid_)
      {
        StoreMapLineToInternal(t_inp_elem_cs.left_boundary_, t_otp_elem_cs.left_point_cs_);
        StoreMapLineToInternal(t_inp_elem_cs.right_boundary_, t_otp_elem_cs.right_point_cs_);
        if ((t_otp_elem_cs.left_point_cs_.valid_b_ || t_otp_elem_cs.right_point_cs_.valid_b_))
        {
          t_otp_elem_cs.valid_b_ = bc::true_v;
          map_lane_collection_cs_.map_point_ar_[t_lane_idx_u8].valid_b_ = bc::true_v;
          t_success_b = bc::true_v;
        }
      }
    }
  }
  map_lane_collection_cs_.map_quality_ar_.prob_exist_f_ = map_adapter_cs.map_car_position_.confidence_;
  map_lane_collection_cs_.map_quality_ar_.valid_b_ = t_success_b;
}

void Preprocess::LaneBoundaryOffsetsForMatching(const LaneData& lane_cs, const bc::TCArray<bc::uint8_t, 4> end_ar,
                                                bc::TCArray<bc::float32_t, 4>& offset_ar)
{
  bc::TCArray<LaneElement, 2> t_eles_cs;
  if (lane_cs.lane_valid_)
  {
    t_eles_cs[0] = lane_cs.lane_elements_[0];
    t_eles_cs[1] = lane_cs.lane_elements_[1];
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < 2; ++t_idx_u8)
    {
      if (t_eles_cs[t_idx_u8].element_valid_)
      {
        if (t_eles_cs[t_idx_u8].left_boundary_.existence_)
        {
          if (t_eles_cs[t_idx_u8].left_boundary_.ego_point_idx_ == kInvalidEgoPointIdx)
          {
#ifdef STD_COUT_ENABLE
            std::cout << "Error occurs in LaneBoundaryOffsetsForMatching!!!" << std::endl;
#endif
          }
          else
          {
            offset_ar[t_idx_u8 * 2 + 0] =
                t_eles_cs[t_idx_u8].left_boundary_.pts_[t_eles_cs[t_idx_u8].left_boundary_.ego_point_idx_].y;
            // +                     t_eles_cs[t_idx_u8].left_boundary_.pts_[end_ar[t_idx_u8 * 2 + 0]].y);
          }
        }
        if (t_eles_cs[t_idx_u8].right_boundary_.existence_)
        {
          if (t_eles_cs[t_idx_u8].right_boundary_.ego_point_idx_ == kInvalidEgoPointIdx)
          {
#ifdef STD_COUT_ENABLE
            std::cout << "Error occurs in LaneBoundaryOffsetsForMatching!!!" << std::endl;
#endif
          }
          else
          {
            offset_ar[t_idx_u8 * 2 + 1] =
                t_eles_cs[t_idx_u8].right_boundary_.pts_[t_eles_cs[t_idx_u8].right_boundary_.ego_point_idx_].y;
            //  +                     t_eles_cs[t_idx_u8].right_boundary_.pts_[end_ar[t_idx_u8 * 2 + 1]].y);
          }
        }
      }
    }
  }
}

void Preprocess::FixElementBoundaryBend(EmAdapterHDmapData& map_adapter_cs)
{
  /**
   * When one lane has more than one elements, some boundary may bend because this boundary comes from two boundary
   * segments in Ehr process.
   * if left boundary tends to right boundary and mutation point is A, this function will copy the rear part of A in
   * right boundary to left boundary as the overlap part.
   *
   */
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLaneNum; ++t_idx_u8)
  {
    if (map_adapter_cs.lanes_[t_idx_u8].lane_valid_ && map_adapter_cs.lanes_[t_idx_u8].valid_elements_cnts_ > 1)
    {
      for (bc::uint8_t t_element_u8 = 0;
           t_element_u8 < kMaxElemNumInOneLane && t_element_u8 < map_adapter_cs.lanes_[t_idx_u8].valid_elements_cnts_;
           ++t_element_u8)
      {
        if (map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].left_boundary_.existence_ &&
            map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].right_boundary_.existence_)
        {
          bc::uint8_t t_mutate1_idx_u8 = 0;
          bc::uint8_t t_mutate1_cnts_u8 = 0;
          bc::uint8_t t_mutate2_idx_u8 = 0;
          bc::uint8_t t_mutate2_cnts_u8 = 0;
          bc::TCArray<Point3D, kMaxBoundaryPoint>& t_l_pts_ar =
              map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].left_boundary_.pts_;
          bc::TCArray<Point3D, kMaxBoundaryPoint>& t_r_pts_ar =
              map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].right_boundary_.pts_;
          bc::uint8_t t_l_nums_u8 =
              map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].left_boundary_.total_valid_number_;
          bc::uint8_t t_r_nums_u8 =
              map_adapter_cs.lanes_[t_idx_u8].lane_elements_[t_element_u8].right_boundary_.total_valid_number_;
          for (bc::uint8_t t_cnt_u8 = 1; t_cnt_u8 < t_l_nums_u8; ++t_cnt_u8)
          {
            if (fabs(t_l_pts_ar[t_cnt_u8].y - t_l_pts_ar[t_cnt_u8 - 1].y) > kPointGapThreshDy)
            {
              if (t_mutate1_idx_u8 == 0)
              {
                t_mutate1_idx_u8 = t_cnt_u8;
                t_mutate1_cnts_u8 = 1;
              }
            }
            else
            {
              t_mutate1_cnts_u8++;
            }
          }
          for (bc::uint8_t t_cnt_u8 = 1; t_cnt_u8 < t_r_nums_u8; ++t_cnt_u8)
          {
            if (fabs(t_r_pts_ar[t_cnt_u8].y - t_r_pts_ar[t_cnt_u8 - 1].y) > kPointGapThreshDy)
            {
              if (t_mutate2_idx_u8 == 0)
              {
                t_mutate2_idx_u8 = t_cnt_u8;
                t_mutate2_cnts_u8 = 1;
              }
            }
            else
            {
              t_mutate2_cnts_u8++;
            }
          }
          if (t_mutate1_idx_u8 * t_mutate2_idx_u8 != 0)
          {
#ifdef STD_COUT_ENABLE
            std::cout << "Error occurs in FixElementBoundaryBend!!!" << std::endl;
#endif
            break;
          }
          /** left boudary mutates */
          if (t_mutate1_idx_u8 > 0 && t_mutate1_cnts_u8 > 3 &&
              fabs(t_l_pts_ar[t_mutate1_idx_u8].y - t_r_pts_ar[t_mutate1_idx_u8].y) < kPointGapThreshDy)
          {
            for (bc::uint8_t t_cnt_u8 = 0; t_cnt_u8 <= t_mutate1_idx_u8; ++t_cnt_u8)
            {
              t_l_pts_ar[t_cnt_u8] = t_r_pts_ar[t_cnt_u8];
            }
          }
          /** right boudary mutates */
          if (t_mutate2_idx_u8 > 0 && t_mutate2_cnts_u8 > 3 &&
              fabs(t_l_pts_ar[t_mutate2_idx_u8].y - t_r_pts_ar[t_mutate2_idx_u8].y) < kPointGapThreshDy)
          {
            for (bc::uint8_t t_cnt_u8 = 0; t_cnt_u8 <= t_mutate2_idx_u8; ++t_cnt_u8)
            {
              t_r_pts_ar[t_cnt_u8] = t_l_pts_ar[t_cnt_u8];
            }
          }
        }
      }
    }
  }
}

void Preprocess::MapMatchingWithLastEm(const bc::TCArray<LaneData, kMaxLaneNum>& em_lane_lst1_ar,
                                       EmAdapterHDmapData& map_adapter_cs)
{
  lane_match_s8 = 0;
  bc::bool_t t_match_b = bc::false_v;
  map_lane_change_s8 = 0;
  bc::uint16_t lst_host_ele_id_u16_ = em_lane_lst1_ar[kHostLane].lane_elements_[0].element_id_;
  bc::uint16_t map_host_ele_id_u16 = map_adapter_cs.lanes_[kHostLane].lane_elements_[0].element_id_;
  if (lst_host_ele_id_u16_ == kInvalidElementId)
  {
    /// do nothing
    t_match_b = bc::true_v;
  }
  else
  {
    if (map_host_ele_id_u16 == lst_host_ele_id_u16_)
    {
      /// do nothing
      t_match_b = bc::true_v;
    }
    else
    {
      for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
      {
        if (map_adapter_cs.lanes_[t_lane_idx_u8].lane_valid_ && t_match_b == bc::false_v)
        {
          for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
          {
            if (map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_valid_ &&
                map_adapter_cs.lanes_[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_id_ == lst_host_ele_id_u16_ &&
                t_match_b == bc::false_v)
            {
              if (t_ele_idx_u8 == 0)
              {
                lane_match_s8 = kHostLane - t_lane_idx_u8;
                t_match_b = bc::true_v;
                break;
              }
              else
              {
                ///@todo when element >1
              }
            }
            else
            {
              continue;
            }
          }
        }
        else
        {
          continue;
        }
      }
      /// no element can match with last host
      if (t_match_b == bc::false_v)
      {
        /// find match with map host
        for (bc::uint8_t t_lane_idx_u8 = 0; t_lane_idx_u8 < kMaxLaneNum; t_lane_idx_u8++)
        {
          if (em_lane_lst1_ar[t_lane_idx_u8].lane_valid_ && t_match_b == bc::false_v)
          {
            for (bc::uint8_t t_ele_idx_u8 = 0; t_ele_idx_u8 < kMaxElemNumInOneLane; t_ele_idx_u8++)
            {
              if (em_lane_lst1_ar[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_valid_ &&
                  em_lane_lst1_ar[t_lane_idx_u8].lane_elements_[t_ele_idx_u8].element_id_ == map_host_ele_id_u16 &&
                  t_match_b == bc::false_v)
              {
                /// e.g. last cycle 0:103,1:102,2:98,3:104
                ///     this map          1:103,2:102,3:104
                /// it means ego changed to left
                map_lane_change_s8 = kHostLane - t_lane_idx_u8;
              }
            }
          }
        }
      }
      /// cannot match any element
      if (t_match_b == bc::false_v)
      {
#ifdef STD_COUT_ENABLE
        std::cout << "Warning in Preprocess::MapMatchingWithLastEm: Map element id match failed!" << std::endl;
#endif
      }
      /// do matching
      if (lane_match_s8 != 0)
      {
        // std::cout << "Map lane mapping triggler" << std::endl;
        EmAdapterHDmapData t_map_adapter_cs = map_adapter_cs;
        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLaneNum; ++t_idx_u8)
        {
          map_adapter_cs.lanes_[t_idx_u8] = LaneData();
        }
        for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLaneNum; ++t_idx_u8)
        {
          if (t_idx_u8 + lane_match_s8 < 0 || t_idx_u8 + lane_match_s8 > kMaxLaneNum - 1)
          {
#ifdef STD_COUT_ENABLE
            std::cout << "Warning occurs in MapMatchingWithLastEm: lost Leftmost and Rightmost lane!!!" << std::endl;
#endif
            continue;
          }
          else
          {
            map_adapter_cs.lanes_[t_idx_u8 + lane_match_s8] = t_map_adapter_cs.lanes_[t_idx_u8];
          }
        }
        // map_adapter_cs.relat_dest_lane_ -= lane_match_s8;
        bc::float32_t temp_1 = map_adapter_cs.lanes_[kHostLane + lane_match_s8].lane_elements_[0].route_left_dis_ -
                               map_adapter_cs.lanes_[kHostLane].lane_elements_[0].route_left_dis_;
        bc::float32_t temp_2 = map_adapter_cs.lanes_[kHostLane + lane_match_s8].lane_elements_[1].route_left_dis_ -
                               map_adapter_cs.lanes_[kHostLane].lane_elements_[0].route_left_dis_;
        if (map_adapter_cs.relat_dest_lane_ != 0 ||
            ((map_adapter_cs.relat_dest_lane_ == 0) &&
             ((map_adapter_cs.lanes_[kHostLane + lane_match_s8].lane_elements_[0].element_valid_ &&
               fabsf(temp_1) > FLOAT32_EPSILON) ||
              (map_adapter_cs.lanes_[kHostLane + lane_match_s8].lane_elements_[0].element_valid_ &&
               fabsf(temp_2) > FLOAT32_EPSILON))))
        {
          map_adapter_cs.relat_dest_lane_ -= lane_match_s8;
          for (bc::uint8_t i = 0; i < kMaxMapGuidePtNum; ++i)
          {
            if (map_adapter_cs.map_guide_pts_[i].valid_ &&
                (map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kMergePoint ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kRoadSplit ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchEntranceRamp ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchExitRamp ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchEntranceJCT ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchExitJCT ||
                 map_adapter_cs.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchOffway))
            {
              map_adapter_cs.map_guide_pts_[i].target_lane_index_ -= lane_match_s8;
            }
          }
        }
      }
    }
  }
}
void Preprocess::StoreMapLineToInternal(const LaneBoundary& external_bound_cs, BoundaryPoint& internal_bound_cs)
{
  if (!external_bound_cs.existence_ || external_bound_cs.total_valid_number_ < 2U)
  {
    internal_bound_cs.valid_b_ = bc::false_v;
    return;
  }
  bc::uint8_t t_total_nums_u16 = external_bound_cs.total_valid_number_;
  bc::uint8_t t_start_idx_u8 = external_bound_cs.reserve_[0];
  bc::TCArray<Point3D, kMaxBoundaryPoint> t_external_points_ar;
  bc::uint8_t t_total_num = t_start_idx_u8 + t_total_nums_u16;
  bc::float32_t t_total_num_f = external_bound_cs.reserve_[0] + external_bound_cs.total_valid_number_;
  for (bc::uint8_t t_idx_u8 = t_start_idx_u8;
       t_idx_u8 < (bc::uint16_t)((bc::uint16_t)t_start_idx_u8 + (bc::uint16_t)t_total_nums_u16); ++t_idx_u8)
  {
    t_external_points_ar[t_idx_u8] = external_bound_cs.pts_[t_idx_u8];
    if (std::isnan(t_external_points_ar[t_idx_u8].x) || std::isnan(t_external_points_ar[t_idx_u8].y))
    {
#ifdef STD_COUT_ENABLE
      std::cout << "Right Input Nan in Environment Model StoreMapLineToInternal!!!" << std::endl;
#endif
    }
  }
  // bc::TCArray<Point3D, kMaxBoundaryPoint> t_external_points_ar = external_bound_cs.pts_;
  if (StoreTrackBoundaryPoints(t_external_points_ar, internal_bound_cs, t_total_nums_u16, t_start_idx_u8))
  {
    internal_bound_cs.valid_b_ = bc::true_v;
    internal_bound_cs.boundary_type_en_ = external_bound_cs.segs_[0].boundary_type_;
    internal_bound_cs.dx_start_f_ = x_ref_ar_[internal_bound_cs.dy_start_idx_u8_];
    internal_bound_cs.dx_end_f_ = x_ref_ar_[internal_bound_cs.dy_end_idx_u8_];
  }
}

void Preprocess::GetDbgData(EmPreDbgData& dbg_data_cs)
{
  dbg_data_cs.dx_ar_ = x_ref_ar_;
  dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_ = per_lane_collection_cs_.per_point_ar_[2].left_point_cs_.dy_ar_;
  dbg_data_cs.ledveh_cs_.right_cs_.dy_ar_ =
      ego_track_lane_collection_cs_.ego_track_point_ar_[2].lane_element_point_ar_[0].left_point_cs_.dy_ar_;
  dbg_data_cs.lv_dbg_cs_ = lv_dbg_cs_;
  if (ldveh_lane_collection_cs_.ldveh_point_ar_[0].valid_b_)
  {
    // dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_ = ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_ar_;
    // dbg_data_cs.ledveh_cs_.right_cs_.dy_ar_ = ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_ar_;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundaryPoint; ++t_idx_u8)
    {
      dbg_data_cs.ledveh_cs_.center_cs_.dy_ar_[t_idx_u8] =
          0.5 * (ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_ar_[t_idx_u8] +
                 ldveh_lane_collection_cs_.ldveh_point_ar_[0].right_point_cs_.dy_ar_[t_idx_u8]);
    }
    dbg_data_cs.ledveh_cs_.tar_id_u8_ = ldveh_lane_collection_cs_.veh_id_u16_;
    dbg_data_cs.ledveh_cs_.valid_b_ = bc::true_v;
    // dbg_data_cs.ledveh_cs_.end_idx_ = 99;
    // dbg_data_cs.ledveh_cs_.start_idx_ = 20;
    dbg_data_cs.ledveh_cs_.end_idx_ = ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_end_idx_u8_;
    dbg_data_cs.ledveh_cs_.start_idx_ = ldveh_lane_collection_cs_.ldveh_point_ar_[0].left_point_cs_.dy_start_idx_u8_;
  }
  else
  {
    dbg_data_cs.ledveh_cs_.valid_b_ = bc::false_v;
    dbg_data_cs.ledveh_cs_.tar_id_u8_ = 0;
    dbg_data_cs.ledveh_cs_.end_idx_ = 99;
    dbg_data_cs.ledveh_cs_.start_idx_ = 0;
  }
  dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_[0] = (joint_points_collection_cs_[0].joint_b_ == bc::true_v) ? 1 : 0;
  dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_[1] = (joint_points_collection_cs_[1].joint_b_ == bc::true_v) ? 1 : 0;
  dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_[2] = (joint_points_collection_cs_[2].joint_b_ == bc::true_v) ? 1 : 0;
  dbg_data_cs.ledveh_cs_.left_cs_.dy_ar_[3] = (joint_points_collection_cs_[3].joint_b_ == bc::true_v) ? 1 : 0;
  dbg_data_cs.joint_cs_.left_per_cs_ = joint_points_collection_cs_[0];
  dbg_data_cs.joint_cs_.right_per_cs_ = joint_points_collection_cs_[1];
  dbg_data_cs.joint_cs_.left_ldveh_cs_ = joint_points_collection_cs_[2];
  dbg_data_cs.joint_cs_.right_ldveh_cs_ = joint_points_collection_cs_[3];
  if (per_lane_collection_cs_.per_coeff_ar_[2].valid_b_)
  {
    dbg_data_cs.per_cs_.host_left_coeff_ = per_lane_collection_cs_.per_coeff_ar_[2].left_coeff_cs_.coeff_cs_;
    dbg_data_cs.per_cs_.host_right_coeff_ = per_lane_collection_cs_.per_coeff_ar_[2].right_coeff_cs_.coeff_cs_;
    dbg_data_cs.per_cs_.valid_b_ = bc::true_v;
    dbg_data_cs.per_cs_.host_left_coeff_.model_degree = per_lc_reason_u8_;
  }
  else
  {
    dbg_data_cs.per_cs_.valid_b_ = bc::false_v;
    dbg_data_cs.per_cs_ = zone::common::EmPrePerData();
    dbg_data_cs.per_cs_.host_left_coeff_.model_degree = 0;
  }
  if (map_adapter_cs_.lanes_[2].lane_valid_)
  {
    dbg_data_cs.map_cs_ = map_dbg_data_;
    dbg_data_cs.map_cs_.host_ref_coeff_ = map_adapter_cs_.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_;
    dbg_data_cs.map_cs_.valid_b_ = bc::true_v;
    dbg_data_cs.map_cs_.filtered_c0_ = map_end_x_;
    // map_adapter_cs_.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c0_position +
    // filtered_delta_offset_;
    dbg_data_cs.map_cs_.filtered_c1_ = map_end_idx_;
    // map_adapter_cs_.lanes_[2].lane_elements_[0].reference_line_.coeff_.clothoid_.c1_heading_angle +
    // filtered_delta_heading_;
  }
  else
  {
    dbg_data_cs.map_cs_ = map_dbg_data_;
    dbg_data_cs.map_cs_.valid_b_ = bc::false_v;
    dbg_data_cs.map_cs_ = zone::common::EmPreMapData();
  }

for(bc::uint8_t i = 0; i < 5;i++){
  dbg_data_cs.reserve_[2 * i] = per_lane_collection_cs_.per_coeff_ar_[i].left_coeff_cs_.life_time_f_;
  dbg_data_cs.reserve_[2 * i + 1] = per_lane_collection_cs_.per_coeff_ar_[i].right_coeff_cs_.life_time_f_;
}

dbg_data_cs.reserve_[10] = left_per_ref_similarity_f_;
dbg_data_cs.reserve_[11] = right_per_ref_similarity_f_;
}

void Preprocess::SplitMapLaneData(EmAdapterHDmapData& map_em_data)
{
  for (bc::uint8_t i = 0; i < kMaxLaneNum; i++)
  {
    ::zone::data::em_data::LaneData& cur_lane = map_em_data.lanes_[i];
    if (cur_lane.lane_valid_ && cur_lane.lane_elements_[0].element_valid_ && cur_lane.lane_elements_[1].element_valid_)
    {
      if (cur_lane.lane_elements_[0].merge_flag_ && cur_lane.lane_elements_[1].merge_flag_)
      {
        cur_lane.valid_elements_cnts_ = 1;
        cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
      }
      else
      {
        bc::uint8_t t_min_ele0_l_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_min_ele1_l_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_max_ele0_l_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_max_ele1_l_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_min_ele0_r_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_min_ele1_r_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_max_ele0_r_idx_u8 = kMaxBoundaryPoint;
        bc::uint8_t t_max_ele1_r_idx_u8 = kMaxBoundaryPoint;
        FindMarginalBoundaryPtsWithSameDx(cur_lane.lane_elements_[0].left_boundary_,
                                          cur_lane.lane_elements_[1].left_boundary_, t_min_ele0_l_idx_u8,
                                          t_min_ele1_l_idx_u8, t_max_ele0_l_idx_u8, t_max_ele1_l_idx_u8);
        FindMarginalBoundaryPtsWithSameDx(cur_lane.lane_elements_[0].right_boundary_,
                                          cur_lane.lane_elements_[1].right_boundary_, t_min_ele0_r_idx_u8,
                                          t_min_ele1_r_idx_u8, t_max_ele0_r_idx_u8, t_max_ele1_r_idx_u8);
        if (fabsf(cur_lane.lane_elements_[1].left_boundary_.pts_[t_min_ele1_l_idx_u8].y -
                  cur_lane.lane_elements_[0].left_boundary_.pts_[t_min_ele0_l_idx_u8].y) > 0.1f ||
            fabsf(cur_lane.lane_elements_[1].left_boundary_.pts_[t_max_ele1_l_idx_u8].y -
                  cur_lane.lane_elements_[0].left_boundary_.pts_[t_max_ele0_l_idx_u8].y) > 0.1f ||
            fabsf(cur_lane.lane_elements_[1].right_boundary_.pts_[t_min_ele1_r_idx_u8].y -
                  cur_lane.lane_elements_[0].right_boundary_.pts_[t_min_ele0_r_idx_u8].y) > 0.1f ||
            fabsf(cur_lane.lane_elements_[1].right_boundary_.pts_[t_max_ele1_r_idx_u8].y -
                  cur_lane.lane_elements_[0].right_boundary_.pts_[t_max_ele0_r_idx_u8].y) > 0.1f)
        {
          if (cur_lane.lane_elements_[0].relative_idx_ < cur_lane.lane_elements_[1].relative_idx_)
          {
            if (i == ::zone::data::em_data::kLLLane)
            {
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
            }
            else if (i == ::zone::data::em_data::kLeftLane)
            {
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[0] = cur_lane.lane_elements_[1];
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].main_lane_idx_ = cur_lane.main_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].global_lane_idx_ = cur_lane.global_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].valid_elements_cnts_ = 1;
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[1].element_valid_ = bc::false_v;
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_valid_ = bc::true_v;
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
              if (map_em_data.relat_dest_lane_ == 1 &&
                  !map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].is_dest_lane_ele_ &&
                  (map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].route_left_dis_ <
                       map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[0].route_left_dis_ ||
                   map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = 2;
              }
            }
            else if (i == ::zone::data::em_data::kHostLane)
            {
              map_em_data.lanes_[ ::zone::data::em_data::kLLLane] =
                  map_em_data.lanes_[ ::zone::data::em_data::kLeftLane];
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0] = cur_lane.lane_elements_[1];
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].main_lane_idx_ = cur_lane.main_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].global_lane_idx_ = cur_lane.global_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].valid_elements_cnts_ = 1;
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[1].element_valid_ = bc::false_v;
              map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_valid_ = bc::true_v;
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
              if (map_em_data.relat_dest_lane_ == 0 &&
                  !map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].is_dest_lane_ele_ &&
                  (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_ <
                       map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].route_left_dis_ ||
                   map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = 1;
              }
              else if (map_em_data.relat_dest_lane_ == 1 &&
                       !map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].is_dest_lane_ele_ &&
                       (map_em_data.lanes_[ ::zone::data::em_data::kLeftLane].lane_elements_[0].route_left_dis_ <
                            map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[0].route_left_dis_ ||
                        map_em_data.lanes_[ ::zone::data::em_data::kLLLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = 2;
              }
            }
            else if (i == ::zone::data::em_data::kRightLane)
            {
              // std::cout << "!!!!!!!!!!!!!!! Unreal scenario on RightLane!" << std::endl;
            }
            else if (i == ::zone::data::em_data::kRRLane)
            {
              // std::cout << "!!!!!!!!!!!!!!! Unreal scenario on RRLane!" << std::endl;
            }
          }
          else if (cur_lane.lane_elements_[0].relative_idx_ > cur_lane.lane_elements_[1].relative_idx_)
          {
            if (i == ::zone::data::em_data::kLLLane)
            {
              // std::cout << "!!!!!!!!!!!!!!! Unreal scenario on LLLane!" << std::endl;
            }
            else if (i == ::zone::data::em_data::kLeftLane)
            {
              // std::cout << "!!!!!!!!!!!!!!! Unreal scenario on LeftLane!" << std::endl;
            }
            else if (i == ::zone::data::em_data::kHostLane)
            {
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane] =
                  map_em_data.lanes_[ ::zone::data::em_data::kRightLane];
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0] = cur_lane.lane_elements_[1];
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].main_lane_idx_ = cur_lane.main_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].global_lane_idx_ = cur_lane.global_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].valid_elements_cnts_ = 1;
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[1].element_valid_ = bc::false_v;
              map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_valid_ = bc::true_v;
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
              i++;
              if (map_em_data.relat_dest_lane_ == 0 &&
                  !map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].is_dest_lane_ele_ &&
                  (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].route_left_dis_ <
                       map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].route_left_dis_ ||
                   map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = -1;
              }
              else if (map_em_data.relat_dest_lane_ == -1 &&
                       !map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].is_dest_lane_ele_ &&
                       (map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].route_left_dis_ <
                            map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[0].route_left_dis_ ||
                        map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = -2;
              }
            }
            else if (i == ::zone::data::em_data::kRightLane)
            {
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[0] = cur_lane.lane_elements_[1];
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].main_lane_idx_ = cur_lane.main_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].global_lane_idx_ = cur_lane.global_lane_idx_;
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].valid_elements_cnts_ = 1;
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[1].element_valid_ = bc::false_v;
              map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_valid_ = bc::true_v;
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
              i++;
              if (map_em_data.relat_dest_lane_ == -1 &&
                  !map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].is_dest_lane_ele_ &&
                  (map_em_data.lanes_[ ::zone::data::em_data::kRightLane].lane_elements_[0].route_left_dis_ <
                       map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[0].route_left_dis_ ||
                   map_em_data.lanes_[ ::zone::data::em_data::kRRLane].lane_elements_[0].is_dest_lane_ele_))
              {
                map_em_data.relat_dest_lane_ = -2;
              }
            }
            else if (i == ::zone::data::em_data::kRRLane)
            {
              cur_lane.valid_elements_cnts_ = 1;
              cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
            }
          }
          else
          {
            cur_lane.valid_elements_cnts_ = 1;
            cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
#ifdef STD_COUT_ENABLE
            std::cout << "Error occurs in SplitMapLaneData!!! Two elements in one lane have same index!!!" << std::endl;
#endif
          }
        }
        else
        {
          cur_lane.valid_elements_cnts_ = 1;
          cur_lane.lane_elements_[1].element_valid_ = bc::false_v;
        }
      }
    }
  }

  // Reorganize lane marking assigned lane idx
  for (bc::uint8_t i = 0; i < zone::data::em_data::kMaxLaneMarkingNum; i++)
  {
    map_em_data.lane_marking_ar_[i].lane_assigned_ = kMaxLaneNum;
    for (bc::uint8_t j = 0; j < kMaxLaneNum; j++)
    {
      if (map_em_data.lane_marking_element_id_ar_[i] == map_em_data.lanes_[j].lane_elements_[0].element_id_)
      {
        map_em_data.lane_marking_ar_[i].lane_assigned_ = j;
        break;
      }
    }
    if (map_em_data.lane_marking_ar_[i].lane_assigned_ == kMaxLaneNum)
    {
      map_em_data.lane_marking_ar_[i] = EmLaneMarking();
    }
  }
}

void Preprocess::FindMarginalBoundaryPtsWithSameDx(const LaneBoundary& ele0_boundary, const LaneBoundary& ele1_boundary,
                                                   bc::uint8_t& min_ele0_index, bc::uint8_t& min_ele1_index,
                                                   bc::uint8_t& max_ele0_index, bc::uint8_t& max_ele1_index)
{
  if (ele1_boundary.pts_[0].x - ele0_boundary.pts_[0].x > FLOAT32_EPSILON)
  {
    for (bc::uint8_t t_point_idx_offset_u8 = 1; t_point_idx_offset_u8 < kMaxBoundaryPoint; t_point_idx_offset_u8++)
    {
      if (ele1_boundary.pts_[0].x - ele0_boundary.pts_[t_point_idx_offset_u8].x < FLOAT32_EPSILON)
      {
        min_ele0_index = t_point_idx_offset_u8;
        min_ele1_index = 0;
        break;
      }
    }
  }
  else if (ele0_boundary.pts_[0].x - ele1_boundary.pts_[0].x > FLOAT32_EPSILON)
  {
    for (bc::uint8_t t_point_idx_offset_u8 = 1; t_point_idx_offset_u8 < kMaxBoundaryPoint; t_point_idx_offset_u8++)
    {
      if (ele0_boundary.pts_[0].x - ele1_boundary.pts_[t_point_idx_offset_u8].x < FLOAT32_EPSILON)
      {
        min_ele0_index = 0;
        min_ele1_index = t_point_idx_offset_u8;
        break;
      }
    }
  }
  else
  {
    min_ele0_index = 0;
    min_ele1_index = 0;
  }
  if (ele1_boundary.pts_[ele1_boundary.total_valid_number_ - 1].x -
          ele0_boundary.pts_[ele0_boundary.total_valid_number_ - 1].x >
      FLOAT32_EPSILON)
  {
    for (bc::uint8_t t_point_idx_offset_u8 = 1; t_point_idx_offset_u8 < kMaxBoundaryPoint; t_point_idx_offset_u8++)
    {
      if (ele1_boundary.pts_[ele1_boundary.total_valid_number_ - 1 - t_point_idx_offset_u8].x -
              ele0_boundary.pts_[ele0_boundary.total_valid_number_ - 1].x <
          FLOAT32_EPSILON)
      {
        max_ele0_index = ele0_boundary.total_valid_number_ - 1;
        max_ele1_index = ele1_boundary.total_valid_number_ - 1 - t_point_idx_offset_u8;
        break;
      }
    }
  }
  else if (ele0_boundary.pts_[0].x - ele1_boundary.pts_[0].x > FLOAT32_EPSILON)
  {
    for (bc::uint8_t t_point_idx_offset_u8 = 1; t_point_idx_offset_u8 < kMaxBoundaryPoint; t_point_idx_offset_u8++)
    {
      if (ele0_boundary.pts_[ele0_boundary.total_valid_number_ - 1 - t_point_idx_offset_u8].x -
              ele1_boundary.pts_[ele1_boundary.total_valid_number_ - 1].x <
          FLOAT32_EPSILON)
      {
        max_ele0_index = ele0_boundary.total_valid_number_ - 1 - t_point_idx_offset_u8;
        max_ele1_index = ele1_boundary.total_valid_number_ - 1;
        break;
      }
    }
  }
  else
  {
    max_ele0_index = ele0_boundary.total_valid_number_ - 1;
    max_ele1_index = ele1_boundary.total_valid_number_ - 1;
  }
}

void Preprocess::MapScenarioDistinction(EmAdapterHDmapData& map_em_data)
{
  bc::bool_t t_check_status_b = bc::false_v;
  bc::float32_t temp_start = 0.f;
  bc::float32_t temp_end = 0.f;
  bc::bool_t has_tollbooth = bc::false_v;
  bc::float32_t temp_service_zone_start = 0.f;
  bc::float32_t temp_service_zone_end = 0.f;
  bc::bool_t has_service_zone = bc::false_v;
  if (map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].element_valid_)
  {
    ReferenceLine refline = map_em_data.lanes_[ ::zone::data::em_data::kHostLane].lane_elements_[0].reference_line_;
    for (bc::uint8_t i = 0; i < ::zone::data::em_data::kMaxSpecialSituationSegsInOneLaneElement; ++i)
    {
      if (refline.special_segs_[i].valid_ &&
          refline.special_segs_[i].special_situation_ == ::zone::data::em_data::SpecialSituation::kTollBooth)
      {
        if (has_tollbooth)
        {
          temp_start = std::min(temp_start, refline.special_segs_[i].start_s_);
          temp_end = std::max(temp_end, refline.special_segs_[i].end_s_);
        }
        else
        {
          has_tollbooth = bc::true_v;
          temp_start = refline.special_segs_[i].start_s_;
          temp_end = refline.special_segs_[i].end_s_;
        }
      }
      else if (refline.special_segs_[i].valid_ &&
               refline.special_segs_[i].special_situation_ == ::zone::data::em_data::SpecialSituation::kLinkSapaEnter)
      {
        if (has_service_zone)
        {
          temp_service_zone_start = std::min(temp_service_zone_start, refline.special_segs_[i].start_s_);
          temp_service_zone_end = std::max(temp_service_zone_end, refline.special_segs_[i].end_s_);
        }
        else
        {
          has_service_zone = bc::true_v;
          temp_service_zone_start = refline.special_segs_[i].start_s_;
          temp_service_zone_end = refline.special_segs_[i].end_s_;
        }
      }
    }
    if ((has_tollbooth && ((temp_start >= kTollBoothEndDistance && temp_start <= kTollBoothStartDistance) ||
                           (temp_end <= kTollBoothStartDistance && temp_end >= kTollBoothEndDistance) ||
                           (temp_start <= 0.f && temp_end >= 0.f))) ||
        (has_service_zone &&
         ((temp_service_zone_start >= kTollBoothEndDistance && temp_service_zone_start <= kTollBoothStartDistance) ||
          (temp_service_zone_end <= kTollBoothStartDistance && temp_service_zone_end >= kTollBoothEndDistance) ||
          (temp_service_zone_start <= 0.f && temp_service_zone_end >= 0.f))))
    {
      t_check_status_b = bc::true_v;
    }
    for (bc::uint8_t i = 0; i < ::zone::data::em_data::kMaxMapGuidePtNum; ++i)
    {
      if (map_em_data.map_guide_pts_[i].valid_ &&
          (map_em_data.map_guide_pts_[i].type_ == ::zone::data::em_data::MapGuidePointType::kRemainDistance ||
           map_em_data.map_guide_pts_[i].type_ == ::zone::data::em_data::MapGuidePointType::kHighwayEnd) &&
          (map_em_data.map_guide_pts_[i].end_s_ < FLOAT32_EPSILON))
      {
        t_check_status_b = bc::true_v;
        break;
      }
    }
  }
  if (t_check_status_b)
  {
    // Update tollbooth
    tollbooth_scenario_cs_ = zone::data::em_data::ScenarioClassification();
    if (has_tollbooth && temp_start < 200.f && temp_end > 0.f)
    {
      tollbooth_scenario_cs_.scenario_type_ = zone::data::em_data::ScenarioType::kTollBooth;
      tollbooth_scenario_cs_.start_s_ = temp_start;
      tollbooth_scenario_cs_.end_s_ = temp_end + 20.f;  //增加冗余距离
      tollbooth_scenario_cs_.valid_ = bc::true_v;

      // zone::data::em_data::ReferenceLine& t_refline =
      //     map_em_data.lanes_[::zone::data::em_data::kHostLane].lane_elements_[0].reference_line_;
      // bc::uint8_t t_host_total_num = t_refline.ref_line_valid_pts_;
      // bc::uint8_t t_host_ego_idx = t_refline.current_point_idx_;
      // bc::float32_t t_accumulated_s = 0.f;
      // bc::uint8_t t_accumulated_idx = 0;
      // if (t_refline.ref_line_pts_[t_host_ego_idx].pos_.x > 0.f)
      // {
      //   t_accumulated_s += t_refline.ref_line_pts_[t_host_ego_idx].pos_.x *
      //   t_refline.ref_line_pts_[t_host_ego_idx].pos_.x +
      //                      t_refline.ref_line_pts_[t_host_ego_idx].pos_.y *
      //                      t_refline.ref_line_pts_[t_host_ego_idx].pos_.y;
      // }
      // for (bc::uint8_t i = t_host_ego_idx + 1; i < t_host_total_num; i++)
      // {
      //   if (t_accumulated_s < temp_start)
      //   {
      //     bc::float32_t t_delta_x = t_refline.ref_line_pts_[i].pos_.x - t_refline.ref_line_pts_[i - 1].pos_.x;
      //     bc::float32_t t_delta_y = t_refline.ref_line_pts_[i].pos_.y - t_refline.ref_line_pts_[i - 1].pos_.y;
      //     t_accumulated_s += t_delta_x * t_delta_x + t_delta_y * t_delta_y;
      //     t_accumulated_idx = i;
      //   }
      // }
      // left_boundary_ptn_.x =
      //     map_em_data.lanes_[::zone::data::em_data::kHostLane].lane_elements_[0].left_boundary_.pts_[t_accumulated_idx].x
      //     + 30.f;
      // left_boundary_ptn_.y =
      //     map_em_data.lanes_[::zone::data::em_data::kHostLane].lane_elements_[0].left_boundary_.pts_[t_accumulated_idx].y;
      // right_boundary_ptn_.x =
      //     map_em_data.lanes_[::zone::data::em_data::kHostLane].lane_elements_[0].right_boundary_.pts_[t_accumulated_idx].x
      //     + 30.f;
      // right_boundary_ptn_.y =
      //     map_em_data.lanes_[::zone::data::em_data::kHostLane].lane_elements_[0].right_boundary_.pts_[t_accumulated_idx].y;
    }
    map_em_data = EmAdapterHDmapData();
  }
}

void Preprocess::DiagnosticManagement(const bc::float64_t& time_stamp_f64)
{
  Diag_Event_Info event_info;
  event_info.app_id = IPD_Function_Module::IPD_Fusion_Module;
  event_info.app_version = std::string(GIT_COMMIT);  // + " " + GIT_DATE;
  event_info.time = static_cast<uint64_t>(time_stamp_f64 * 1e3);
  event_info.type = IPD_Diag_type::IPD_EXCEPTION_INFO;

  DiagEventPushBack(input_diag_manager_ar_[kInputChkMap].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkMap].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_EHR_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkEgoMotion].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_EGMOTION_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkPerObj].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_OBJTRACKS_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkPerLane].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_LANEMARKING_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkNavi].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkNavi].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_SD_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkNaviPlus].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_SD_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkPerRdEdge].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkPerRdEdge].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_ROADEDGE_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkPerFreeSp].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkPerFreeSp].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_FREESPACE_MSG_EMPTY_TIMEOUT, event_info);
  DiagEventPushBack(input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_,
                    input_diag_manager_lst1_ar_[kInputChkPerStaticObj].timeout_man_cs_,
                    FUSION_MODULE::E_FUSION_STATICOBJ_MSG_EMPTY_TIMEOUT, event_info);

  if (event_info.event_lists.size() > 0)
  {
    saic::diag::sendEvent(event_info);
  }
}

void Preprocess::DiagEventPushBack(const DiagMan& man_cs, const DiagMan& man_older_cs,
                                   const FUSION_MODULE& event_name_en, Diag_Event_Info& event_info)
{
  if (man_older_cs.GetStatus() == DiagStatus::kErrWrn && man_cs.GetStatus() == DiagStatus::kErrCfm)
  {
    event_info.event_lists.push_back(event_name_en);
  }
  else if (man_older_cs.GetStatus() == DiagStatus::kErrCfm && man_cs.GetStatus() == DiagStatus::kNormal)
  {
    event_info.event_lists.push_back(event_name_en | (1 << 28));
  }
  else
  {
    /// do nothing
  }
}

void Preprocess::OutputDiagInfo(const bc::TCArray<InputDiagInfo, kNumInputChk>& input_diag_manager_ar,
                                EmData& em_output_cs)
{
  bc::uint32_t t_em_diag_status_u32 = 0;
  for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kNumInputChk; t_idx_u8++)
  {
    em_output_cs.em_diag_info_cs_.em_input_diag_ar_[t_idx_u8].timeout_u8_ =
        static_cast<bc::uint8_t>(input_diag_manager_ar[t_idx_u8].timeout_man_cs_.GetStatus());
    em_output_cs.em_diag_info_cs_.em_input_diag_ar_[t_idx_u8].status_u8_ =
        static_cast<bc::uint8_t>(input_diag_manager_ar[t_idx_u8].status_man_cs_.GetStatus());
    if ((input_diag_manager_ar[t_idx_u8].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
         input_diag_manager_ar[t_idx_u8].status_man_cs_.GetStatus() == DiagStatus::kErrCfm) &&
        t_idx_u8 < 32)
    {
      SetBitU32(t_idx_u8, bc::true_v, t_em_diag_status_u32);
    }
  }
  em_output_cs.em_diag_info_cs_.em_diag_status_u32_ = t_em_diag_status_u32;
}

void Preprocess::NaviGuideInfo(const NavigationPlusData& navi_plus_cs, const EmData& em_output_lst1_cs)
{
  GuideAction& t_opt_navi_guide_info_cs = (*em_collection_ptr_).navi_guide_info_.next_guide_action_;
  if (input_diag_manager_ar_[kInputChkNaviPlus].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    FirstTBTGuideInfo t_ipt_navi_guide_info_cs = navi_plus_cs.guide_info_.first_guide_;
    t_opt_navi_guide_info_cs.valid_ = t_ipt_navi_guide_info_cs.valid_;
    t_opt_navi_guide_info_cs.distance_ = t_ipt_navi_guide_info_cs.dist_to_guide_;
    t_opt_navi_guide_info_cs.guide_action_ = t_ipt_navi_guide_info_cs.guide_action_;
  }
  else if (input_diag_manager_ar_[kInputChkNaviPlus].status_man_cs_.GetStatus() == DiagStatus::kErrCfm ||
           input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.GetStatus() == DiagStatus::kErrCfm)
  {
    t_opt_navi_guide_info_cs = GuideAction();
  }
  else
  {
    t_opt_navi_guide_info_cs = em_output_lst1_cs.navi_guide_info_.next_guide_action_;
  }
}

bc::bool_t Preprocess::GetRoadedgeType(RoadEdge& roadedge_cs, LaneLineSource per_roadedge_type_s32)
{
  if (per_roadedge_type_s32 < 0)
  {
    return bc::false_v;
  }
  else
  {
    bc::uint8_t t_per_roadedge_type_index_u8 = 0;
    bc::uint32_t t_per_roadedge_type_u32 = (bc::uint32_t)per_roadedge_type_s32;
    for (bc::uint8_t t_idx_u8 = 15; t_idx_u8 < 22; t_idx_u8++)
    {
      if (GetBitU32(t_per_roadedge_type_u32, t_idx_u8))
      {
        t_per_roadedge_type_index_u8 = t_idx_u8;
        break;
      }
    }

    switch (t_per_roadedge_type_index_u8)
    {
      case 15:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kCurb;
        break;
      case 16:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kGuardRail;
        break;
      case 17:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kConcreteBarrier;
        break;
      case 18:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kWall;
        break;
      case 19:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kCanopy;
        break;
      case 20:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kConeBucket;
        break;
      case 21:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kOther;
        break;
      default:
        roadedge_cs.segs_[0].type_ = RoadEdgeType::kUnknown;
        break;
    };
    return bc::true_v;
  }
}

void Preprocess::StoreSingleRoadedge(RoadEdge& roadedge_cs, const LaneMarking& per_roadedge_cs, bc::uint8_t line_pos_u8)
{
  if (per_roadedge_cs.line_position == line_pos_u8)
  {
    if(per_roadedge_cs.lines_3d_num > 0){
      roadedge_cs.existence_ = bc::true_v;
    }else {
      roadedge_cs.existence_ = bc::false_v;
    }
    if (roadedge_cs.existence_)
    {
      bc::bool_t t_roadedge_type_b = GetRoadedgeType(roadedge_cs, per_roadedge_cs.line_source);
      if (t_roadedge_type_b)
      {
        bc::uint16_t t_fusion_roadedge_seg_point_num = 0;
        bc::uint16_t t_accum_roadedge_num = 0;
        bc::bool_t t_break_b = bc::false_v;
        for (bc::uint8_t t_line_3d_num_u8 = 0; t_line_3d_num_u8 < per_roadedge_cs.lines_3d_num; ++t_line_3d_num_u8)
        {
          t_fusion_roadedge_seg_point_num = per_roadedge_cs.lines_3d[t_line_3d_num_u8].points_num;
          for (bc::uint8_t t_point_idx_u8 = 0; t_point_idx_u8 < t_fusion_roadedge_seg_point_num; ++t_point_idx_u8)
          {
            if (t_accum_roadedge_num >= kMaxBoundaryPoint)
            {
              t_break_b = bc::true_v;
              break;
            }
            roadedge_cs.pts_[t_accum_roadedge_num].x =
                per_roadedge_cs.lines_3d[t_line_3d_num_u8].points[t_point_idx_u8].x;
            roadedge_cs.pts_[t_accum_roadedge_num].y =
                per_roadedge_cs.lines_3d[t_line_3d_num_u8].points[t_point_idx_u8].y;
            t_accum_roadedge_num += 1;
          }
          if (t_break_b)
          {
            break;
          }
        }
        if (t_accum_roadedge_num == 0)
        {
          roadedge_cs.existence_ = bc::false_v;
        }
        else
        {
          roadedge_cs.segs_[0].valid_ = roadedge_cs.existence_;
          roadedge_cs.segs_[0].start_idx_ = 0;
          roadedge_cs.segs_[0].end_idx_ = t_accum_roadedge_num - 1;
        }
      }
      else
      {
        roadedge_cs = RoadEdge();
      }
    }
    else
    {
      roadedge_cs = RoadEdge();
    }
  }
  else
  {
    roadedge_cs = RoadEdge();
  }
}

void Preprocess::RoadedgeStore(const LaneMarkings& per_lane_cs)
{
  RoadEdge& roadedge_l = (*em_collection_ptr_).left_road_edge_;
  RoadEdge& roadedge_r = (*em_collection_ptr_).right_road_edge_;
  if (input_diag_manager_ar_[kInputChkPerLane].status_man_cs_.GetStatus() == DiagStatus::kNormal &&
      input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.GetStatus() == DiagStatus::kNormal)
  {
    StoreSingleRoadedge(roadedge_l, per_lane_cs.lines[kLeftRoadedgeIndex], kLeftRoadedgeLinePosition);
    StoreSingleRoadedge(roadedge_r, per_lane_cs.lines[kRightRoadedgeIndex], kRightRoadedgeLinePosition);
  }
  if (ehr_analy_b_)
  {
    if (map_adapter_cs_.left_road_edge_.existence_)
    {
      for (bc::uint8_t i = 0; i < kMaxBoundarySegment; ++i)
      {
        if (map_adapter_cs_.left_road_edge_.segs_[i].valid_)
        {
          bc::uint8_t t_seg_start_idx = map_adapter_cs_.left_road_edge_.segs_[i].start_idx_;
          if (map_adapter_cs_.left_road_edge_.pts_[t_seg_start_idx].x > x_ref_ar_[kMaxNumBoundary - 1])
          {
            map_adapter_cs_.left_road_edge_.segs_[i] = ::zone::data::em_data::RodeEdgeSegment();
          }
          else
          {
            bc::bool_t t_find_left_end_idx = bc::false_v;
            for (bc::uint8_t j = t_seg_start_idx;
                 j <= map_adapter_cs_.left_road_edge_.segs_[i].end_idx_ && !t_find_left_end_idx; ++j)
            {
              if (map_adapter_cs_.left_road_edge_.pts_[j].x > x_ref_ar_[kMaxNumBoundary - 1])
              {
                map_adapter_cs_.left_road_edge_.segs_[i].end_idx_ = j;
                t_find_left_end_idx = bc::true_v;
              }
            }
          }
        }
      }
    }
    if (map_adapter_cs_.right_road_edge_.existence_)
    {
      for (bc::uint8_t i = 0; i < kMaxBoundarySegment; ++i)
      {
        if (map_adapter_cs_.right_road_edge_.segs_[i].valid_)
        {
          bc::uint8_t t_seg_start_idx = map_adapter_cs_.right_road_edge_.segs_[i].start_idx_;
          if (map_adapter_cs_.right_road_edge_.pts_[t_seg_start_idx].x > x_ref_ar_[kMaxNumBoundary - 1])
          {
            map_adapter_cs_.right_road_edge_.segs_[i] = ::zone::data::em_data::RodeEdgeSegment();
          }
          else
          {
            bc::bool_t t_find_right_end_idx = bc::false_v;
            for (bc::uint8_t j = t_seg_start_idx;
                 j <= map_adapter_cs_.right_road_edge_.segs_[i].end_idx_ && !t_find_right_end_idx; ++j)
            {
              if (map_adapter_cs_.right_road_edge_.pts_[j].x > x_ref_ar_[kMaxNumBoundary - 1])
              {
                map_adapter_cs_.right_road_edge_.segs_[i].end_idx_ = j;
                t_find_right_end_idx = bc::true_v;
              }
            }
          }
        }
      }
    }
    roadedge_l = map_adapter_cs_.left_road_edge_;
    roadedge_r = map_adapter_cs_.right_road_edge_;
  }
}
InputDiagInfo Preprocess::GetInputDiagInfo(const bc::uint8_t idx_input_u8)
{
  InputDiagInfo t_diag_cs = InputDiagInfo();
  if (idx_input_u8 < kNumInputChk)
  {
    t_diag_cs = input_diag_manager_ar_[idx_input_u8];
  }
  return t_diag_cs;
}
}  // namespace environment_model
}  // namespace zone
