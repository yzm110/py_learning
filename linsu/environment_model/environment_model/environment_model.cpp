#include "environment_model.h"

namespace zone {
namespace environment_model {

EnvironmentModel::EnvironmentModel()
    : Component("EnvironmentModel"),
      per_objs_cs_(),
      per_lane_cs_(),
      per_static_obj_cs_(),
      per_freespace_cs_(),
      ego_motion_cs_(),
      ehr2em_cs_(),
      navi_cs_(),
      navi_plus_cs_(),
      world_info_cs_(),
      param_st_(),
      x_ref_ar_(),
      em_output_cs_(),
      em_output_lst_cycle_cs_(),
      em_dbg_data_(),
      preprocess_cs_(em_output_cs_),
      boundary_pack_cs_(),
      weight_pack_cs_(),
      semantic_pack_cs_(),
      weight_distribution_cs_(),
      hlmf_cs_(em_output_cs_, em_output_lst_cycle_cs_),
      olr_cs_(em_output_cs_, em_output_lst_cycle_cs_),
      static_obj_process_cs_(em_output_cs_),
#if CFG_intersection_recognition == 1
      intersection_recognition_cs_(em_output_cs_),
#endif
      input_update_ar_(),
      time_last_cycle_f64_(),
      time_cycle_f_(),
      em_error_code_u32_(),
      last_em_st_end_idx_(),
      map_lane_change_s8_(),
      map_host_elem_id_u16_()
{
  time_last_cycle_f64_ = 0.f;
  time_cycle_f_ = 0.f;
  for (bc::uint8_t i = 0; i < kNumInputChk; i++)
  {
    input_update_ar_[i] = bc::false_v;
  }

  //   zone::sovp::asf::log::LogError("environment_model") <<  "#####LogError test";
  // #ifdef STD_COUT_ENABLE
  //   std::cout << "#####COUT_ENABLE test" << std::endl;
  // #endif
  //   zone::sovp::asf::log::LogDebug("environment_model") <<  "#####LogDebug test";
}
bc::bool_t EnvironmentModel::SetInput(const ObjTracks& objs, const LaneMarkings& lane_markings,
                                      const StaticObjectData& static_obj, const FreespaceList& freespace,
                                      const EgoMotionData& ego_motion, const EhrToEmData& ehr2em,
                                      const NavigationData& navi, const NavigationPlusData& navi_plus,
                                      const WorkCondition& world_info,
                                      // const ReceiveLocalizationOutput& slam_loc_output,
                                      // const ReceiveLocalizationStart& slam_loc_start,
                                      const bc::float64_t time_stamp_f64)
{
  /** Check if each input is updated, set result into input_update_ar_.
   * TODO: Should check timestamp rolling rule. If timestamp is always accumulated, following logic is OK.
   */
  if (time_stamp_f64 < time_last_cycle_f64_)
  {
    InitUser();
    time_cycle_f_ = 0.001f;
    em_error_code_u32_ = 1U;
  }
  else
  {
    if (objs.frame_id > per_objs_cs_.frame_id)
    {
      per_objs_cs_ = objs;
      input_update_ar_[kInputChkPerObj] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkPerObj] = bc::false_v;
    }

    if (lane_markings.frame_id > per_lane_cs_.frame_id)
    {
      per_lane_cs_ = lane_markings;
      input_update_ar_[kInputChkPerLane] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkPerLane] = bc::false_v;
    }

    if (static_obj.time_stamp_f64_ > per_static_obj_cs_.time_stamp_f64_)
    {
      per_static_obj_cs_ = static_obj;
      input_update_ar_[kInputChkPerStaticObj] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkPerStaticObj] = bc::false_v;
    }

    if (navi.timestamp_ > navi_cs_.timestamp_)
    {
      navi_cs_ = navi;
      input_update_ar_[kInputChkNavi] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkNavi] = bc::false_v;
    }

    if (navi_plus.timestamp_ > navi_plus_cs_.timestamp_)
    {
      navi_plus_cs_ = navi_plus;
      input_update_ar_[kInputChkNaviPlus] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkNaviPlus] = bc::false_v;
    }

    if (freespace.timeStamp > per_freespace_cs_.timeStamp)
    {
      per_freespace_cs_ = freespace;
      input_update_ar_[kInputChkPerFreeSp] = bc::true_v;
    }
    else
    {
      input_update_ar_[kInputChkPerFreeSp] = bc::false_v;
    }

    if (ego_motion.timestamp > ego_motion_cs_.timestamp)
    {
      ego_motion_cs_ = ego_motion;
      input_update_ar_[kInputChkEgoMotion] = bc::true_v;
    }
    else
    {
      ego_motion_cs_.timestamp = ego_motion.timestamp;
      input_update_ar_[kInputChkEgoMotion] = bc::false_v;
    }

    if (ehr2em.to_em_data_id_ > ehr2em_cs_.to_em_data_id_)
    {
      ehr2em_cs_ = ehr2em;
      input_update_ar_[kInputChkMap] = bc::true_v;
    }
    else
    {
      ehr2em_cs_.to_em_data_id_ = ehr2em.to_em_data_id_;
      ehr2em_cs_.return_code_[0] = ehr2em.return_code_[0];
      ehr2em_cs_.return_code_[1] = ehr2em.return_code_[1];
      ehr2em_cs_.return_code_[2] = ehr2em.return_code_[2];
      ehr2em_cs_.return_code_[3] = ehr2em.return_code_[3];
      input_update_ar_[kInputChkMap] = bc::false_v;
    }

    if (world_info.timestamp_ > world_info_cs_.timestamp_)
    {
      world_info_cs_ = world_info;
      input_update_ar_[kInputChkWorldInfo] = bc::true_v;
    }
    else
    {
      ego_motion_cs_.timestamp = ego_motion.timestamp;
      input_update_ar_[kInputChkWorldInfo] = bc::false_v;
    }

    /** 2. calc cycle time */
    time_cycle_f_ = CalcCycleTime(time_stamp_f64);

    // slam_data_ = slam_loc_output;
    // slam_start_data_ = slam_loc_start;
  }
  return bc::true_v;
}

void EnvironmentModel::SetParam(const EmParam& param_st)
{
  param_st_ = param_st;
  /** Set error confirm/healing threshold time for each signal */
  preprocess_cs_.input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_ehr_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkMap].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_ehr_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_ego_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkEgoMotion].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_ego_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_obj_track_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerObj].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_obj_track_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_lane_marking_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerLane].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_lane_marking_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkNavi].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_speed_limit_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkNavi].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_speed_limit_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_navi_plus_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkNaviPlus].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_navi_plus_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerRdEdge].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_road_edge_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerRdEdge].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_road_edge_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerFreeSp].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_free_space_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerFreeSp].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_free_space_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_.SetTimeCfmThrd(
      param_st_.diag_man_param_st_.p_time_cfm_thrd_speed_static_obj_f_);
  preprocess_cs_.input_diag_manager_ar_[kInputChkPerStaticObj].timeout_man_cs_.SetTimeHealThrd(
      param_st_.diag_man_param_st_.p_time_heal_thrd_speed_static_obj_f_);
}

void EnvironmentModel::SetTimeStamp(const bc::float64_t time_stamp_f64) { time_last_cycle_f64_ = time_stamp_f64; }

bc::bool_t EnvironmentModel::GetOutput(EmData& em_data_cs_)
{
  em_data_cs_ = em_output_cs_;
  return bc::true_v;
}
bc::bool_t EnvironmentModel::GetDbgData(Em2DBGData& dbg_data)
{
  preprocess_cs_.GetDbgData(em_dbg_data_.pre_cs_);
  hlmf_cs_.GetDbgData(em_dbg_data_.pre_cs_);
  dbg_data = em_dbg_data_;
  return bc::true_v;
}
void EnvironmentModel::InitUser()
{
  time_last_cycle_f64_ = 0.0;
  em_output_cs_ = EmData();
  em_output_lst_cycle_cs_ = EmData();
  preprocess_cs_.Reset();
  hlmf_cs_.Reset(em_output_cs_, em_output_lst_cycle_cs_);
  olr_cs_.Reset(em_output_cs_, em_output_lst_cycle_cs_);
  em_dbg_data_ = Em2DBGData();
  boundary_pack_cs_ = BoundaryPack();
  weight_pack_cs_ = WeightPack();
  semantic_pack_cs_ = SemanticPack();
  weight_distribution_cs_ = WeightDistribution();
  time_cycle_f_ = 0.f;
  em_error_code_u32_ = 0U;
  last_em_st_end_idx_ = StEndIdxPack();
  for (bc::uint8_t i = 0; i < kNumInputChk; i++)
  {
    input_update_ar_[i] = bc::false_v;
  }
}

void EnvironmentModel::RunUser()
{
  bc::float64_t start_time = Time::now();
  bc::float64_t current_time;
  bc::float64_t last_time = start_time;
  // std::cout << "!!!!!EM start" << std::endl;
  em_output_cs_ = EmData();
  /** 1. initialize internal variables */
  preprocess_cs_.Run(per_objs_cs_, per_lane_cs_, per_static_obj_cs_, per_freespace_cs_, ego_motion_cs_, ehr2em_cs_,
                     navi_cs_, navi_plus_cs_, world_info_cs_, em_output_lst_cycle_cs_, param_st_, input_update_ar_,
                     time_cycle_f_, last_em_st_end_idx_, time_last_cycle_f64_, boundary_pack_cs_, semantic_pack_cs_,
                     em_output_cs_, static_obj_process_cs_);
  current_time = Time::now();
  em_output_cs_.reserve_[1] = current_time - last_time;
  em_output_cs_.reserve_[10] = time_cycle_f_;
  last_time = current_time;
  // std::cout << "!!!!!Finish Preprocess (reserve1)time:   " << em_output_cs_.reserve_[1] << "s" << std::endl;
  /** 2. weight distribution*/
  x_ref_ar_ = preprocess_cs_.getCoordinateRef();
  weight_distribution_cs_.Run(x_ref_ar_, boundary_pack_cs_, param_st_, semantic_pack_cs_,
                              preprocess_cs_.getEgoPoseCollection(), weight_pack_cs_);
  current_time = Time::now();
  em_output_cs_.reserve_[2] = current_time - last_time;
  last_time = current_time;
  // std::cout << "!!!!!Finish WeightDistribution (reserve2)time:  " << em_output_cs_.reserve_[2] << "s" << std::endl;

  /** 3. hlmf*/
  hlmf_cs_.SetInput(weight_distribution_cs_.LdvehCenOutput());
  hlmf_cs_.Run(x_ref_ar_, boundary_pack_cs_, weight_pack_cs_, semantic_pack_cs_, last_em_st_end_idx_,
               preprocess_cs_.getEgoPoseCollection(), em_output_lst_cycle_cs_, param_st_, time_cycle_f_);
  current_time = Time::now();
  em_output_cs_.reserve_[3] = current_time - last_time;
  last_time = current_time;
  // std::cout << "!!!!!Finish HLMF (reserve3)time:  " << em_output_cs_.reserve_[3] << " s" << std::endl;

  /** 4. PostProcess*/
  map_lane_change_s8_ = preprocess_cs_.getMapLaneChangeDir();
  map_host_elem_id_u16_ = preprocess_cs_.GetMapHostLaneElementId();
  /// ref相对地图的移动
  em_output_cs_.reserve_[20] = map_lane_change_s8_;
  /// 地图相对上一个cycle的移动
  em_output_cs_.reserve_[21] = preprocess_cs_.getRefLaneChangeDir();
  em_output_cs_.reserve_[22] = map_host_elem_id_u16_;

  UpdateLaneChangeStatus();

  current_time = Time::now();
  em_output_cs_.reserve_[4] = current_time - last_time;
  // std::cout << "!!!!!Finish PostProcess (reserve4)time:  " << em_output_cs_.reserve_[6] << " s" << std::endl;

  /** 5. HostLaneProcess*/
  HostLaneProcess(x_ref_ar_, preprocess_cs_.getEgoPoseCollection(), last_em_st_end_idx_.lane_st_end_idx_[kHostLane]);
  current_time = Time::now();
  em_output_cs_.reserve_[5] = current_time - last_time;
  last_time = current_time;
  // std::cout << "!!!!!Finish HostLaneProcess (reserve5)time:  " << em_output_cs_.reserve_[4] << " s" << std::endl;

  /** 7. olr*/
  olr_cs_.SetInput(time_cycle_f_);
  olr_cs_.SetParam(param_st_.olr_param);
  olr_cs_.Run();

  // static_obj_process_cs_.StaticObjLaneAssignment();
  static_obj_process_cs_.SetTimeCycle(time_cycle_f_);
  static_obj_process_cs_.Run(preprocess_cs_.GetInputDiagInfo(kInputChkPerStaticObj),
                             preprocess_cs_.getEgoPoseCollection(), per_static_obj_cs_, semantic_pack_cs_);
  static_obj_process_cs_.SetOutput();

#if CFG_intersection_recognition == 1
  intersection_recognition_cs_.SetTimeCycle(time_cycle_f_);
  intersection_recognition_cs_.Run(preprocess_cs_.GetInputDiagInfo(kInputChkPerStaticObj),
                                   preprocess_cs_.getEgoPoseCollection(), static_obj_process_cs_);
#endif
  PostProcess(preprocess_cs_.getEgoPoseCollection());

  current_time = Time::now();
  em_output_cs_.timestamp_ = current_time;
  em_output_cs_.reserve_[6] = current_time - last_time;
  // std::cout << "!!!!!Finish OLR (reserve6)time:  " << em_output_cs_.reserve_[5] << " s" << std::endl;

  em_output_cs_.reserve_[0] = current_time - start_time;
  em_output_cs_.reserve_[7] = max(em_output_cs_.reserve_[0], em_output_lst_cycle_cs_.reserve_[7]);
  // std::cout << "!!!!!Finish EM (reserve0)total time:     " << em_output_cs_.reserve_[0] << " s" << std::endl;
  // std::cout << " (reserve7)max time:     " << em_output_cs_.reserve_[7] << " s" << std::endl;

  /** 7. Store last cycle variables*/
  em_output_lst_cycle_cs_ = em_output_cs_;

  /** Debug Purpose Only */
  em_output_cs_.reserve_[30] = preprocess_cs_.GetMapDiagTime();
  em_output_cs_.reserve_[31] = preprocess_cs_.GetMapDiagStatus();
  em_output_cs_.reserve_[32] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.c0_weight_.w_map_cs_.w_norm_f_;
  em_output_cs_.reserve_[33] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.c0_weight_.w_per_cs_.w_norm_f_;
  em_output_cs_.reserve_[34] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.c0_weight_.w_ref_cs_.w_norm_f_;
  em_output_cs_.reserve_[35] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.c0_weight_.w_ldveh_cs_.w_norm_f_;
  em_output_cs_.reserve_[36] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.point_weight_ar_[21].w_map_cs_.w_norm_f_;
  em_output_cs_.reserve_[37] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.point_weight_ar_[21].w_per_cs_.w_norm_f_;
  em_output_cs_.reserve_[38] =
      weight_pack_cs_.lane_weight_ar_[2].element_weight_ar_[0].left_weight_ar_.point_weight_ar_[21].w_ref_cs_.w_norm_f_;
  em_output_cs_.reserve_[39] = weight_pack_cs_.lane_weight_ar_[2]
                                   .element_weight_ar_[0]
                                   .left_weight_ar_.point_weight_ar_[21]
                                   .w_ldveh_cs_.w_norm_f_;

  em_output_cs_.reserve_[23] = preprocess_cs_.GetRawMapElmId0();
  em_output_cs_.reserve_[24] = preprocess_cs_.GetRawMapElmId1();
  em_output_cs_.reserve_[25] = preprocess_cs_.GetRawMapElmId2();
  em_output_cs_.reserve_[26] = preprocess_cs_.GetRawMapElmId3();
  em_output_cs_.reserve_[27] = preprocess_cs_.GetRawMapElmId4();

  em_output_cs_.reserve_[50] = ehr2em_cs_.return_code_[0];
  em_output_cs_.reserve_[51] = ehr2em_cs_.return_code_[1];
  em_output_cs_.reserve_[52] = ehr2em_cs_.return_code_[2];
  em_output_cs_.reserve_[53] = ehr2em_cs_.return_code_[3];
  em_output_cs_.reserve_[54] = ehr2em_cs_.to_em_data_id_;
  em_output_cs_.reserve_[55] = ehr2em_cs_.host_lane_idx_;
  em_output_cs_.reserve_[60] = semantic_pack_cs_.c0;
  em_output_cs_.reserve_[61] = semantic_pack_cs_.c1;
  em_output_cs_.reserve_[62] = semantic_pack_cs_.c2;
  em_output_cs_.reserve_[63] = semantic_pack_cs_.c3;
  em_output_cs_.reserve_[64] =
      0.5 * (per_lane_cs_.lines[0].lines_3d[0].y_coeffs[0] + per_lane_cs_.lines[1].lines_3d[0].y_coeffs[0]);
  em_output_cs_.reserve_[65] =
      0.5 * (per_lane_cs_.lines[0].lines_3d[0].y_coeffs[1] + per_lane_cs_.lines[1].lines_3d[0].y_coeffs[1]);
  em_output_cs_.reserve_[66] =
      0.5 * (per_lane_cs_.lines[0].lines_3d[0].y_coeffs[2] + per_lane_cs_.lines[1].lines_3d[0].y_coeffs[2]);
  em_output_cs_.reserve_[67] =
      0.5 * (per_lane_cs_.lines[0].lines_3d[0].y_coeffs[3] + per_lane_cs_.lines[1].lines_3d[0].y_coeffs[3]);

  em_output_cs_.em_diag_info_cs_.em_map_per_cross_check_info_.host_left_map_per_similiarity_f_ =
      preprocess_cs_.GetLeftMapPerSimilarityData();
  em_output_cs_.em_diag_info_cs_.em_map_per_cross_check_info_.host_right_map_per_similiarity_f_ =
      preprocess_cs_.GetRightMapPerSimilarityData();
  em_output_cs_.em_diag_info_cs_.em_map_per_cross_check_info_.host_left_similiarity_check_state_u8_ =
      static_cast<MapPerSimiliarityCheckState>(preprocess_cs_.GetleftMapPerIsCheckedData());
  em_output_cs_.em_diag_info_cs_.em_map_per_cross_check_info_.host_right_similiarity_check_state_u8_ =
      static_cast<MapPerSimiliarityCheckState>(preprocess_cs_.GetrightMapPerIsCheckedData());
  em_output_cs_.reserve_[71] = preprocess_cs_.GetLeftPostP();
  em_output_cs_.reserve_[72] = preprocess_cs_.GetRightPostP();

#if CFG_intersection_recognition == 1
  em_output_cs_.reserve_[11] = intersection_recognition_cs_.its_build_success_b_;
  em_output_cs_.reserve_[12] = intersection_recognition_cs_.its_bound_pts_ar_[0].point_cs_.x;
  em_output_cs_.reserve_[13] = intersection_recognition_cs_.its_bound_pts_ar_[0].point_cs_.y;
  em_output_cs_.reserve_[14] = intersection_recognition_cs_.its_bound_pts_ar_[1].point_cs_.x;
  em_output_cs_.reserve_[15] = intersection_recognition_cs_.its_bound_pts_ar_[1].point_cs_.y;
  em_output_cs_.reserve_[16] = intersection_recognition_cs_.its_bound_pts_ar_[2].point_cs_.x;
  em_output_cs_.reserve_[17] = intersection_recognition_cs_.its_bound_pts_ar_[2].point_cs_.y;
  em_output_cs_.reserve_[18] = intersection_recognition_cs_.its_bound_pts_ar_[3].point_cs_.x;
  em_output_cs_.reserve_[19] = intersection_recognition_cs_.its_bound_pts_ar_[3].point_cs_.y;

  em_output_cs_.reserve_[40] = intersection_recognition_cs_.its_entry_exit_pts_ar_[0].ref_point_cs_.x;
  em_output_cs_.reserve_[42] = intersection_recognition_cs_.its_entry_exit_pts_ar_[0].ref_point_cs_.y;
  em_output_cs_.reserve_[43] = intersection_recognition_cs_.its_entry_exit_pts_ar_[1].ref_point_cs_.x;
  em_output_cs_.reserve_[44] = intersection_recognition_cs_.its_entry_exit_pts_ar_[1].ref_point_cs_.y;
  em_output_cs_.reserve_[45] = intersection_recognition_cs_.its_entry_exit_pts_ar_[2].ref_point_cs_.x;
  em_output_cs_.reserve_[46] = intersection_recognition_cs_.its_entry_exit_pts_ar_[2].ref_point_cs_.y;
  em_output_cs_.reserve_[47] = intersection_recognition_cs_.its_entry_exit_pts_ar_[3].ref_point_cs_.x;
  em_output_cs_.reserve_[48] = intersection_recognition_cs_.its_entry_exit_pts_ar_[3].ref_point_cs_.y;
  em_output_cs_.reserve_[49] = static_cast<bc::uint8_t>(intersection_recognition_cs_.its_manager_cs_.state_en_);

#endif
}

bc::float32_t EnvironmentModel::CalcCycleTime(const bc::float64_t time_stamp_f64)
{
  bc::float32_t t_dt = (bc::float32_t)(time_stamp_f64 - time_last_cycle_f64_);
  time_last_cycle_f64_ = time_stamp_f64;
  t_dt = std::max(t_dt, (bc::float32_t)0.001);
  return (t_dt);
}

void EnvironmentModel::HostLaneProcess(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar,
                                       EgoPoseCollection& ego_motion_cs, LaneStEndIdxPack& lane_st_end_idx)
{
  // Generate host lane in case no other source valid
  if (em_output_cs_.lanes_[kHostLane].lane_valid_ == bc::false_v ||
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].element_valid_ == bc::false_v ||
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].reference_line_.available_ == bc::false_v)
  {
    bc::float32_t t_default_lane_width_f = 2.6f;
    bc::TCArray<bc::float32_t, 4> t_coeff_ar{0.f, 0.f, 0.f, 0.f};
    if (ego_motion_cs.GetCurrentEgoPose().GetValidation() == bc::true_v)
    {
      t_coeff_ar[2] = 0.5 * ego_motion_cs.GetCurrentEgoPose().GetCurvature();
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_ = 0;
      SetBitU8(LaneBoundary::kSrcMaskEgo, bc::true_v,
               em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_);
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kVirtual;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_ = 0;
      SetBitU8(LaneBoundary::kSrcMaskEgo, bc::true_v,
               em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_);
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kVirtual;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.reserve_[3] += 16;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.reserve_[3] += 16;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.matching_id_ = 1;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.matching_id_ = 2;
    }
    else
    {
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.source_type_ = 0;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kUnknown;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.source_type_ = 0;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.segs_[0].boundary_type_ =
          LaneBoundaryType::kUnknown;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.reserve_[3] += 32;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.reserve_[3] += 32;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.matching_id_ = 1;
      em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.matching_id_ = 2;
    }
    GenerateHostLane(x_ref_ar, t_default_lane_width_f, t_coeff_ar);
    em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.ego_point_idx_ = kFrontStartIdx;
    em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_.ego_point_idx_ = kFrontStartIdx;
    lane_st_end_idx.ele_st_end_idx_[0].boundary_end_idx_ = kMaxBoundaryPoint - 1;
    lane_st_end_idx.ele_st_end_idx_[0].boundary_st_idx_ = 0;
    lane_st_end_idx.ele_st_end_idx_[0].boundary_end_x_f_ = x_ref_ar[kMaxBoundaryPoint - 1];
    lane_st_end_idx.ele_st_end_idx_[0].boundary_st_x_f_ = x_ref_ar[0];
    em_output_cs_.lanes_[kRightLane] = LaneData();
    em_output_cs_.lanes_[kLeftLane] = LaneData();
    em_output_cs_.lanes_[kRRLane] = LaneData();
    em_output_cs_.lanes_[kLLLane] = LaneData();
  }
  // Add traffic agent
  FillInAgentsData();
}

void EnvironmentModel::GenerateHostLane(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar,
                                        bc::float32_t lane_width_f, bc::TCArray<bc::float32_t, 4> coeff_ar)
{
  em_output_cs_.lanes_[kHostLane].lane_valid_ = bc::true_v;
  em_output_cs_.lanes_[kHostLane].valid_elements_cnts_ = 1;

  LaneElement& t_host_element_cs = em_output_cs_.lanes_[kHostLane].lane_elements_[0];
  t_host_element_cs.relative_idx_ = 0;
  t_host_element_cs.element_valid_ = bc::true_v;

  ReferenceLine& t_refline_cs = t_host_element_cs.reference_line_;
  t_refline_cs.available_ = bc::true_v;
  t_refline_cs.current_point_idx_ = 20;
  t_refline_cs.ref_line_valid_pts_ = kMaxNumBoundary;

  // Boundary
  for (bc::uint8_t i = 0; i < kMaxNumBoundary; ++i)
  {
    bc::float32_t current_x = x_ref_ar[i];
    bc::float32_t current_heading =
        std::atan(coeff_ar[1] + 2.f * coeff_ar[2] * current_x + 3.f * coeff_ar[3] * current_x * current_x);
    bc::float32_t current_curvature =
        2.f * coeff_ar[2] +
        6.f * coeff_ar[3] * current_x / std::pow(1 + std::tan(current_heading) * std::tan(current_heading), 1.5f);
    t_host_element_cs.left_boundary_.pts_[i].x = current_x - 0.5 * lane_width_f * sinf(current_heading);
    t_host_element_cs.left_boundary_.pts_[i].y = coeff_ar[0] + 0.5 * lane_width_f * cosf(current_heading) +
                                                 coeff_ar[1] * current_x + coeff_ar[2] * current_x * current_x +
                                                 coeff_ar[3] * current_x * current_x * current_x;
    t_host_element_cs.right_boundary_.pts_[i].x = current_x + 0.5 * lane_width_f * sinf(current_heading);
    t_host_element_cs.right_boundary_.pts_[i].y = coeff_ar[0] - 0.5 * lane_width_f * cosf(current_heading) +
                                                  coeff_ar[1] * current_x + coeff_ar[2] * current_x * current_x +
                                                  coeff_ar[3] * current_x * current_x * current_x;
    // Refline points
    t_refline_cs.ref_line_pts_[i].pos_.x = current_x;
    t_refline_cs.ref_line_pts_[i].pos_.y = coeff_ar[0] + coeff_ar[1] * current_x + coeff_ar[2] * current_x * current_x +
                                           coeff_ar[3] * current_x * current_x * current_x;
    t_refline_cs.ref_line_pts_[i].heading_ = current_heading;
    t_refline_cs.ref_line_pts_[i].curvature_ = current_curvature;
    t_refline_cs.ref_line_pts_[i].lane_width_ = lane_width_f;
    if (i >= kFrontStartIdx)
    {
      t_refline_cs.ref_line_pts_[i].valid_ = bc::true_v;
    }
    else
    {
      t_refline_cs.ref_line_pts_[i].valid_ = bc::false_v;
    }
    if (i > 0)
    {
      bc::float32_t t_delta_ref_x = t_refline_cs.ref_line_pts_[i].pos_.x - t_refline_cs.ref_line_pts_[i - 1].pos_.x;
      bc::float32_t t_delta_ref_y = t_refline_cs.ref_line_pts_[i].pos_.y - t_refline_cs.ref_line_pts_[i - 1].pos_.y;
      t_refline_cs.ref_line_pts_[i].s_ = t_refline_cs.ref_line_pts_[i - 1].s_ +
                                         std::sqrt(t_delta_ref_x * t_delta_ref_x + t_delta_ref_y * t_delta_ref_y);
    }
    else if (i == 0)
    {
      t_refline_cs.ref_line_pts_[i].s_ = 0.f;
    }
  }
  t_host_element_cs.left_boundary_.existence_ = bc::true_v;
  t_host_element_cs.left_boundary_.total_valid_number_ = kMaxNumBoundary;
  // t_host_element_cs.left_boundary_.source_type_ = left.source_type_;
  t_host_element_cs.left_boundary_.ego_point_idx_ = 20;

  t_host_element_cs.left_boundary_.segs_[0].valid_ = bc::true_v;
  t_host_element_cs.left_boundary_.segs_[0].start_s_ = 0.f;
  t_host_element_cs.left_boundary_.segs_[0].end_s_ = 100.f;
  // t_host_element_cs.left_boundary_.segs_[0].boundary_type_ = left.type_;

  t_host_element_cs.right_boundary_.existence_ = bc::true_v;
  t_host_element_cs.right_boundary_.total_valid_number_ = kMaxNumBoundary;
  // t_host_element_cs.right_boundary_.source_type_ = right.source_type_;
  t_host_element_cs.right_boundary_.ego_point_idx_ = 20;

  t_host_element_cs.right_boundary_.segs_[0].valid_ = bc::true_v;
  t_host_element_cs.right_boundary_.segs_[0].start_s_ = 0.f;
  t_host_element_cs.right_boundary_.segs_[0].end_s_ = 100.f;
  // t_host_element_cs.right_boundary_.segs_[0].boundary_type_ = right.type_;

  // Refline coeff
  t_refline_cs.coeff_.valid_ = bc::true_v;
  t_refline_cs.coeff_.clothoid_.c0_position = coeff_ar[0];
  t_refline_cs.coeff_.clothoid_.c1_heading_angle = coeff_ar[1];
  t_refline_cs.coeff_.clothoid_.c2_curvature = coeff_ar[2];
  t_refline_cs.coeff_.clothoid_.c3_curvature_derivative = coeff_ar[3];
  t_refline_cs.coeff_.start_x_ = 0.f;
  t_refline_cs.coeff_.end_x_ = x_ref_ar[kMaxNumBoundary - 1];
  t_refline_cs.coeff_.start_s_ = t_refline_cs.ref_line_pts_[0].s_;
  t_refline_cs.coeff_.end_s_ = t_refline_cs.ref_line_pts_[kMaxNumBoundary - 1].s_;

  // Refline heading segs
  t_refline_cs.heading_segs_[0].start_heading_ = std::atan(t_refline_cs.coeff_.clothoid_.c1_heading_angle);
  t_refline_cs.heading_segs_[0].end_heading_ = t_refline_cs.heading_segs_[0].start_heading_;
  t_refline_cs.heading_segs_[0].valid_ = bc::true_v;

  // Refline curvature segs
  t_refline_cs.curvature_segs_[0].start_s_ = t_refline_cs.coeff_.start_s_;
  t_refline_cs.curvature_segs_[0].end_s_ = t_refline_cs.coeff_.end_s_;
  t_refline_cs.curvature_segs_[0].start_curvature_ = 2 * (t_refline_cs.coeff_.clothoid_.c2_curvature);
  t_refline_cs.curvature_segs_[0].end_curvature_ = t_refline_cs.curvature_segs_[0].start_curvature_;
  t_refline_cs.curvature_segs_[0].valid_ = bc::true_v;

  /// Refline width segs
  t_refline_cs.width_segs_[0].start_s_ = t_refline_cs.coeff_.start_s_;
  t_refline_cs.width_segs_[0].end_s_ = t_refline_cs.coeff_.end_s_;
  t_refline_cs.width_segs_[0].start_width_ = lane_width_f;
  t_refline_cs.width_segs_[0].end_width_ = lane_width_f;
  t_refline_cs.width_segs_[0].valid_ = bc::true_v;
}

void EnvironmentModel::FillInAgentsData()
{
  bc::TCArray<FusionObj, kFusMaxObjNum> t_objs_ar = preprocess_cs_.getPerObjCollection().objs_ar_;
  for (bc::uint16_t idx = 0; idx < kFusMaxObjNum; ++idx)
  {
    TrafficAgentData& t_agent_cs = em_output_cs_.agents_[idx];
    if (t_objs_ar[idx].is_valid)
    {
      t_agent_cs.valid_ = bc::true_v;
      t_agent_cs.pos_.x = t_objs_ar[idx].x;
      t_agent_cs.pos_.y = t_objs_ar[idx].y;
      t_agent_cs.pos_.z = 0.f;
      t_agent_cs.id_ = t_objs_ar[idx].id;
      t_agent_cs.idx_in_agents_array_ = idx;
      t_agent_cs.long_vel_relative_ = t_objs_ar[idx].long_velocity_relative;
      t_agent_cs.lat_vel_relative_ = t_objs_ar[idx].lat_velocity_relative;
      t_agent_cs.long_vel_absolute_ = t_objs_ar[idx].long_velocity_abs;
      t_agent_cs.lat_vel_absolute_ = t_objs_ar[idx].lat_velocity_abs;
      t_agent_cs.vel_ = std::hypot(t_agent_cs.long_vel_absolute_, t_agent_cs.lat_vel_absolute_);
      t_agent_cs.long_accel_relative_ = t_objs_ar[idx].long_acceleration_relative;
      t_agent_cs.lat_accel_relative_ = t_objs_ar[idx].lat_acceleration_relative;
      t_agent_cs.long_accel_absolute_ = t_objs_ar[idx].long_acceleration_abs;
      t_agent_cs.lat_accel_absolute_ = t_objs_ar[idx].lat_acceleration_abs;
      t_agent_cs.accel_ = std::hypot(t_agent_cs.long_accel_absolute_, t_agent_cs.lat_accel_absolute_);
      t_agent_cs.yaw_ = t_objs_ar[idx].yaw;
      t_agent_cs.yawrate_ = t_objs_ar[idx].yawrate;
      t_agent_cs.heading_ = t_objs_ar[idx].heading;
      t_agent_cs.length_ = t_objs_ar[idx].length;
      t_agent_cs.width_ = t_objs_ar[idx].width;
      t_agent_cs.is_cipv_ = t_objs_ar[idx].is_cipv;
      t_agent_cs.category_ = t_objs_ar[idx].category;
      t_agent_cs.fusion_source_ = t_objs_ar[idx].fusion_source;
      t_agent_cs.motion_pattern_ = t_objs_ar[idx].motion_pattern;
      t_agent_cs.brake_light_status_ = t_objs_ar[idx].brake_light_status;
      t_agent_cs.turn_light_status_ = t_objs_ar[idx].turn_light_status;
      t_agent_cs.long_position_std_dev_ = t_objs_ar[idx].long_position_std_dev;
      t_agent_cs.lat_position_std_dev_ = t_objs_ar[idx].lat_position_std_dev;
      t_agent_cs.corner_point_ = t_objs_ar[idx].corner_point_cs_;
    }
    else
    {
      t_agent_cs.valid_ = bc::false_v;
    }
  }
}

void EnvironmentModel::PostProcess(EgoPoseCollection& ego_motion_cs)
{
  UpdateScenario();
  UpdatePreferBoundary(ego_motion_cs);
  // CheckHostLane();
  // 纯感知场景判断左侧的车道是否是对向车道
  if (!em_output_cs_.map_geofence_.is_in_map_)
  {
    PerOppositeLaneFlag();
  }
  // 带地图场景判断左侧的车道是否是对向车道
  else
  {
    MapOppositeLaneFlag();
  }
  SpeedLimitProcess();
  UpdateGeoFenceType();
}

void EnvironmentModel::PerOppositeLaneFlag()
{
  LaneBoundaryColor t_left_line_color = LaneBoundaryColor::kUnknown;
  LaneBoundaryColor t_ll_line_color = LaneBoundaryColor::kUnknown;
  for (bc::uint8_t t_seg_idx_u8 = 0; t_seg_idx_u8 < kMaxBoundarySegment; t_seg_idx_u8++)
  {
    if (em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].valid_ &&
        em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].start_s_ <= 0.f &&
        em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].end_s_ > 0.f)
    {
      t_left_line_color =
          em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].color_type_;
    }
    if (em_output_cs_.lanes_[kLeftLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].valid_ &&
        em_output_cs_.lanes_[kLeftLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].start_s_ <= 0.f &&
        em_output_cs_.lanes_[kLeftLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].end_s_ > 0.f)
    {
      t_ll_line_color =
          em_output_cs_.lanes_[kLeftLane].lane_elements_[0].left_boundary_.segs_[t_seg_idx_u8].color_type_;
    }
  }
  if (t_left_line_color == LaneBoundaryColor::kYellow)
  {
    em_output_cs_.lanes_[kLeftLane].opposite_lane_ = em_output_cs_.lanes_[kLeftLane].lane_valid_;
    em_output_cs_.lanes_[kLLLane].opposite_lane_ = em_output_cs_.lanes_[kLLLane].lane_valid_;
  }
  else if (t_ll_line_color == LaneBoundaryColor::kYellow)
  {
    em_output_cs_.lanes_[kLLLane].opposite_lane_ = em_output_cs_.lanes_[kLLLane].lane_valid_;
  }
  else
  {
    // do nothing
  }
}

void EnvironmentModel::MapOppositeLaneFlag()
{
  if (em_output_cs_.lanes_[kLeftLane].lane_valid_)
  {
    bc::uint8_t t_ll_line_source_type = em_output_cs_.lanes_[kLeftLane].lane_elements_[0].left_boundary_.source_type_;
    if (GetBitU8(t_ll_line_source_type, LaneBoundary::kSrcMaskMap) == 1)
    {
      em_output_cs_.lanes_[kLeftLane].opposite_lane_ = bc::false_v;
    }
    else
    {
      em_output_cs_.lanes_[kLeftLane].opposite_lane_ = bc::true_v;
    }
    if (!em_output_cs_.lanes_[kLLLane].lane_valid_)
    {
      // do nothing
    }
    else if (em_output_cs_.lanes_[kLeftLane].opposite_lane_)
    {
      em_output_cs_.lanes_[kLLLane].opposite_lane_ = bc::true_v;
    }
    else
    {
      bc::uint8_t t_lll_line_source_type = em_output_cs_.lanes_[kLLLane].lane_elements_[0].left_boundary_.source_type_;
      if (GetBitU8(t_lll_line_source_type, LaneBoundary::kSrcMaskMap) == 1)
      {
        em_output_cs_.lanes_[kLLLane].opposite_lane_ = bc::false_v;
      }
      else
      {
        em_output_cs_.lanes_[kLLLane].opposite_lane_ = bc::true_v;
      }
    }
  }
  else
  {
    // do nothing
  }
}

void EnvironmentModel::UpdateLaneChangeStatus()
{
  if (em_output_cs_.lanes_[2].lane_valid_ && em_output_cs_.lanes_[2].lane_elements_[0].element_valid_ &&
      em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.available_)
  {
    /// Find left or right lane
    LaneElement right_lane;
    LaneElement left_lane;
    bc::bool_t t_find_right_lane_b = bc::false_v;
    bc::bool_t t_find_left_lane_b = bc::false_v;
    bc::int8_t t_right_change_u8 = 0;
    bc::int8_t t_left_change_u8 = 0;
    /// First right lane
    if (em_output_cs_.lanes_[2].global_lane_idx_ != UINT32_MAX)
    {
      /// 3和2同属一个车道，3不能认为是右车道
      if (em_output_cs_.lanes_[2].global_lane_idx_ == em_output_cs_.lanes_[3].global_lane_idx_)
      {
        if (em_output_cs_.lanes_[2].global_lane_idx_ != em_output_cs_.lanes_[4].global_lane_idx_)
        {
          /// 2id > 4id； 4ID无限大，为感知车道线，也认为是2的右车道
          right_lane = em_output_cs_.lanes_[4].lane_elements_[0];
          t_find_right_lane_b = bc::true_v;
          t_right_change_u8 = kLaneChangeToRR;
        }
      }
      else
      {
        if (em_output_cs_.lanes_[3].global_lane_idx_ == em_output_cs_.lanes_[4].global_lane_idx_ &&
            map_host_elem_id_u16_ != 0 &&
            map_host_elem_id_u16_ == em_output_cs_.lanes_[4].lane_elements_[0].element_id_)
        {
          right_lane = em_output_cs_.lanes_[4].lane_elements_[0];
          t_find_right_lane_b = bc::true_v;
          t_right_change_u8 = kLaneChangeToRR;
        }
        else
        {
          right_lane = em_output_cs_.lanes_[3].lane_elements_[0];
          t_find_right_lane_b = bc::true_v;
          t_right_change_u8 = kLaneChangeToRight;
        }
      }
    }
    else
    {
      right_lane = em_output_cs_.lanes_[3].lane_elements_[0];
      t_find_right_lane_b = bc::true_v;
      t_right_change_u8 = kLaneChangeToRight;
    }
    /// Second left lane
    if (em_output_cs_.lanes_[2].global_lane_idx_ != UINT32_MAX)
    {
      if (em_output_cs_.lanes_[2].global_lane_idx_ == em_output_cs_.lanes_[1].global_lane_idx_)
      {
        if (em_output_cs_.lanes_[2].global_lane_idx_ != em_output_cs_.lanes_[0].global_lane_idx_)
        {
          left_lane = em_output_cs_.lanes_[0].lane_elements_[0];
          t_find_left_lane_b = bc::true_v;
          t_left_change_u8 = kLaneChangeToLL;
        }
      }
      else
      {
        if (em_output_cs_.lanes_[0].global_lane_idx_ == em_output_cs_.lanes_[1].global_lane_idx_ &&
            map_host_elem_id_u16_ != 0 &&
            map_host_elem_id_u16_ == em_output_cs_.lanes_[0].lane_elements_[0].element_id_)
        {
          left_lane = em_output_cs_.lanes_[0].lane_elements_[0];
          t_find_left_lane_b = bc::true_v;
          t_left_change_u8 = kLaneChangeToLL;
        }
        else
        {
          left_lane = em_output_cs_.lanes_[1].lane_elements_[0];
          t_find_left_lane_b = bc::true_v;
          t_left_change_u8 = kLaneChangeToLeft;
        }
      }
    }
    else
    {
      left_lane = em_output_cs_.lanes_[1].lane_elements_[0];
      t_find_left_lane_b = bc::true_v;
      t_left_change_u8 = kLaneChangeToLeft;
    }

    LaneBoundary t_host_right_boundary = em_output_cs_.lanes_[kHostLane].lane_elements_[0].right_boundary_;
    LaneBoundary t_host_left_boundary = em_output_cs_.lanes_[kHostLane].lane_elements_[0].left_boundary_;
    LaneBoundary t_right_left_boundary = right_lane.left_boundary_;
    LaneBoundary t_left_right_boundary = left_lane.right_boundary_;
    // bc::uint8_t t_ego_idx_u8 = t_host_left_boundary.ego_point_idx_;
    bc::int8_t t_lane_change_dir_s8 = 0;
    bc::bool_t t_lane_change_b = bc::false_v;
    bc::uint8_t t_lane_start_idx_u8 = 0;
    bc::uint8_t t_lane_end_idx_u8 = kMaxLaneNum - 1;
    bc::bool_t t_host_right_virtual_b = bc::false_v;
    bc::bool_t t_host_left_virtual_b = bc::false_v;
    bc::bool_t t_right_left_virtual_b = bc::false_v;
    bc::bool_t t_left_right_virtual_b = bc::false_v;
    // std::cout << "change left left y:     " << t_host_left_boundary.pts_[t_ego_idx_u8].y << std::endl;
    // std::cout << "change left right y:     " << t_host_right_boundary.pts_[t_ego_idx_u8].y << std::endl;
    // std::cout << "change left width:     " << (t_host_left_boundary.pts_[t_ego_idx_u8].y -
    // t_host_right_boundary.pts_[t_ego_idx_u8].y) << std::endl;
    for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxBoundarySegment; t_idx_u8++)
    {
      if (t_host_right_boundary.segs_[t_idx_u8].boundary_type_ == LaneBoundaryType::kVirtual &&
          t_host_right_boundary.segs_[t_idx_u8].start_s_ < 0.f && t_host_right_boundary.segs_[t_idx_u8].end_s_ > 0.f)
      {
        t_host_right_virtual_b = bc::true_v;
      }
      if (t_host_left_boundary.segs_[t_idx_u8].boundary_type_ == LaneBoundaryType::kVirtual &&
          t_host_left_boundary.segs_[t_idx_u8].start_s_ < 0.f && t_host_left_boundary.segs_[t_idx_u8].end_s_ > 0.f)
      {
        t_host_left_virtual_b = bc::true_v;
      }
      if (t_right_left_boundary.existence_)
      {
        if (t_right_left_boundary.segs_[t_idx_u8].boundary_type_ == LaneBoundaryType::kVirtual &&
            t_right_left_boundary.segs_[t_idx_u8].start_s_ < 0.f && t_right_left_boundary.segs_[t_idx_u8].end_s_ > 0.f)
        {
          t_right_left_virtual_b = bc::true_v;
        }
      }
      if (t_left_right_boundary.existence_)
      {
        if (t_left_right_boundary.segs_[t_idx_u8].boundary_type_ == LaneBoundaryType::kVirtual &&
            t_left_right_boundary.segs_[t_idx_u8].start_s_ < 0.f && t_left_right_boundary.segs_[t_idx_u8].end_s_ > 0.f)
        {
          t_left_right_virtual_b = bc::true_v;
        }
      }
    }
    /// predict 0.3s ego position  only in per
    Point2D ego_pos_pred;
    bc::uint8_t t_pred_ego_idx = t_host_left_boundary.ego_point_idx_;
    if (em_output_cs_.map_geofence_.is_in_map_)
    {
      ego_pos_pred = Point2D(0.f, 0.f);
      t_pred_ego_idx = kFrontStartIdx;
    }
    else
    {
      ego_pos_pred.x = ego_motion_cs_.twist.linear.x * 0.3;
      ego_pos_pred.y = ego_motion_cs_.twist.linear.y * 0.3;
      for (bc::uint8_t t_pt_idx_u8 = t_host_left_boundary.ego_point_idx_; t_pt_idx_u8 < kMaxBoundaryPoint;
           t_pt_idx_u8++)
      {
        if (x_ref_ar_[t_pt_idx_u8 - 1] < ego_pos_pred.x && x_ref_ar_[t_pt_idx_u8] > ego_pos_pred.x)
        {
          t_pred_ego_idx = t_pt_idx_u8;
          break;
        }
      }
    }
    /** change lane to left or right*/
    /** change lane to right
     * right boundary > 0 + debounce_dy
     * left boundary > lane width + debounce_dy
     */
    if (t_find_right_lane_b &&
        ((t_host_right_boundary.pts_[t_pred_ego_idx].y - ego_pos_pred.y) > kLaneChangeDebounceDy ||
         (t_host_right_virtual_b && t_right_left_boundary.existence_ && t_right_left_virtual_b == bc::false_v &&
          (t_right_left_boundary.pts_[t_pred_ego_idx].y - ego_pos_pred.y) > kLaneChangeDebounceDy)))
    {
      t_lane_change_dir_s8 = t_right_change_u8;
      t_lane_start_idx_u8 = 0 - t_lane_change_dir_s8;
      t_lane_change_b = bc::true_v;
      for (bc::uint8_t t_lane_idx_u8 = t_lane_start_idx_u8; t_lane_idx_u8 < t_lane_end_idx_u8 + 1; t_lane_idx_u8++)
      {
        em_output_cs_.lanes_[t_lane_idx_u8 + t_lane_change_dir_s8] = em_output_cs_.lanes_[t_lane_idx_u8];
        last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8 + t_lane_change_dir_s8] =
            last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8];
      }
      for (bc::uint8_t t_lane_idx_u8 = t_lane_end_idx_u8 + t_lane_change_dir_s8 + 1;
           t_lane_idx_u8 < t_lane_end_idx_u8 + 1; t_lane_idx_u8++)
      {
        em_output_cs_.lanes_[t_lane_idx_u8] = LaneData();
        last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8] = LaneStEndIdxPack();
      }
      // em_output_cs_.host_lane_idx_ += t_lane_change_dir_s8;
    }

    /** change lane to left
     * left boundary < 0 - debounce_dy
     * right boundary < -lane width - debounce_dy
     */
    else if (t_find_left_lane_b &&
             ((t_host_left_boundary.pts_[t_pred_ego_idx].y - ego_pos_pred.y) < -kLaneChangeDebounceDy ||
              (t_host_left_virtual_b && t_left_right_boundary.existence_ && t_left_right_virtual_b == bc::false_v &&
               (t_left_right_boundary.pts_[t_pred_ego_idx].y - ego_pos_pred.y) < -kLaneChangeDebounceDy)))
    {
      t_lane_change_dir_s8 = t_left_change_u8;
      t_lane_end_idx_u8 = kMaxLaneNum - 1 - t_lane_change_dir_s8;
      t_lane_change_b = bc::true_v;
      for (bc::int8_t t_lane_idx_s8 = t_lane_end_idx_u8; t_lane_idx_s8 >= t_lane_start_idx_u8; t_lane_idx_s8--)
      {
        em_output_cs_.lanes_[t_lane_idx_s8 + t_lane_change_dir_s8] = em_output_cs_.lanes_[t_lane_idx_s8];
        last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_s8 + t_lane_change_dir_s8] =
            last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_s8];
      }
      for (bc::uint8_t t_lane_idx_u8 = t_lane_start_idx_u8; t_lane_idx_u8 < t_lane_change_dir_s8; t_lane_idx_u8++)
      {
        em_output_cs_.lanes_[t_lane_idx_u8] = LaneData();
        last_em_st_end_idx_.lane_st_end_idx_[t_lane_idx_u8] = LaneStEndIdxPack();
      }
      // em_output_cs_.host_lane_idx_ += t_lane_change_dir_s8;
    }
    bc::int8_t t_lc_dir_s8 = (abs(t_lane_change_dir_s8) == 2) ? t_lane_change_dir_s8 / 2 : t_lane_change_dir_s8;
    em_output_cs_.ego_lc_cnts_ = em_output_lst_cycle_cs_.ego_lc_cnts_ - t_lc_dir_s8;
    if (t_lane_change_b)
    {
      // em_output_cs_.relat_dest_lane_ -= t_lane_change_dir_s8;
      bc::float32_t temp_1 = em_output_cs_.lanes_[kHostLane + t_lane_change_dir_s8].lane_elements_[0].route_left_dis_ -
                             em_output_cs_.lanes_[kHostLane].lane_elements_[0].route_left_dis_;
      bc::float32_t temp_2 = em_output_cs_.lanes_[kHostLane + t_lane_change_dir_s8].lane_elements_[1].route_left_dis_ -
                             em_output_cs_.lanes_[kHostLane].lane_elements_[0].route_left_dis_;
      if (em_output_cs_.relat_dest_lane_ != 0 ||
          ((em_output_cs_.relat_dest_lane_ == 0) &&
           ((em_output_cs_.lanes_[kHostLane + t_lane_change_dir_s8].lane_elements_[0].element_valid_ &&
             fabsf(temp_1) > FLOAT32_EPSILON) ||
            (em_output_cs_.lanes_[kHostLane + t_lane_change_dir_s8].lane_elements_[0].element_valid_ &&
             fabsf(temp_2) > FLOAT32_EPSILON))))
      {
        em_output_cs_.relat_dest_lane_ -= t_lane_change_dir_s8;
        for (bc::uint8_t i = 0; i < kMaxMapGuidePtNum; ++i)
        {
          if (em_output_cs_.map_guide_pts_[i].valid_ &&
              (em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kMergePoint ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kRoadSplit ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchEntranceRamp ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchExitRamp ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchEntranceJCT ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchExitJCT ||
               em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kSwitchOffway))
          {
            em_output_cs_.map_guide_pts_[i].target_lane_index_ -= t_lane_change_dir_s8;
          }
        }
      }
      for (bc::uint8_t t_pt_idx_u8 = 0; t_pt_idx_u8 < kFusMaxObjNum; t_pt_idx_u8++)
      {
        if (em_output_cs_.agents_[t_pt_idx_u8].valid_)
        {
          bc::uint8_t t_valid_asso_idx_u8 = 0;
          bc::bool_t t_invalid_associa_idx_b = bc::false_v;
          bc::TCArray<LaneAssociation, kMaxLaneAssociationNum>& t_lane_asso_ar =
              em_output_cs_.agents_[t_pt_idx_u8].lane_association_;
          for (bc::uint8_t t_asso_lane_idx_u8 = 0; t_asso_lane_idx_u8 < kMaxLaneAssociationNum; t_asso_lane_idx_u8++)
          {
            if (t_lane_asso_ar[t_asso_lane_idx_u8].valid_)
            {
              bc::int8_t t_idx_shift_s8 = t_lane_asso_ar[t_asso_lane_idx_u8].assigned_lane_idx_ + t_lane_change_dir_s8;
              if (t_idx_shift_s8 >= kMaxLaneAssociationNum || t_idx_shift_s8 < 0)
              {
                t_lane_asso_ar[t_asso_lane_idx_u8] = LaneAssociation();
                t_invalid_associa_idx_b = bc::true_v;
              }
              else
              {
                if (t_invalid_associa_idx_b)
                {
                  t_lane_asso_ar[t_valid_asso_idx_u8] = t_lane_asso_ar[t_asso_lane_idx_u8];
                  t_lane_asso_ar[t_asso_lane_idx_u8] = LaneAssociation();
                }
                else
                {
                  t_lane_asso_ar[t_asso_lane_idx_u8].assigned_lane_idx_ = (bc::uint8_t)t_idx_shift_s8;
                }
                t_valid_asso_idx_u8++;
              }
            }
          }
        }
      }
    }
    else
    {
      if (map_lane_change_s8_ != 0)
      {
        em_output_cs_.ego_lc_cnts_ = em_output_lst_cycle_cs_.ego_lc_cnts_ - map_lane_change_s8_;
      }
    }
  }
}

void EnvironmentModel::UpdateScenario()
{
  // Update split and merge point
  bc::uint8_t ctn = 0;
  if (em_output_cs_.lanes_[2].lane_valid_ && em_output_cs_.lanes_[2].lane_elements_[0].element_valid_ &&
      em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.available_)
  {
    if (semantic_pack_cs_.nearst_split_scenario_.valid_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == semantic_pack_cs_.nearst_split_scenario_.lane_trans_dir_ &&
            t_current_transition.related_element_id_ == semantic_pack_cs_.nearst_split_scenario_.related_element_id_ &&
            (abs(em_output_cs_.relat_dest_lane_) <= 2 &&
             t_current_transition.related_element_id_ ==
                 em_output_cs_.lanes_[2 - em_output_cs_.relat_dest_lane_].lane_elements_[0].element_id_) &&
            fabsf(semantic_pack_cs_.nearst_split_scenario_.start_s_ - t_current_transition.start_s_) < 0.1f &&
            fabsf(semantic_pack_cs_.nearst_split_scenario_.end_s_ - t_current_transition.end_s_) < 0.1f)
        {
          if ((t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitToRight ||
               t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitFromRight) &&
              em_output_cs_.relat_dest_lane_ <= 0)
          {
            em_output_cs_.scenarios_[ctn].direction_ = 1;
          }
          else if ((t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitToLeft ||
                    t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitFromLeft) &&
                   em_output_cs_.relat_dest_lane_ >= 0)
          {
            em_output_cs_.scenarios_[ctn].direction_ = -1;
          }
          else
          {
            break;
          }
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kSplitPoint;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
          break;
        }
      }
    }
    /// TODO: Improve the following method while change from lane to element
    // Special case that related split point is not in the host lane due to EM shifting left or right
    if ((semantic_pack_cs_.nearst_split_scenario_.valid_ && !em_output_cs_.scenarios_[0].valid_) ||
        !semantic_pack_cs_.nearst_split_scenario_.valid_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_)
        {
          if ((t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitToRight ||
               t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitFromRight) &&
              em_output_cs_.relat_dest_lane_ < 0 &&
              em_output_cs_.lanes_[2].lane_elements_[0].route_left_dis_ > t_current_transition.start_s_ &&
              (t_current_transition.related_element_id_ == em_output_cs_.lanes_[3].lane_elements_[0].element_id_ ||
               em_output_cs_.lanes_[2].lane_elements_[0].element_id_ ==
                   em_output_cs_.lanes_[3]
                       .lane_elements_[0]
                       .reference_line_.lane_transition_dir_segs_[seg_idx]
                       .related_element_id_))
          {
            em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kSplitPoint;
            em_output_cs_.scenarios_[ctn].direction_ = 1;
            em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
            em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
            em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
            em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
            ctn++;
#ifdef STD_COUT_ENABLE
            std::cout << "WARNING in UpdateScenario: Cannot find related split point due to shifting EM model!!!"
                      << std::endl;
#endif
            break;
          }
          else if ((t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitToLeft ||
                    t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kSplitFromLeft) &&
                   em_output_cs_.relat_dest_lane_ > 0 &&
                   em_output_cs_.lanes_[2].lane_elements_[0].route_left_dis_ > t_current_transition.start_s_ &&
                   (t_current_transition.related_element_id_ == em_output_cs_.lanes_[1].lane_elements_[0].element_id_ ||
                    em_output_cs_.lanes_[2].lane_elements_[0].element_id_ ==
                        em_output_cs_.lanes_[1]
                            .lane_elements_[0]
                            .reference_line_.lane_transition_dir_segs_[seg_idx]
                            .related_element_id_))
          {
            em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kSplitPoint;
            em_output_cs_.scenarios_[ctn].direction_ = -1;
            em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
            em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
            em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
            em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
            ctn++;
#ifdef STD_COUT_ENABLE
            std::cout << "WARNING in UpdateScenario: Cannot find related split point due to shifting EM model!!!"
                      << std::endl;
#endif
            break;
          }
        }
      }
    }
    // Recognize merge scenario
    if (em_output_cs_.lanes_[1].lane_valid_ && em_output_cs_.lanes_[1].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[1].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[1].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromRight &&
            (em_output_cs_.relat_dest_lane_ > 0 || em_output_cs_.lanes_[1].lane_elements_[0].merge_flag_) &&
            ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == em_output_cs_.lanes_[2].lane_elements_[0].element_id_)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = -1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = em_output_cs_.lanes_[1].lane_elements_[0].element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
    if (em_output_cs_.lanes_[3].lane_valid_ && em_output_cs_.lanes_[3].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[3].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[3].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromLeft &&
            (em_output_cs_.relat_dest_lane_ < 0 || em_output_cs_.lanes_[3].lane_elements_[0].merge_flag_) &&
            ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == em_output_cs_.lanes_[2].lane_elements_[0].element_id_)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = 1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = em_output_cs_.lanes_[3].lane_elements_[0].element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
    if (em_output_cs_.lanes_[0].lane_valid_ && em_output_cs_.lanes_[0].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[0].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[0].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromRight &&
            (em_output_cs_.relat_dest_lane_ > 0 || em_output_cs_.lanes_[0].lane_elements_[0].merge_flag_) &&
            ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == em_output_cs_.lanes_[2].lane_elements_[0].element_id_)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = -1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = em_output_cs_.lanes_[0].lane_elements_[0].element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
    if (em_output_cs_.lanes_[4].lane_valid_ && em_output_cs_.lanes_[4].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[4].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[4].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromLeft &&
            (em_output_cs_.relat_dest_lane_ < 0 || em_output_cs_.lanes_[4].lane_elements_[0].merge_flag_) &&
            ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == em_output_cs_.lanes_[2].lane_elements_[0].element_id_)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = 1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = em_output_cs_.lanes_[4].lane_elements_[0].element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
    if (em_output_cs_.lanes_[2].lane_valid_ && em_output_cs_.lanes_[2].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeToLeft && ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == 0 && t_current_transition.end_s_ > 0.f)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = -1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
        else if (t_current_transition.valid_ &&
                 t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeToRight &&
                 ctn < kMaxScenarioNum && t_current_transition.related_element_id_ == 0 &&
                 t_current_transition.end_s_ > 0.f)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kMergePoint;
          em_output_cs_.scenarios_[ctn].direction_ = 1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
    // Recognize neighbor merge scenario
    if (em_output_cs_.lanes_[2].lane_valid_ && em_output_cs_.lanes_[2].lane_elements_[0].element_valid_ &&
        em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.available_)
    {
      for (bc::uint8_t seg_idx = 0; seg_idx < kMaxLaneTransitionSegsInOneLaneElement; ++seg_idx)
      {
        RefLineSegLaneTransitionDir& t_current_transition =
            em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.lane_transition_dir_segs_[seg_idx];
        if (t_current_transition.valid_ &&
            t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromLeft && ctn < kMaxScenarioNum &&
            t_current_transition.related_element_id_ == em_output_cs_.lanes_[1].lane_elements_[0].element_id_ &&
            t_current_transition.end_s_ > 0.f)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kNeighborMerge;
          em_output_cs_.scenarios_[ctn].direction_ = -1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
        else if (t_current_transition.valid_ &&
                 t_current_transition.lane_trans_dir_ == LaneTransitionDirection::kMergeFromRight &&
                 ctn < kMaxScenarioNum &&
                 t_current_transition.related_element_id_ == em_output_cs_.lanes_[3].lane_elements_[0].element_id_ &&
                 t_current_transition.end_s_ > 0.f)
        {
          em_output_cs_.scenarios_[ctn].scenario_type_ = ScenarioType::kNeighborMerge;
          em_output_cs_.scenarios_[ctn].direction_ = 1;
          em_output_cs_.scenarios_[ctn].start_s_ = t_current_transition.start_s_;
          em_output_cs_.scenarios_[ctn].end_s_ = t_current_transition.end_s_;
          em_output_cs_.scenarios_[ctn].related_element_id_ = t_current_transition.related_element_id_;
          em_output_cs_.scenarios_[ctn].valid_ = bc::true_v;
          ctn++;
        }
      }
    }
  }
  if (ctn < kMaxScenarioNum)
  {
    em_output_cs_.scenarios_[ctn] = preprocess_cs_.GetTollBoothScenario();
  }
  ctn++;
}

void EnvironmentModel::UpdatePreferBoundary(EgoPoseCollection& ego_motion_cs)
{
  if (em_output_cs_.lanes_[2].lane_valid_ && em_output_cs_.lanes_[2].lane_elements_[0].element_valid_ &&
      em_output_cs_.lanes_[2].lane_elements_[0].reference_line_.available_)
  {
    bc::uint16_t t_left_ego_idx_u16 = em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.ego_point_idx_;
    bc::uint16_t t_right_ego_idx_u16 = em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.ego_point_idx_;
    bc::float32_t t_left_y_f = em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[t_left_ego_idx_u16].y;
    bc::float32_t t_right_y_f = em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[t_right_ego_idx_u16].y;
    // 考虑自车后轴中心距离左右边线的距离
    bc::float32_t t_curv_f = 0.5 * ego_motion_cs.GetCurrentEgoPose().GetCurvature();
    ReferenceLine& t_host_ref_cs = em_output_cs_.lanes_[2].lane_elements_[0].reference_line_;
    bc::uint8_t t_extra_wide_pts_ctn_u8 = 0;
    bc::float32_t t_left_dalta_y_f = 0.f;
    bc::float32_t t_right_dalta_y_f = 0.f;
    bc::uint8_t t_total_pts_ctn_u8 = 0;
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_left_heading_ar = {0.f};
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_left_delta_heading_ar = {0.f};
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_left_kappa_ar = {0.f};
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_right_heading_ar = {0.f};
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_right_delta_heading_ar = {0.f};
    bc::TCArray<bc::float32_t, kMaxNumBoundary> t_right_kappa_ar = {0.f};
    bc::float32_t t_step_f = 0.f;
    bc::float32_t t_left_curv_f = 0.f;
    bc::float32_t t_right_curv_f = 0.f;
    for (bc::uint8_t i = t_host_ref_cs.current_point_idx_;
         i < bc::min(t_host_ref_cs.ref_line_valid_pts_, static_cast<bc::uint16_t>(kMaxRefLinePtsNum - 2)) &&
         t_host_ref_cs.ref_line_pts_[i].pos_.x < 100.f;
         ++i)
    {
      if (t_host_ref_cs.ref_line_pts_[i].lane_width_ > 4.5f)
      {
        t_extra_wide_pts_ctn_u8++;
      }
      t_left_dalta_y_f += fabsf(em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i].y -
                                em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i - 1].y);
      t_right_dalta_y_f += fabsf(em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i].y -
                                 em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i - 1].y);
      t_step_f = em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i + 1].x -
                 em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i].x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_left_heading_ar[t_total_pts_ctn_u8] = (em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i + 1].y -
                                               em_output_cs_.lanes_[2].lane_elements_[0].left_boundary_.pts_[i].y) /
                                              t_step_f;
      if (t_total_pts_ctn_u8 > 0)
      {
        t_left_delta_heading_ar[t_total_pts_ctn_u8 - 1] =
            (t_left_heading_ar[t_total_pts_ctn_u8] - t_left_heading_ar[t_total_pts_ctn_u8 - 1]) / t_step_f;
        t_left_kappa_ar[t_total_pts_ctn_u8 - 1] =
            t_left_delta_heading_ar[t_total_pts_ctn_u8 - 1] /
            (pow(1 + t_left_heading_ar[t_total_pts_ctn_u8 - 1] * t_left_heading_ar[t_total_pts_ctn_u8 - 1], 1.5));
        t_left_curv_f += fabsf(t_left_kappa_ar[t_total_pts_ctn_u8 - 1]);
      }
      t_step_f = em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i + 1].x -
                 em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i].x;
      t_step_f = bc::max(t_step_f, 1.f);
      t_right_heading_ar[t_total_pts_ctn_u8] =
          (em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i + 1].y -
           em_output_cs_.lanes_[2].lane_elements_[0].right_boundary_.pts_[i].y) /
          t_step_f;
      if (t_total_pts_ctn_u8 > 0)
      {
        t_right_delta_heading_ar[t_total_pts_ctn_u8 - 1] =
            (t_right_heading_ar[t_total_pts_ctn_u8] - t_right_heading_ar[t_total_pts_ctn_u8 - 1]) / t_step_f;
        t_right_kappa_ar[t_total_pts_ctn_u8 - 1] =
            t_right_delta_heading_ar[t_total_pts_ctn_u8 - 1] /
            (pow(1 + t_right_heading_ar[t_total_pts_ctn_u8 - 1] * t_right_heading_ar[t_total_pts_ctn_u8 - 1], 1.5));
        t_right_curv_f += fabsf(t_right_kappa_ar[t_total_pts_ctn_u8 - 1]);
      }
      t_total_pts_ctn_u8++;
    }

    if (t_total_pts_ctn_u8 == 0)
    {
      return;
    }
    if (t_extra_wide_pts_ctn_u8 >= 5)
    {
      if (em_output_lst_cycle_cs_.prefer_boundary_ == 0 ||
          em_output_lst_cycle_cs_.ego_lc_cnts_ != em_output_cs_.ego_lc_cnts_)
      {
        // if (t_left_dalta_y_f <= t_right_dalta_y_f)
        // if (t_left_curv_f / t_total_pts_ctn_u8 < 0.002f && t_right_curv_f / t_total_pts_ctn_u8 < 0.002f)
        if (fabsf(t_left_curv_f - t_right_curv_f) / t_total_pts_ctn_u8 < 0.001f)
        {
          // 自车在左右边线之间
          if (t_left_y_f * t_right_y_f <= 0.f)
          {
            if (fabsf(t_left_y_f) <= fabsf(t_right_y_f))
            {
              em_output_cs_.prefer_boundary_ = 1;
            }
            else
            {
              em_output_cs_.prefer_boundary_ = -1;
            }
          }
          // 自车在左边线左边
          else if (t_left_y_f < 0.f)
          {
            em_output_cs_.prefer_boundary_ = 1;
          }
          // 自车在右边线右边
          else
          {
            em_output_cs_.prefer_boundary_ = -1;
          }
        }
        else if (t_left_curv_f <= t_right_curv_f)
        {
          em_output_cs_.prefer_boundary_ = 1;
        }
        else
        {
          em_output_cs_.prefer_boundary_ = -1;
        }
      }
      else if (em_output_lst_cycle_cs_.prefer_boundary_ == 1)
      {
        // if (t_left_dalta_y_f >= t_right_dalta_y_f + t_total_pts_ctn_u8 * 0.5f)
        if (t_left_curv_f >= t_right_curv_f + t_total_pts_ctn_u8 * 0.01f)
        {
          em_output_cs_.prefer_boundary_ = -1;
        }
        else
        {
          em_output_cs_.prefer_boundary_ = 1;
        }
      }
      else if (em_output_lst_cycle_cs_.prefer_boundary_ == -1)
      {
        // if (t_right_dalta_y_f >= t_left_dalta_y_f + t_total_pts_ctn_u8 * 0.5f)
        if (t_right_curv_f >= t_left_curv_f + t_total_pts_ctn_u8 * 0.01f)
        {
          em_output_cs_.prefer_boundary_ = 1;
        }
        else
        {
          em_output_cs_.prefer_boundary_ = -1;
        }
      }
      else
      {
        em_output_cs_.prefer_boundary_ = 0;
      }
    }
    em_output_cs_.reserve_[85] = t_left_dalta_y_f;
    em_output_cs_.reserve_[86] = t_right_dalta_y_f;
    em_output_cs_.reserve_[87] = t_total_pts_ctn_u8;
    em_output_cs_.reserve_[88] = t_extra_wide_pts_ctn_u8;
    em_output_cs_.reserve_[84] = t_left_curv_f;
    em_output_cs_.reserve_[89] = t_right_curv_f;
  }
}

// void EnvironmentModel::FillInVSlamData()
// {
//   EMVSlamData& emvslam_output = em_output_cs_.slam_data_;
//   emvslam_output.x_ = slam_data_.x;
//   emvslam_output.y_ = slam_data_.y;
//   emvslam_output.yaw_ = slam_data_.yaw;
//   emvslam_output.reach_slot_flag_ = slam_data_.reach_slot_flag;
//   emvslam_output.relocalization_ready_ = slam_start_data_.relocalization_ready;
//   for (bc::uint8_t i = 0; i < kSlamPointsNum; ++i)
//   {
//     EMTrajPointVSlamRef& emvslamref_output = emvslam_output.slam_traj_points_ref_[i];
//     const ReceiveLocalizationTrajInfo& emvslamref = slam_data_.local_trajs[i];
//     emvslamref_output.x_ = emvslamref.x;
//     emvslamref_output.y_ = emvslamref.y;
//     emvslamref_output.z_ = emvslamref.z;
//     emvslamref_output.yaw_ = emvslamref.yaw;
//     emvslamref_output.gear_position_ = emvslamref.gear_position;
//     emvslamref_output.front_wheel_angle_ = emvslamref.front_wheel_angle;
//     emvslamref_output.valid_ = emvslamref.reserve;
//   }

//   for (bc::uint8_t i = 0; i < KSlamSlotNum; ++i)
//   {
//     EMSlotVSlam& emvslamslot_output = emvslam_output.slam_slots_[i];
//     const ReceiveLocalizationSlotInfo& emvslamslot = slam_data_.slots[i];
//     emvslamslot_output.valid_ = emvslamslot.valid;
//     for (bc::uint8_t i = 0; i < 4; ++i)
//     {
//       // Point3D temp(emvslamslot.slot_points[i][0],emvslamslot.slot_points[i][1]);
//       Point3D& temp = emvslamslot_output.slot_points_[i];
//       temp.x = emvslamslot.slot_points[i][0];
//       temp.y = emvslamslot.slot_points[i][1];
//       temp.z = 0;
//       // emvslamslot_output
//     }
//   }
// }

void EnvironmentModel::UpdateGeoFenceType()
{
  // Update is_in_map while on urban road or near to destination
  bc::bool_t t_condition_match_b = bc::false_v;
  bc::float32_t t_condition_distance_f = 0.f;
  for (bc::uint8_t i = 0; i < kMaxMapGuidePtNum && em_output_cs_.map_guide_pts_[i].valid_; i++)
  {
    if (em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kRemainDistance
        // || em_output_cs_.map_guide_pts_[i].type_ == MapGuidePointType::kNaviDistance
    )
    {
      if (em_output_cs_.map_guide_pts_[i].start_s_ < 1500.f)
      {
        t_condition_match_b = bc::true_v;
      }
      else
      {
        t_condition_distance_f = em_output_cs_.map_guide_pts_[i].start_s_;
      }
      break;
    }
  }

  if (!pass_toll_booth_b_ && preprocess_cs_.GetTollBoothScenario().valid_ &&
      preprocess_cs_.GetTollBoothScenario().start_s_ < 0.f)
  {
    pass_toll_booth_b_ = bc::true_v;
  }
  else if (pass_toll_booth_b_ && t_condition_distance_f > 1500.f)
  {
    pass_toll_booth_b_ = bc::false_v;
  }

  if (pass_toll_booth_b_ && t_condition_match_b)
  {
    em_output_cs_.map_geofence_.geo_fence_type_ = zone::data::em_data::GeoFenceType::kUrban;
  }
  else if (em_output_cs_.map_geofence_.is_in_map_)
  {
    em_output_cs_.map_geofence_.geo_fence_type_ = zone::data::em_data::GeoFenceType::kHighWay;
  }
}

void EnvironmentModel::FillInFreeSpaceData()
{
  EMFreeSpaceData& freespace_output = em_output_cs_.free_space_data_;
  freespace_output.time_stamp_ = per_freespace_cs_.timeStamp;
  freespace_output.look_index_ = per_freespace_cs_.timeStamp;
  for (bc::uint16_t i = 0; i < kFreePointsNum; ++i)
  {
    const FreespacePoint& freespace_point = per_freespace_cs_.freespacePoints[i];
    EMFreeSpacePoint& freepoint_output = freespace_output.freespace_points_[i];
    freepoint_output.long_position_ = freespace_point.longPosition;
    freepoint_output.lat_position_ = freespace_point.latPosition;
    freepoint_output.reserved_ = freespace_point.reserved;
    freepoint_output.type_ = freespace_point.type;
  }
}

void EnvironmentModel::SpeedLimitProcess()
{
  FillInPerSpdLmt();
  FillInHDMapSpdLmt();
  FillInSDMapSpdLmt();
}

void EnvironmentModel::FillInPerSpdLmt()
{
  // speed limit sign
  StaticObject t_displayed_traffic_spdlmt_sign_cs = semantic_pack_cs_.traffic_spdlmt_sign_ar_[0];
  bc::uint8_t t_displayed_traffic_spdlmt_sign_idx_u8 = 0;
  if (semantic_pack_cs_.traffic_spdlmt_sign_num_u8_ > 1)
  {
    for (bc::uint8_t t_signidx_u8 = 1;
         t_signidx_u8 < std::min(semantic_pack_cs_.traffic_spdlmt_sign_num_u8_, (bc::uint8_t)HORIZON_OBJECT_MAX_NUM);
         t_signidx_u8++)
    {
      if (fabsf(semantic_pack_cs_.traffic_spdlmt_sign_ar_[t_signidx_u8].position_.y_) <
          fabsf(semantic_pack_cs_.traffic_spdlmt_sign_ar_[t_displayed_traffic_spdlmt_sign_idx_u8].position_.y_))
      {
        t_displayed_traffic_spdlmt_sign_idx_u8 = t_signidx_u8;
        t_displayed_traffic_spdlmt_sign_cs = semantic_pack_cs_.traffic_spdlmt_sign_ar_[t_signidx_u8];
      }
    }
  }
  else
  {
    /** Do nothing*/
  }
  em_output_cs_.reserve_[80] = semantic_pack_cs_.traffic_spdlmt_sign_idx_ar_[t_displayed_traffic_spdlmt_sign_idx_u8];
  em_output_cs_.reserve_[81] = t_displayed_traffic_spdlmt_sign_cs.id_;
  if (semantic_pack_cs_.traffic_spdlmt_sign_num_u8_ > 0)
  {
    for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < kMaxLaneNum; t_laneidx_u8++)
    {
      if (em_output_cs_.lanes_[t_laneidx_u8].lane_valid_)
      {
        for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < kMaxElemNumInOneLane; t_elementidx_u8++)
        {
          if (em_output_cs_.lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8].element_valid_)
          {
            RefLineSegSpeedLimitSource& t_speed_limit_per_seg =
                em_output_cs_.lanes_[t_laneidx_u8]
                    .lane_elements_[t_elementidx_u8]
                    .reference_line_.speed_limit_source_segs_[kPerNormalSpdLmtSegIdx];
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtSegIdx].valid_ = bc::true_v;
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtSegIdx].start_s_ =
                t_displayed_traffic_spdlmt_sign_cs.position_.x_;
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtSegIdx].max_spd_limit_ =
                t_displayed_traffic_spdlmt_sign_cs.attr_.value_;
          }
        }
      }
    }
  }
  else
  {
    /** Do nothing*/
  }
  // removal speed limit sign
  StaticObject t_displayed_traffic_spdlmt_rev_sign_cs = semantic_pack_cs_.traffic_spdlmt_rev_sign_ar_[0];
  bc::uint8_t t_displayed_traffic_spdlmt_rev_sign_idx_u8 = 0;
  if (semantic_pack_cs_.traffic_spdlmt_rev_sign_num_u8_ > 1)
  {
    for (bc::uint8_t t_signidx_u8 = 1; t_signidx_u8 < std::min(semantic_pack_cs_.traffic_spdlmt_rev_sign_num_u8_,
                                                               (bc::uint8_t)HORIZON_OBJECT_MAX_NUM);
         t_signidx_u8++)
    {
      if (fabsf(semantic_pack_cs_.traffic_spdlmt_rev_sign_ar_[t_signidx_u8].position_.y_) <
          fabsf(semantic_pack_cs_.traffic_spdlmt_rev_sign_ar_[t_displayed_traffic_spdlmt_rev_sign_idx_u8].position_.y_))
      {
        t_displayed_traffic_spdlmt_rev_sign_idx_u8 = t_signidx_u8;
        t_displayed_traffic_spdlmt_rev_sign_cs = semantic_pack_cs_.traffic_spdlmt_rev_sign_ar_[t_signidx_u8];
      }
    }
  }
  else
  {
    /** Do nothing*/
  }
  em_output_cs_.reserve_[82] =
      semantic_pack_cs_.traffic_spdlmt_rev_sign_idx_ar_[t_displayed_traffic_spdlmt_rev_sign_idx_u8];
  em_output_cs_.reserve_[83] = t_displayed_traffic_spdlmt_rev_sign_cs.id_;
  if (semantic_pack_cs_.traffic_spdlmt_rev_sign_num_u8_ > 0)
  {
    for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < kMaxLaneNum; t_laneidx_u8++)
    {
      if (em_output_cs_.lanes_[t_laneidx_u8].lane_valid_)
      {
        for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < kMaxElemNumInOneLane; t_elementidx_u8++)
        {
          if (em_output_cs_.lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8].element_valid_)
          {
            RefLineSegSpeedLimitSource& t_speed_limit_per_seg =
                em_output_cs_.lanes_[t_laneidx_u8]
                    .lane_elements_[t_elementidx_u8]
                    .reference_line_.speed_limit_source_segs_[kPerNormalSpdLmtSegIdx];
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtRevSegIdx].valid_ = bc::true_v;
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtRevSegIdx].start_s_ =
                t_displayed_traffic_spdlmt_rev_sign_cs.position_.x_;
            t_speed_limit_per_seg.speed_limit_segs_[kPerSpdLmtRevSegIdx].max_spd_limit_ =
                t_displayed_traffic_spdlmt_rev_sign_cs.attr_.value_;
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

void EnvironmentModel::FillInHDMapSpdLmt()
{
  for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < kMaxLaneNum; t_laneidx_u8++)
  {
    if (em_output_cs_.lanes_[t_laneidx_u8].lane_valid_)
    {
      for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < kMaxElemNumInOneLane; t_elementidx_u8++)
      {
        LaneElement& t_element_cs = em_output_cs_.lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
        if (t_element_cs.element_valid_)
        {
          bc::uint8_t t_emspdlmtsegsidx_u8 = 0;
          for (bc::uint8_t t_idx_u8 = 0; t_idx_u8 < kMaxLanePropertySegsInOneLaneElement; ++t_idx_u8)
          {
            RefLineSegSpeedLimit t_hdmap_spdlmtsegs_cs = semantic_pack_cs_.map_semantic_ar_[t_laneidx_u8]
                                                             .lane_semantic_ar_[t_elementidx_u8]
                                                             .speed_limit_segs_[t_idx_u8];
            if (t_hdmap_spdlmtsegs_cs.valid_ && t_hdmap_spdlmtsegs_cs.end_s_ >= 0.f)
            {
              t_element_cs.reference_line_.speed_limit_source_segs_[kHDMapSpdLmtSegIdx]
                  .speed_limit_segs_[t_emspdlmtsegsidx_u8] = t_hdmap_spdlmtsegs_cs;
              t_emspdlmtsegsidx_u8++;
            }
          }
        }
      }
    }
  }
}

void EnvironmentModel::FillInSDMapSpdLmt()
{
  for (bc::uint8_t t_laneidx_u8 = 0; t_laneidx_u8 < kMaxLaneNum; t_laneidx_u8++)
  {
    if (em_output_cs_.lanes_[t_laneidx_u8].lane_valid_)
    {
      for (bc::uint8_t t_elementidx_u8 = 0; t_elementidx_u8 < kMaxElemNumInOneLane; t_elementidx_u8++)
      {
        LaneElement& t_element_cs = em_output_cs_.lanes_[t_laneidx_u8].lane_elements_[t_elementidx_u8];
        if (t_element_cs.element_valid_)
        {
          /** Normal speed limit */
          if (semantic_pack_cs_.max_normal_spdlmt_valid_b_)
          {
            RefLineSegSpeedLimit& t_normal_spdlmtseg_cs =
                t_element_cs.reference_line_.speed_limit_source_segs_[kSDMapNormalSpdLmtSegIdx].speed_limit_segs_[0];
            t_normal_spdlmtseg_cs.valid_ = bc::true_v;
            t_normal_spdlmtseg_cs.max_spd_limit_ = (bc::float32_t)semantic_pack_cs_.max_vel_normal_speed_limit_u8_;
            t_normal_spdlmtseg_cs.start_s_ = (bc::float32_t)semantic_pack_cs_.max_dx_normal_speed_limit_u16_;
            t_normal_spdlmtseg_cs.end_s_ = FLOAT32_MAX;
          }
          else if (semantic_pack_cs_.min_normal_spdlmt_valid_b_)
          {
            RefLineSegSpeedLimit& t_normal_spdlmtseg_cs =
                t_element_cs.reference_line_.speed_limit_source_segs_[kSDMapNormalSpdLmtSegIdx].speed_limit_segs_[0];
            t_normal_spdlmtseg_cs.valid_ = bc::true_v;
            t_normal_spdlmtseg_cs.min_spd_limit_ = (bc::float32_t)semantic_pack_cs_.min_vel_normal_speed_limit_u8_;
            t_normal_spdlmtseg_cs.start_s_ = (bc::float32_t)semantic_pack_cs_.min_dx_normal_speed_limit_u16_;
            t_normal_spdlmtseg_cs.end_s_ = FLOAT32_MAX;
          }
          else
          {
            /** Do nothing*/
          }
          /** Electronic eye speed limit */
          if (semantic_pack_cs_.ele_eye_spdlmt_valid_b_)
          {
            RefLineSegSpeedLimit& t_ele_eye_spdlmtseg_cs =
                t_element_cs.reference_line_.speed_limit_source_segs_[kSDMapEleEyeSpdLmtSegIdx].speed_limit_segs_[0];
            t_ele_eye_spdlmtseg_cs.start_s_ = (bc::float32_t)semantic_pack_cs_.dx_ele_eye_speed_limit_u32_;
            t_ele_eye_spdlmtseg_cs.end_s_ = t_ele_eye_spdlmtseg_cs.start_s_;
            t_ele_eye_spdlmtseg_cs.max_spd_limit_ = (bc::float32_t)semantic_pack_cs_.vel_ele_eye_speed_limit_u8_;
            t_ele_eye_spdlmtseg_cs.min_spd_limit_ = FLOAT32_EPSILON;
            t_ele_eye_spdlmtseg_cs.valid_ = bc::true_v;
          }
          else
          {
            /** Do nothing*/
          }
          /**section speed limit*/
          if (semantic_pack_cs_.intervel_spdlmt_valid_b_)
          {
            RefLineSegSpeedLimit& t_intervel_spdlmtseg_cs =
                t_element_cs.reference_line_.speed_limit_source_segs_[kSDMapSectionSpdLmtSegIdx].speed_limit_segs_[0];
            t_intervel_spdlmtseg_cs.start_s_ = (bc::float32_t)semantic_pack_cs_.dx_interval_starting_point_u16_;
            t_intervel_spdlmtseg_cs.end_s_ = (bc::float32_t)semantic_pack_cs_.dx_interval_ending_point_u16_;
            t_intervel_spdlmtseg_cs.max_spd_limit_ = (bc::float32_t)semantic_pack_cs_.vel_interval_speed_limit_u8_;
            t_intervel_spdlmtseg_cs.min_spd_limit_ = FLOAT32_EPSILON;
            t_intervel_spdlmtseg_cs.valid_ = bc::true_v;
          }
          else
          {
            /** Do nothing*/
          }
        }
      }
    }
  }
}
}  // namespace environment_model
}  // namespace zone
