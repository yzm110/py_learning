#ifndef POINT_BASED_EM_ENVIRONMENT_MODEL_H
#define POINT_BASED_EM_ENVIRONMENT_MODEL_H
#define CFG_intersection_recognition 0

#include <algorithm>
#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/component.h"
#include "common/time/time.h"

#include "common/data/ego_motion_data.h"
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
#include "common/data/em_dbg_data.h"
#include "common/data/em_param.h"
#include "common/data/horizon_static_object.h"
#include "common/data/navigation_data.h"
#include "common/data/navigation_plus_data.h"
#include "common/data/p_fusion_freespace.h"
#include "common/data/p_fusion_road_edge.h"
#include "common/data/p_horizon_fusion_lane.h"
#include "common/data/p_horizon_fusion_object.h"
#include "common/data/p_horizon_workcondition.h"
#include "common/data/prediction_data.h"
#include "common/data/situation_analysis_data.h"
// #include "common/data/slam_data.h"

#include "asf/log/log.h"
#include "environment_model_internal.h"
#include "hlmf/hlmf.h"
#include "olr/olr.h"
#include "preprocess/preprocess.h"
#include "static_obj_process/sop.h"
#include "weight_distribution/weight_distribution.h"
#if CFG_intersection_recognition == 1
#include "urban_scenario_recognition/isr.h"
#endif

namespace zone {
namespace environment_model {

using ::zone::common::Time;
using ::zone::common::EgoMotionData;
using ::zone::common::EhrToEmData;
using ::zone::common::navi::NavigationData;
using ::zone::common::navi_plus::NavigationPlusData;
using ::zone::data::em_data::EMFreeSpaceData;
using ::zone::data::em_data::EMFreeSpacePoint;
using ::zone::data::em_data::kMaxStaticObjNum;
using ::zone::data::em_data::kFreePointsNum;
using ::zone::data::em_data::MapGuidePointType;
// using ::zone::data::em_data::kSlamPointsNum;
// using ::zone::data::em_data::KSlamSlotNum;
using ::zone::data::em_data::kMaxScenarioNum;
using ::zone::data::em_data::EmStaticObject;
using ::zone::data::em_data::LaneTransitionDirection;
using ::zone::data::em_data::ScenarioType;
using ::zone::data::em_data::EMTrajPointVSlamRef;
// using ::zone::data::em_data::EMVSlamData;
// using ::zone::data::em_data::EMSlotVSlam;
// using ::zone::common::slam::ReceiveLocalizationOutput;
// using ::zone::common::slam::ReceiveLocalizationStart;
// using ::zone::common::slam::ReceiveLocalizationTrajInfo;
// using ::zone::common::slam::ReceiveLocalizationSlotInfo;
using ::zone::common::StaticObjectData;
using ::zone::common::horizon::WorkCondition;
using ::zone::common::EmParam;
using ::zone::common::Em2DBGData;

class EnvironmentModel : public Component
{
 public:
  EnvironmentModel();
  ~EnvironmentModel() = default;

  bc::bool_t SetInput(
      const ObjTracks& objs, const LaneMarkings& lane_markings, const StaticObjectData& static_obj,
      const FreespaceList& freespace, const EgoMotionData& ego_motion, const EhrToEmData& ehr2em,
      const NavigationData& navi, const NavigationPlusData& navi_plus, const WorkCondition& world_info,
      // const ReceiveLocalizationOutput& slam_loc_output, const ReceiveLocalizationStart& slam_loc_start,
      const bc::float64_t time_stamp_f64);

  void SetParam(const EmParam& param_st);

  void SetTimeStamp(const bc::float64_t time_stamp_f64);

  bc::bool_t GetOutput(EmData& em_data_cs_);
  bc::bool_t GetDbgData(Em2DBGData& dbg_data);

  void PerOppositeLaneFlag();
  void MapOppositeLaneFlag();

  /******************************************************************************************/
  /** function*/
  virtual void InitUser() override;
  virtual void RunUser() override;
  bc::float32_t CalcCycleTime(const bc::float64_t time_stamp_f64);
  void HostLaneProcess(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar, EgoPoseCollection& ego_motion_cs,
                       LaneStEndIdxPack& lane_st_end_idx);
  void PostProcess(EgoPoseCollection& ego_motion_cs);
  void UpdateLaneChangeStatus();
  void UpdateScenario();
  void UpdateGeoFenceType();
  void UpdatePreferBoundary(EgoPoseCollection& ego_motion_cs);
  void SpeedLimitProcess();
  void FillInPerSpdLmt();
  void FillInHDMapSpdLmt();
  void FillInSDMapSpdLmt();
  /******************************************************************************************/
  /** input and output classes */
  ObjTracks per_objs_cs_;
  LaneMarkings per_lane_cs_;
  StaticObjectData per_static_obj_cs_;
  FreespaceList per_freespace_cs_;
  EgoMotionData ego_motion_cs_;
  EhrToEmData ehr2em_cs_;
  NavigationData navi_cs_;
  NavigationPlusData navi_plus_cs_;
  WorkCondition world_info_cs_;
  // ReceiveLocalizationOutput slam_data_;
  // ReceiveLocalizationStart slam_start_data_;
  EmParam param_st_;
  bc::TCArray<bc::float32_t, kMaxNumBoundary> x_ref_ar_;
  EmData em_output_cs_;
  EmData em_output_lst_cycle_cs_;
  // EmAdapterHDmapData em_adapter_hdmap_cs_;
  Em2DBGData em_dbg_data_;

  /******************************************************************************************/
  /** internal classes and structures */
  Preprocess preprocess_cs_;
  BoundaryPack boundary_pack_cs_;
  WeightPack weight_pack_cs_;
  SemanticPack semantic_pack_cs_;
  WeightDistribution weight_distribution_cs_;
  Hlmf hlmf_cs_;
  Olr olr_cs_;
  StaticObjProcess static_obj_process_cs_;
#if CFG_intersection_recognition == 1
  IntersectionRecognition intersection_recognition_cs_;
#endif
  /******************************************************************************************/
  /** arrays and vectors */
  bc::TCArray<bc::bool_t, kNumInputChk> input_update_ar_;
  /******************************************************************************************/
  /** variables */
  bc::float64_t time_last_cycle_f64_;
  bc::float32_t time_cycle_f_;
  bc::uint32_t em_error_code_u32_;
  StEndIdxPack last_em_st_end_idx_;
  bc::int8_t map_lane_change_s8_ = 0;
  bc::uint16_t map_host_elem_id_u16_ = 0;
  bc::bool_t pass_toll_booth_b_ = bc::false_v;

 private:
  void GenerateHostLane(const bc::TCArray<bc::float32_t, kMaxNumBoundary>& x_ref_ar, bc::float32_t lane_width_f,
                        bc::TCArray<bc::float32_t, 4> coeff_ar);
  void FillInAgentsData();

  void FillInFreeSpaceData();

  // void FillInVSlamData();
};
}
}
#endif
