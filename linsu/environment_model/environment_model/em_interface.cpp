#include "em_interface.h"
#include "environment_model.h"

namespace zone {
namespace environment_model {

EmInterface::EmInterface() : algo_(std::make_shared<EnvironmentModel>()) {}

bc::bool_t EmInterface::SetInput(const ObjTracks& objs, const LaneMarkings& lane_markings,
                                 const StaticObjectData& static_obj, const FreespaceList& freespace,
                                 const EgoMotionData& ego_motion, const EhrToEmData& ehr2em, const NavigationData& navi,
                                 const NavigationPlusData& navi_plus, const WorkCondition& world_info,
                                 // const ReceiveLocalizationOutput& slam_loc_output,
                                 // const ReceiveLocalizationStart& slam_loc_start,
                                 const bc::float64_t time_stamp_f64)
{
  return algo_->SetInput(objs, lane_markings, static_obj, freespace, ego_motion, ehr2em, navi, navi_plus, world_info,
                         // slam_loc_output, slam_loc_start,
                         time_stamp_f64);
}

void EmInterface::SetParam(const EmParam& param) { algo_->SetParam(param); }

bc::bool_t EmInterface::GetOutput(EmData& em_data) { return algo_->GetOutput(em_data); }

void EmInterface::Run() { algo_->Run(); }

void EmInterface::SetTimeStamp(const bc::float64_t time_stamp_f64) { algo_->SetTimeStamp(time_stamp_f64); }

bc::bool_t EmInterface::GetDbgData(Em2DBGData& dbg_data) { return algo_->GetDbgData(dbg_data); }

}  // namespace environment_model
}  // namespace zone