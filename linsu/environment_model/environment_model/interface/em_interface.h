#ifndef POINT_BASED_EM_INTERFACE_H_INCLULDED_
#define POINT_BASED_EM_INTERFACE_H_INCLULDED_

#include <memory>
#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/ego_motion_data.h"
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
#include "common/data/em_dbg_data.h"
#include "common/data/em_param.h"
#include "common/data/horizon_static_object.h"
#include "common/data/navigation_data.h"
#include "common/data/navigation_plus_data.h"
#include "common/data/p_fusion_freespace.h"
#include "common/data/p_horizon_fusion_lane.h"
#include "common/data/p_horizon_fusion_object.h"
#include "common/data/p_horizon_workcondition.h"
// #include "common/data/slam_data.h"

using namespace std;

namespace zone {
namespace environment_model {

using ::zone::common::EgoMotionData;
using ::zone::common::EmDBGData;
using ::zone::common::Em2DBGData;
using ::zone::data::em_data::EmData;
using ::zone::common::EmParam;
using ::zone::common::EhrToEmData;
using ::zone::common::navi::NavigationData;
using ::zone::common::navi_plus::NavigationPlusData;
using ::zone::fusion::ObjTracks;
using ::zone::fusion::LaneMarkings;
using ::zone::common::StaticObjectData;
using ::zone::common::horizon::WorkCondition;
// using ::zone::common::slam::ReceiveLocalizationOutput;
// using ::zone::common::slam::ReceiveLocalizationStart;

class EnvironmentModel;

class EmInterface
{
 public:
  EmInterface();
  ~EmInterface() = default;
  bc::bool_t SetInput(
      const ObjTracks& objs, const LaneMarkings& lane_markings, const StaticObjectData& static_obj,
      const FreespaceList& freespace, const EgoMotionData& ego_motion, const EhrToEmData& ehr2em,
      const NavigationData& navi, const NavigationPlusData& navi_plus, const WorkCondition& world_info,
      // const ReceiveLocalizationOutput& slam_loc_output, const ReceiveLocalizationStart& slam_loc_start,
      const bc::float64_t time_stamp_f64);
  void SetParam(const EmParam& param);
  bc::bool_t GetOutput(EmData& em_data);
  bc::bool_t GetDbgData(Em2DBGData& dbg_data);
  void Run();
  void SetTimeStamp(const bc::float64_t time_stamp_f64);

 private:
  std::shared_ptr<EnvironmentModel> algo_;
};
}  // environment_model
}  // namespace zone

#endif