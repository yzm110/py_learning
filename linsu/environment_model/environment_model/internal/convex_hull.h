#ifndef CONVEX_HULL
#define CONVEX_HULL

#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/basic_type.h"
#include "common/data/em_data.h"
#include "common/data/em_param.h"
#include "common/data/zone_shared_data.h"
#include "common/data/ego_motion_data.h"
#include "ego_pose.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>



namespace zone {
namespace environment_model {

static const bc::uint8_t kMaxPointNum = 64;
using ::zone::common::EmParam;
using ::zone::common::Point3D;



struct ConvexHullInfo
{
  bc::float32_t dx_f_ = 0.f;
  bc::float32_t dy_f_ = 0.f;
  bc::float32_t heading_f = 0.f;
  bc::float32_t width_f = 0.f;
  bc::float32_t length_f = 0.f;
  bc::bool_t valid_b_ = bc::false_v;
};

class ConvexHull
{
 public:
  explicit ConvexHull(bc::uint8_t method_flag) : method_flag_(method_flag){};

  bc::bool_t Solve(const bc::TFixedVector<Point3D, kMaxPointNum>& pts, ConvexHullInfo& convex_hull, const EgoPose& ego_motion);

  bc::bool_t SolveWithEVD(ConvexHullInfo& convex_hull);

  bc::bool_t SolveWithSVD(ConvexHullInfo& convex_hull);

 private:
  bc::uint8_t method_flag_;
  bc::TFixedVector<Point3D, kMaxPointNum> pts_;
};
}  // namespace environment_model
}  // namespace zone

#endif