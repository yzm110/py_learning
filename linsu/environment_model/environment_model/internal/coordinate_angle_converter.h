#ifndef POINT_BASED_COORDINATE_ANGLE_CONVERTER_HPP
#define POINT_BASED_COORDINATE_ANGLE_CONVERTER_HPP

#include <time.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/basic_type.h"
#include "common/data/em_data.h"
#include "common/data/zone_shared_data.h"
#include "environment_model_internal.h"
#include "math/low_pass_filter.h"

namespace zone {
namespace environment_model {

using namespace std;
using ::zone::common::Point2D;
using ::zone::common::SLTPoint;
using ::zone::common::DEFAULT_DATUM_POINT;
using ::zone::data::em_data::RefLinePoint;
using ::zone::data::em_data::ReferenceLine;
using ::zone::data::em_data::TrafficAgentData;
using ::zone::data::em_data::kMaxRefLinePtsNum;
using ::zone::data::em_data::kFusMaxObjNum;
using ::zone::helper::math::LowPass;

/**
 * @brief Set for mono segments
 *
 * Store the minimum distance in each segment and its index, prepare for the
 * closest point by comparison
 */
class MinDistance
{
 public:
  MinDistance() : gl_index(0), seg_idx(0), Curr_index_in_seg(0), min_distance(0.f) {}
  bc::uint32_t gl_index;           ///< current index in total lane points
  bc::uint32_t seg_idx;            ///< The index of current mono segment
  bc::uint32_t Curr_index_in_seg;  ///< The index of the point corresponding to the
                                   /// minimum distance in the monotonic segment
  bc::float32_t min_distance;      ///< the minimum distance found
};

class CoordinateAngleConverter
{
 public:
  CoordinateAngleConverter(){};

  ~CoordinateAngleConverter(){};

  static bc::bool_t CartesianToFrenetAnglebasedMutiObjects(
      const ReferenceLine& current_refline, const bc::float64_t cycle_time,
      const bc::TFixedVector<bc::uint8_t, ::zone::data::em_data::kFusMaxObjNum>& obj_index,
      const bc::float32_t kSlLLowpass, const bc::float32_t kSlVlLowpass,
      const bc::TFixedVector<RefLinePoint, kMaxRefLinePtsNum>& lane_points,
      const bc::TFixedVector<TrafficAgentData, kFusMaxObjNum>& obj_points,
      const LaneElementMapping& current_element_mapping_info, bc::TFixedVector<SLTPoint, kFusMaxObjNum>& sl_points,
      bc::TFixedVector<RefLinePoint, kFusMaxObjNum>& refline_points,
      bc::TFixedVector<bc::uint8_t, kFusMaxObjNum>& refline_idx, const Point2D& datum_point = DEFAULT_DATUM_POINT);

  static bc::TTriple<SLTPoint, RefLinePoint, bc::uint8_t> CartesianToFrenetAnglebased(
      const ReferenceLine& current_refline, const bc::float64_t cycle_time, const bc::uint8_t obj_index,
      const bc::float32_t kSlLLowpass, const bc::float32_t kSlVlLowpass,
      const bc::TFixedVector<bc::TFixedVector<Point2D, kMaxRefLinePtsNum>, 10>& mono_segs,
      const bc::float32_t& datum_distance, const TrafficAgentData& obj_point,
      const bc::TFixedVector<RefLinePoint, kMaxRefLinePtsNum>& lane_points,
      const LaneElementMapping& current_element_mapping_info);
};
/**
 * @brief Calculate distance of two points p1, p2
 */
inline bc::float32_t distance(const Point2D& p1, const Point2D& p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
};
inline bc::float32_t distance(const Point2D& p1, const TrafficAgentData& p2)
{
  return (p1.x - p2.pos_.x) * (p1.x - p2.pos_.x) + (p1.y - p2.pos_.y) * (p1.y - p2.pos_.y);
};
inline bc::float32_t distance(const TrafficAgentData& p1, const Point2D& p2)
{
  return (p1.pos_.x - p2.x) * (p1.pos_.x - p2.x) + (p1.pos_.y - p2.y) * (p1.pos_.y - p2.y);
};
inline bc::float32_t distance(const Point2D& p1, const RefLinePoint& p2)
{
  return (p1.x - p2.pos_.x) * (p1.x - p2.pos_.x) + (p1.y - p2.pos_.y) * (p1.y - p2.pos_.y);
};
inline bc::float32_t distance(const RefLinePoint& p1, const Point2D& p2)
{
  return (p1.pos_.x - p2.x) * (p1.pos_.x - p2.x) + (p1.pos_.y - p2.y) * (p1.pos_.y - p2.y);
};
inline bc::float32_t distance(const RefLinePoint& p1, const RefLinePoint& p2)
{
  return (p1.pos_.x - p2.pos_.x) * (p1.pos_.x - p2.pos_.x) + (p1.pos_.y - p2.pos_.y) * (p1.pos_.y - p2.pos_.y);
};

/**
 * @brief Divide lane points into several monotonic segments
 * @param ahead lane points
 */
inline bc::TFixedVector<bc::TFixedVector<Point2D, kMaxRefLinePtsNum>, 10> DoMonoCurvePolyFit(
    const bc::TFixedVector<RefLinePoint, kMaxRefLinePtsNum>& ahead)
{
  bc::TFixedVector<bc::TFixedVector<Point2D, kMaxRefLinePtsNum>, 10> t_ahead;
  bc::TFixedVector<Point2D, kMaxRefLinePtsNum> t_mono_seg;
  bc::bool_t monoPositive = bc::true_v;

  if (0 == ahead.size())
  {
    return t_ahead;
  }
  Point2D front_point(ahead.front().pos_.x, ahead.front().pos_.y);
  t_mono_seg.push_back(front_point);
  for (bc::int32_t idx = 1; idx < ahead.size(); ++idx)
  {
    if ((monoPositive == bc::true_v) && (ahead[idx].pos_.x < ahead[idx - 1].pos_.x))
    {
      t_ahead.push_back(t_mono_seg);
      monoPositive = bc::false_v;
      t_mono_seg.clear();
    }
    else if ((monoPositive == bc::false_v) && (ahead[idx].pos_.x > ahead[idx - 1].pos_.x))
    {
      t_ahead.push_back(t_mono_seg);
      monoPositive = bc::true_v;
      t_mono_seg.clear();
    }
    else
    {
      // 单调性不变
    }
    Point2D current_point(ahead[idx].pos_.x, ahead[idx].pos_.y);
    t_mono_seg.push_back(current_point);
  }
  t_ahead.push_back(t_mono_seg);
  return t_ahead;
};

/**
 * @brief Find the foot point from point A3 to segment A1A2
 * @param A1 peivious point
 * @param A2 next point
 * @param A3 object point
 * @details
 * (x-x1)/(y-y1)=(x2-x1)/(y2-y1) \n
 * (x-x3)/(y-y3)*(x2-x1)/(y2-y1) = -1
 */
inline Point2D FootPoint(const RefLinePoint& A1, const RefLinePoint& A2, const TrafficAgentData& A3)
{
  bc::float32_t x1 = A1.pos_.x;
  bc::float32_t y1 = A1.pos_.y;
  bc::float32_t x2 = A2.pos_.x;
  bc::float32_t y2 = A2.pos_.y;
  bc::float32_t x3 = A3.pos_.x;
  bc::float32_t y3 = A3.pos_.y;
  Point2D proj_point(0.f, 0.f);
  /// Calculate the coordinate of projection point
  proj_point.x = (((x2 - x1) * (x2 - x1) * x3 - (y1 - y2) * (x2 - x1) * y3 - (y1 - y2) * (x1 * y2 - y1 * x2)) /
                  ((y1 - y2) * (y1 - y2) + (x2 - x1) * (x2 - x1)));
  proj_point.y = ((-(y1 - y2) * (x2 - x1) * x3 + (y1 - y2) * (y1 - y2) * y3 - (x2 - x1) * (x1 * y2 - y1 * x2)) /
                  ((y1 - y2) * (y1 - y2) + (x2 - x1) * (x2 - x1)));
  return proj_point;
};

inline Point2D FootPoint(const RefLinePoint& A1, const RefLinePoint& A2, const Point2D& A3)
{
  bc::float32_t x1 = A1.pos_.x;
  bc::float32_t y1 = A1.pos_.y;
  bc::float32_t x2 = A2.pos_.x;
  bc::float32_t y2 = A2.pos_.y;
  bc::float32_t x3 = A3.x;
  bc::float32_t y3 = A3.y;
  Point2D proj_point(0.f, 0.f);
  /// Calculate the coordinate of projection point
  proj_point.x = (((x2 - x1) * (x2 - x1) * x3 - (y1 - y2) * (x2 - x1) * y3 - (y1 - y2) * (x1 * y2 - y1 * x2)) /
                  ((y1 - y2) * (y1 - y2) + (x2 - x1) * (x2 - x1)));
  proj_point.y = ((-(y1 - y2) * (x2 - x1) * x3 + (y1 - y2) * (y1 - y2) * y3 - (x2 - x1) * (x1 * y2 - y1 * x2)) /
                  ((y1 - y2) * (y1 - y2) + (x2 - x1) * (x2 - x1)));
  return proj_point;
};

/**
 * @brief Find the point with distance d from A1 on segment A1A2
 */
inline Point2D Find_seg_point(const RefLinePoint& A1, const RefLinePoint& A2, const bc::float32_t d)
{
  Point2D A3(0.f, 0.f);  ///< Point to find
  bc::float32_t proj_delta_x =
      sqrt(d * d * (A2.pos_.x - A1.pos_.x) * (A2.pos_.x - A1.pos_.x) /
           ((A2.pos_.x - A1.pos_.x) * (A2.pos_.x - A1.pos_.x) + (A2.pos_.y - A1.pos_.y) * (A2.pos_.y - A1.pos_.y)));
  bc::float32_t proj_delta_y =
      sqrt(d * d * (A2.pos_.y - A1.pos_.y) * (A2.pos_.y - A1.pos_.y) /
           ((A2.pos_.x - A1.pos_.x) * (A2.pos_.x - A1.pos_.x) + (A2.pos_.y - A1.pos_.y) * (A2.pos_.y - A1.pos_.y)));
  if (A2.pos_.x > A1.pos_.x)
  {
    A3.x = A1.pos_.x + proj_delta_x;
  }
  else
  {
    A3.x = A1.pos_.x - proj_delta_x;
  }
  if (A2.pos_.y > A1.pos_.y)
  {
    A3.y = A1.pos_.y + proj_delta_y;
  }
  else
  {
    A3.y = A1.pos_.y - proj_delta_y;
  }
  return A3;
};

/// datum_point project to lane_points
inline bc::float32_t ProjPointCommon(const RefLinePoint& A1, const RefLinePoint& A2, const RefLinePoint& A3,
                                     const Point2D& A4)
{
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  Point2D a3(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  Point2D proj_point(0.f, 0.f);
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.pos_.x - A1.pos_.x;
  a2.y = A3.pos_.y - A1.pos_.y;
  a3.x = A4.x - A1.pos_.x;
  a3.y = A4.y - A1.pos_.y;
  bc::float32_t dot_prod1 = a1.x * a3.x + a1.y * a3.y;
  bc::float32_t dot_prod2 = a2.x * a3.x + a2.y * a3.y;
  /// same angle
  if (fabsf(dot_prod1 - dot_prod2) <= FLOAT32_EPSILON)
  {
    delta_s = sqrt(distance(A2, A1));
  }
  /// angle <= 90°
  else if (dot_prod1 >= 0.f && dot_prod2 >= 0.f)
  {
    Point2D proj_point1 = FootPoint(A2, A1, A4);
    Point2D proj_point2 = FootPoint(A1, A3, A4);
    bc::float32_t distance1 = distance(proj_point1, A4);
    bc::float32_t distance2 = distance(proj_point2, A4);
    if (fabsf(distance1 - distance2) <= FLOAT32_EPSILON)
    {
      delta_s = sqrt(distance(A2, A1));
    }
    else if (distance1 < distance2)
    {
      proj_point = proj_point1;
      delta_s = sqrt(distance(A2, proj_point));
    }
    else
    {
      proj_point = proj_point2;
      delta_s = sqrt(distance(A2, A1)) + sqrt(distance(A1, proj_point));
    }
  }
  /// angle between a1 and a3 <= 90°, angle between a3 and a2 > 90°
  else if (dot_prod1 >= 0.f)
  {
    proj_point = FootPoint(A1, A2, A4);
    if (fabsf(proj_point.x - A1.pos_.x) <= FLOAT32_EPSILON && fabsf(proj_point.y - A1.pos_.y) <= FLOAT32_EPSILON)
    {
      proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    }
    delta_s = sqrt(distance(A2, proj_point));
  }
  /// angle between a3 and a2 <= 90°, angle between a3 and a1 > 90°
  else if (dot_prod2 >= 0.f)
  {
    proj_point = FootPoint(A1, A3, A4);
    if (fabsf(proj_point.x - A1.pos_.x) <= FLOAT32_EPSILON && fabsf(proj_point.y - A1.pos_.y) <= FLOAT32_EPSILON)
    {
      proj_point = Find_seg_point(A1, A3, FLOAT32_EPSILON);
    }
    delta_s = sqrt(distance(A2, A1)) + sqrt(distance(A1, proj_point));
  }
  else
  {
    delta_s = sqrt(distance(A2, A1));
  }
  return delta_s;
}

inline bc::float32_t ProjPointFirst(const RefLinePoint& A1, const RefLinePoint& A2, const Point2D& A3)
{
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  Point2D proj_point(0.f, 0.f);
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.x - A1.pos_.x;
  a2.y = A3.y - A1.pos_.y;
  bc::float32_t dot_prod = a1.x * a2.x + a1.y * a2.y;
  if (fabsf(dot_prod) <= FLOAT32_EPSILON)
  {
    proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    delta_s = sqrt(distance(proj_point, A1));
  }
  else
  {
    proj_point = FootPoint(A1, A2, A3);
    delta_s = sqrt(distance(proj_point, A1));
  }
  return delta_s;
}

inline bc::float32_t ProjPointLast(const RefLinePoint& A1, const RefLinePoint& A2, const Point2D& A3)
{
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  Point2D proj_point(0.f, 0.f);
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.x - A1.pos_.x;
  a2.y = A3.y - A1.pos_.y;
  bc::float32_t dot_prod = a1.x * a2.x + a1.y * a2.y;
  if (fabsf(dot_prod) <= FLOAT32_EPSILON)
  {
    proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    delta_s = sqrt(distance(proj_point, A2));
  }
  else
  {
    proj_point = FootPoint(A1, A2, A3);
    delta_s = sqrt(distance(proj_point, A2));
  }
  return delta_s;
};

/**
 * @brief Calculate the value of s_ and l_ according to the four points
 * @param A1 minimum point
 * @param A2 previous point
 * @param A3 next point
 * @param A4 object point
 */
inline SLTPoint AngleJudgeCommon(const RefLinePoint& A1, const RefLinePoint& A2, const RefLinePoint& A3,
                                 const TrafficAgentData& A4)
{
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  Point2D a3(0.f, 0.f);
  bc::float32_t lane_theta = 0.f;
  Point2D a4(0.f, 0.f);
  Point2D a5(1.f, 0.f);
  Point2D proj_point(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  bc::float32_t distance_to_line = 0.f;
  SLTPoint delta_s_l;
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.pos_.x - A1.pos_.x;
  a2.y = A3.pos_.y - A1.pos_.y;
  a3.x = A4.pos_.x - A1.pos_.x;
  a3.y = A4.pos_.y - A1.pos_.y;
  bc::float32_t dot_prod1 = a1.x * a3.x + a1.y * a3.y;
  bc::float32_t dot_prod2 = a2.x * a3.x + a2.y * a3.y;
  /// same angle
  if (fabsf(dot_prod1 - dot_prod2) <= FLOAT32_EPSILON)
  {
    proj_point = Point2D(A1.pos_.x, A1.pos_.y);
    delta_s = sqrt(distance(A2, A1));
    a4.x = A3.pos_.x - A2.pos_.x;
    a4.y = A3.pos_.y - A2.pos_.y;
  }
  /// angle <= 90°
  else if (dot_prod1 >= 0.f && dot_prod2 >= 0.f)
  {
    Point2D proj_point1 = FootPoint(A2, A1, A4);
    Point2D proj_point2 = FootPoint(A1, A3, A4);
    bc::float32_t distance1 = distance(proj_point1, A4);
    bc::float32_t distance2 = distance(proj_point2, A4);
    if (fabsf(distance1 - distance2) <= FLOAT32_EPSILON)
    {
      proj_point = Point2D(A1.pos_.x, A1.pos_.y);
      delta_s = sqrt(distance(A2, A1));
      a4.x = A3.pos_.x - A2.pos_.x;
      a4.y = A3.pos_.y - A2.pos_.y;
    }
    else if (distance1 < distance2)
    {
      proj_point = proj_point1;
      delta_s = sqrt(distance(A2, proj_point));
      a4.x = A1.pos_.x - A2.pos_.x;
      a4.y = A1.pos_.y - A2.pos_.y;
    }
    else
    {
      proj_point = proj_point2;
      delta_s = sqrt(distance(A2, A1)) + sqrt(distance(A1, proj_point));
      a4.x = A3.pos_.x - A1.pos_.x;
      a4.y = A3.pos_.y - A1.pos_.y;
    }
  }
  /// angle between a1 and a3 <= 90°, angle between a3 and a2 > 90°
  else if (dot_prod1 >= 0.f)
  {
    proj_point = FootPoint(A1, A2, A4);
    if (fabsf(proj_point.x - A1.pos_.x) <= FLOAT32_EPSILON && fabsf(proj_point.y - A1.pos_.y) <= FLOAT32_EPSILON)
    {
      proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    }
    delta_s = sqrt(distance(A2, proj_point));
    a4.x = A1.pos_.x - A2.pos_.x;
    a4.y = A1.pos_.y - A2.pos_.y;
  }
  /// angle between a3 and a2 <= 90°, angle between a3 and a1 > 90°
  else if (dot_prod2 >= 0.f)
  {
    proj_point = FootPoint(A1, A3, A4);
    if (fabsf(proj_point.x - A1.pos_.x) <= FLOAT32_EPSILON && fabsf(proj_point.y - A1.pos_.y) <= FLOAT32_EPSILON)
    {
      proj_point = Find_seg_point(A1, A3, FLOAT32_EPSILON);
    }
    delta_s = sqrt(distance(A2, A1)) + sqrt(distance(A1, proj_point));
    a4.x = A3.pos_.x - A1.pos_.x;
    a4.y = A3.pos_.y - A1.pos_.y;
  }
  else
  {
    proj_point = Point2D(A1.pos_.x, A1.pos_.y);
    delta_s = sqrt(distance(A2, A1));
    a4.x = A3.pos_.x - A2.pos_.x;
    a4.y = A3.pos_.y - A2.pos_.y;
  }
  double theta_bf_acos_up = (a4.x * a5.x + a4.y * a5.y);
  double theta_bf_acos_down = sqrt((a5.x * a5.x + a5.y * a5.y) * (a4.x * a4.x + a4.y * a4.y));
  double theta_bf_acos = theta_bf_acos_up / theta_bf_acos_down;
  lane_theta = acos(theta_bf_acos);

  bc::float32_t cross_product = a5.x * a4.y - a4.x * a5.y;
  if (cross_product <= 0.f)
  {
    delta_s_l.heading_ = -lane_theta;
  }
  else
  {
    delta_s_l.heading_ = lane_theta;
  }
  distance_to_line = sqrt(distance(A4, proj_point));
  delta_s_l.s_ = delta_s;
  delta_s_l.l_ = distance_to_line;
  return delta_s_l;
};

/**
 * @brief Calculate the value of s_ and l_ while minimum distance point index
 * is 0
 * @param A1 minimum point/previous point 0
 * @param A2 next point  1
 * @param A3 object point
 */
inline SLTPoint AngleJudgeFirst(const RefLinePoint& A1, const RefLinePoint& A2, const TrafficAgentData& A3)
{
  /// A1 min == previous == 0    A2 next ==1   A3 obj
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  bc::float32_t lane_theta = 0.f;
  Point2D a3(0.f, 0.f);
  Point2D a4(1.f, 0.f);
  Point2D proj_point(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  bc::float32_t distance_to_line = 0.f;
  SLTPoint delta_s_l;
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.pos_.x - A1.pos_.x;
  a2.y = A3.pos_.y - A1.pos_.y;
  bc::float32_t dot_prod = a1.x * a2.x + a1.y * a2.y;
  if (fabsf(dot_prod) <= FLOAT32_EPSILON)
  {
    proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    delta_s = sqrt(distance(proj_point, A1));
  }
  else if (dot_prod > 0.f)
  {
    proj_point = FootPoint(A1, A2, A3);
    delta_s = sqrt(distance(proj_point, A1));
  }
  else
  {
    proj_point = FootPoint(A1, A2, A3);
    delta_s = -sqrt(distance(proj_point, A1));
  }
  a3.x = A2.pos_.x - A1.pos_.x;
  a3.y = A2.pos_.y - A1.pos_.y;
  double theta_bf_acos_up = (a3.x * a4.x + a3.y * a4.y);
  double theta_bf_acos_down = sqrt((a3.x * a3.x + a3.y * a3.y) * (a4.x * a4.x + a4.y * a4.y));
  double theta_bf_acos = theta_bf_acos_up / theta_bf_acos_down;
  lane_theta = acos(theta_bf_acos);
  bc::float32_t cross_product = a4.x * a3.y - a3.x * a4.y;
  if (cross_product <= 0.f)
  {
    delta_s_l.heading_ = -lane_theta;
  }
  else
  {
    delta_s_l.heading_ = lane_theta;
  }
  distance_to_line = sqrt(distance(A3, proj_point));
  delta_s_l.s_ = delta_s;
  delta_s_l.l_ = distance_to_line;
  return delta_s_l;
};

/**
 * @brief Calculate the value of s_ and l_ while minimum distance point index
 * is
 * max index
 * @param A1 minimum point/next point 0
 * @param A2 previous point  MAX-1
 * @param A3 object point
 * @details If angle between a1 and a2 = 90°, it means the projected point is
 * one of lane point exactly. In order to avoid confusion with the projection
 * to
 * lane point when the angles are equal or greater than 90°, give a small
 * displacement to it.
 */
inline SLTPoint AngleJudgeLast(const RefLinePoint& A1, const RefLinePoint& A2, const TrafficAgentData& A3)
{
  Point2D a1(0.f, 0.f);
  Point2D a2(0.f, 0.f);
  bc::float32_t lane_theta = 0.f;
  Point2D a3(0.f, 0.f);
  Point2D a4(1.f, 0.f);
  Point2D proj_point(0.f, 0.f);
  bc::float32_t delta_s = 0.f;
  bc::float32_t distance_to_line = 0.f;
  SLTPoint delta_s_l;
  a1.x = A2.pos_.x - A1.pos_.x;
  a1.y = A2.pos_.y - A1.pos_.y;
  a2.x = A3.pos_.x - A1.pos_.x;
  a2.y = A3.pos_.y - A1.pos_.y;
  bc::float32_t dot_prod = a1.x * a2.x + a1.y * a2.y;
  if (fabsf(dot_prod) <= FLOAT32_EPSILON)
  {
    proj_point = Find_seg_point(A1, A2, FLOAT32_EPSILON);
    delta_s = sqrt(distance(proj_point, A2));
  }
  else
  {
    proj_point = FootPoint(A1, A2, A3);
    delta_s = sqrt(distance(proj_point, A2));
  }
  a3.x = A1.pos_.x - A2.pos_.x;
  a3.y = A1.pos_.y - A2.pos_.y;
  double theta_bf_acos_up = (a3.x * a4.x + a3.y * a4.y);
  double theta_bf_acos_down = sqrt((a3.x * a3.x + a3.y * a3.y) * (a4.x * a4.x + a4.y * a4.y));
  double theta_bf_acos = theta_bf_acos_up / theta_bf_acos_down;
  lane_theta = acos(theta_bf_acos);
  bc::float32_t cross_product = a4.x * a3.y - a3.x * a4.y;
  if (cross_product <= 0.f)
  {
    delta_s_l.heading_ = -lane_theta;
  }
  else
  {
    delta_s_l.heading_ = lane_theta;
  }
  distance_to_line = sqrt(distance(A3, proj_point));
  delta_s_l.s_ = delta_s;
  delta_s_l.l_ = distance_to_line;
  return delta_s_l;
};

inline bc::bool_t CoordinateAngleConverter::CartesianToFrenetAnglebasedMutiObjects(
    const ReferenceLine& current_refline, const bc::float64_t cycle_time,
    const bc::TFixedVector<bc::uint8_t, ::zone::data::em_data::kFusMaxObjNum>& obj_index,
    const bc::float32_t kSlLLowpass, const bc::float32_t kSlVlLowpass,
    const bc::TFixedVector<RefLinePoint, kMaxRefLinePtsNum>& lane_points,
    const bc::TFixedVector<TrafficAgentData, kFusMaxObjNum>& obj_points,
    const LaneElementMapping& current_element_mapping_info, bc::TFixedVector<SLTPoint, kFusMaxObjNum>& sl_points,
    bc::TFixedVector<RefLinePoint, kFusMaxObjNum>& refline_points,
    bc::TFixedVector<bc::uint8_t, kFusMaxObjNum>& refline_idx, const Point2D& datum_point)
{
  if (lane_points.size() < 2 || obj_points.size() < 1)
  {
    return bc::false_v;
  }

  bc::float32_t min_datum_to_lane_dis = distance(datum_point, lane_points[0]);
  bc::uint8_t min_index = 0;
  for (bc::uint8_t i = 1; i < lane_points.size(); i++)
  {
    bc::float32_t temp_datum_to_lane_dis = distance(datum_point, lane_points[i]);
    if (temp_datum_to_lane_dis < min_datum_to_lane_dis)
    {
      min_datum_to_lane_dis = temp_datum_to_lane_dis;
      min_index = i;
    }
    else
    {
      continue;
    }
  }
  bc::uint8_t max_index = lane_points.size() - 1;
  bc::uint8_t previous_index = 0;
  bc::uint8_t next_index = 0;
  bc::float32_t datum_distance = 0.f;
  bc::float32_t delta_s = 0.f;

  if (min_index == max_index)
  {
    previous_index = max_index - 1;
    next_index = max_index;
    delta_s = ProjPointLast(lane_points[next_index], lane_points[previous_index], datum_point);
  }
  else if (min_index == 0)
  {
    previous_index = 0;
    next_index = 1;
    delta_s = ProjPointFirst(lane_points[previous_index], lane_points[next_index], datum_point);
  }
  else
  {
    previous_index = min_index - 1;
    next_index = min_index + 1;
    delta_s =
        ProjPointCommon(lane_points[min_index], lane_points[previous_index], lane_points[next_index], datum_point);
  }

  if (previous_index == 0)
  {
    datum_distance = delta_s;
  }
  else
  {
    for (bc::uint16_t i = 1; i <= previous_index; i++)
    {
      datum_distance += sqrt(distance(lane_points[i - 1], lane_points[i]));
    }
    datum_distance += delta_s;
  }

  bc::TFixedVector<bc::TFixedVector<Point2D, kMaxRefLinePtsNum>, 10> mono_segs = DoMonoCurvePolyFit(lane_points);
  /// Deal with each obstacle in turn
  for (bc::uint8_t i = 0; i < obj_points.size(); i++)
  {
    if (obj_points[i].valid_ == bc::false_v)
    {
      break;
    }
    else
    {
      bc::TTriple<SLTPoint, RefLinePoint, bc::uint8_t> turple =
          CartesianToFrenetAnglebased(current_refline, cycle_time, obj_index[i], kSlLLowpass, kSlVlLowpass, mono_segs,
                                      datum_distance, obj_points[i], lane_points, current_element_mapping_info);
      sl_points.push_back(turple.first());
      refline_points.push_back(turple.second());
      refline_idx.push_back(turple.third());
    }
  }
  return bc::true_v;
};

inline bc::TTriple<SLTPoint, RefLinePoint, bc::uint8_t> CoordinateAngleConverter::CartesianToFrenetAnglebased(
    const ReferenceLine& current_refline, const bc::float64_t cycle_time, const bc::uint8_t obj_index,
    const bc::float32_t kSlLLowpass, const bc::float32_t kSlVlLowpass,
    const bc::TFixedVector<bc::TFixedVector<Point2D, kMaxRefLinePtsNum>, 10>& mono_segs,
    const bc::float32_t& datum_distance, const TrafficAgentData& obj_point,
    const bc::TFixedVector<RefLinePoint, kMaxRefLinePtsNum>& lane_points,
    const LaneElementMapping& current_element_mapping_info)
{
  /// Each monotonous segment corresponds to a minimum distance.
  bc::TFixedVector<MinDistance, 10> min_dist;
  bc::uint8_t max_index = lane_points.size() - 1;
  bc::uint8_t seg_total_global_idx = 0;
  /// Find each minimum distance and its index
  MinDistance current_min_dist;
  for (bc::int32_t i = 0; i < mono_segs.size(); ++i)
  {
    current_min_dist.gl_index = seg_total_global_idx;
    current_min_dist.min_distance = distance(mono_segs[i][0], obj_point);
    current_min_dist.seg_idx = i;
    current_min_dist.Curr_index_in_seg = 0;
    for (bc::int32_t j = 1; j < mono_segs[i].size(); ++j)
    {
      bc::float32_t current_distance = distance(mono_segs[i][j], obj_point);
      if (current_distance <= current_min_dist.min_distance)
      {
        current_min_dist.min_distance = current_distance;
        current_min_dist.seg_idx = i;
        current_min_dist.Curr_index_in_seg = j;
        current_min_dist.gl_index = seg_total_global_idx + j;
      }
      else
      {
        break;
      }
    }
    min_dist.push_back(current_min_dist);
    seg_total_global_idx += mono_segs[i].size();
  }

  bc::uint32_t min_index = 0;
  bc::float32_t min_distan = min_dist[0].min_distance;

  /// Compare
  /// Find the minimum index
  for (bc::uint8_t i = 0; i < min_dist.size(); i++)
  {
    if (min_dist[i].min_distance < min_distan)
    {
      min_distan = min_dist[i].min_distance;
      min_index = i;
    }
  }
  bc::uint32_t min_gl_index = min_dist[min_index].gl_index;
  /// find the minimum distance and index,
  /// then find the privious and next point
  bc::uint32_t previous_index = 0;
  bc::uint32_t next_index = 0;
  Point2D proj_point(0.f, 0.f);
  bc::float32_t distance_to_line = 0.f;   // l_
  bc::float32_t distance_to_point = 0.f;  // s_

  SLTPoint delta_s_l;
  if (min_gl_index == max_index)
  {
    previous_index = max_index - 1;
    next_index = max_index;
    delta_s_l = AngleJudgeLast(lane_points[next_index], lane_points[previous_index], obj_point);

    // cout<<"last part:    "<<delta_s_l.heading_<<endl;
    // cout<<"lane_points[next_index]:    "<<lane_points[next_index].pos_.x<<"\t"<<lane_points[next_index].pos_.y<<endl;
    // cout<<"lane_points[previous_index]:
    // "<<lane_points[previous_index].pos_.x<<"\t"<<lane_points[previous_index].pos_.y<<endl;
  }
  else if (min_gl_index == 0)
  {
    previous_index = 0;
    next_index = 1;
    delta_s_l = AngleJudgeFirst(lane_points[previous_index], lane_points[next_index], obj_point);
  }
  else
  {
    previous_index = min_gl_index - 1;
    next_index = min_gl_index + 1;
    delta_s_l =
        AngleJudgeCommon(lane_points[min_gl_index], lane_points[previous_index], lane_points[next_index], obj_point);
  }
  bc::float32_t x1 = lane_points[previous_index].pos_.x;
  bc::float32_t x2 = lane_points[next_index].pos_.x;
  bc::float32_t y1 = lane_points[previous_index].pos_.y;
  bc::float32_t y2 = lane_points[next_index].pos_.y;

  /// The closest point is the first point
  if (previous_index == 0)
  {
    distance_to_point = delta_s_l.s_;
  }
  else
  {
    for (bc::uint16_t i = 1; i <= previous_index; i++)
    {
      distance_to_point += sqrt(distance(lane_points[i - 1], lane_points[i]));
    }
    distance_to_point += delta_s_l.s_;
  }
  /// Cumulative calculation distance s_
  /// Judge s_ positive and negative or 0
  if (fabsf(distance_to_point) <= FLOAT32_EPSILON)
  {
    distance_to_point = FLOAT32_ZERO;
  }
  /// Calculate l_
  distance_to_line = delta_s_l.l_;
  /// Judge l_ positive and negative or 0
  bc::float32_t cross_product = (obj_point.pos_.x - x1) * (y2 - y1) - (x2 - x1) * (obj_point.pos_.y - y1);
  if (cross_product > 0.f)
  {
    distance_to_line = -distance_to_line;
  }
  else if (fabsf(cross_product) <= FLOAT32_EPSILON)
  {
    distance_to_line = FLOAT32_ZERO;
  }
  bc::float64_t delta_theta = obj_point.heading_ - delta_s_l.heading_;
  SLTPoint slt_point;
  slt_point.heading_ = delta_theta;
  /* change calc method:  unsigned --> signed*/
  slt_point.vs_ =
      obj_point.long_vel_absolute_ * cos(delta_s_l.heading_) + obj_point.lat_vel_absolute_ * sin(delta_s_l.heading_);
  slt_point.as_ = obj_point.long_accel_absolute_ * cos(delta_s_l.heading_) +
                  obj_point.lat_accel_absolute_ * sin(delta_s_l.heading_);
  slt_point.al_ = 0.f;  // default value
  slt_point.s_ = distance_to_point - datum_distance;
  if (current_refline.agents_projected_traj_[obj_index].id_ != obj_point.id_)
  {
    slt_point.l_ = distance_to_line;
    slt_point.vl_ = sqrt((obj_point.long_vel_absolute_ * obj_point.long_vel_absolute_ +
                          obj_point.lat_vel_absolute_ * obj_point.lat_vel_absolute_)) *
                    sin(delta_theta);
  }
  else
  {
    if (current_element_mapping_info.element_id_change_b_ && !current_element_mapping_info.is_old_element_b_)
    {
      slt_point.l_ = distance_to_line;
      slt_point.vl_ = sqrt((obj_point.long_vel_absolute_ * obj_point.long_vel_absolute_ +
                            obj_point.lat_vel_absolute_ * obj_point.lat_vel_absolute_)) *
                      sin(delta_theta);
    }
    else
    {
      if (current_refline.agents_projected_traj_[obj_index].valid_)
      {
        bc::float32_t filtered_slt_point_l = LowPass(
            distance_to_line, current_refline.agents_projected_traj_[obj_index].slt_pt_.l_, kSlLLowpass, cycle_time);
        bc::float32_t vl_l =
            (filtered_slt_point_l - current_refline.agents_projected_traj_[obj_index].slt_pt_.l_) / cycle_time;
        bc::float32_t filtered_vl_l =
            LowPass(vl_l, current_refline.agents_projected_traj_[obj_index].slt_pt_.vl_, kSlVlLowpass, cycle_time);
        slt_point.l_ = filtered_slt_point_l;
        slt_point.vl_ = filtered_vl_l;
      }
      else
      {
        slt_point.l_ = distance_to_line;
        slt_point.vl_ = sqrt((obj_point.long_vel_absolute_ * obj_point.long_vel_absolute_ +
                              obj_point.lat_vel_absolute_ * obj_point.lat_vel_absolute_)) *
                        sin(delta_theta);
      }
    }
  }
  /// TODO: Handle invalid situations for Simulation Test & Road Test
  slt_point.valid_ = bc::true_v;
  slt_point.t_ = 0.f;
  if (isnan(slt_point.s_) || isnan(slt_point.l_) || isnan(slt_point.vs_) || isnan(slt_point.vl_) ||
      isnan(slt_point.as_) || isnan(slt_point.al_) || isnan(slt_point.t_))
  {
    // std::cout << "NAN in Environment Model coordinate converter" << std::endl;
  }
  return bc::TTriple<SLTPoint, RefLinePoint, bc::uint8_t>(slt_point, lane_points[min_gl_index], min_gl_index);
};
}
}

#endif /*COORDINATE_ANGLE_CONVERTER_HPP*/
