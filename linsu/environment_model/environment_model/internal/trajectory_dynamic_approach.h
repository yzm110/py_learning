#ifndef POINT_BASED_EM_TRAJECTORY_DYNAMIC_APPROACH_H
#define POINT_BASED_EM_TRAJECTORY_DYNAMIC_APPROACH_H

#include "bc/container/bc_container_all.hpp"
#include "bc/core/bc_core_all.hpp"
#include "common/data/em_data.h"
#include "common/data/em_param.h"

namespace zone {
namespace environment_model {

using ::zone::data::em_data::kFusMaxObjNum;
using ::zone::data::em_data::kInvalidObjectId;
using ::zone::data::em_data::kMaxObjectId;

static const bc::uint8_t kMaxTrajectoryPoint = 50;

class Trajectory
{
 public:
  Trajectory() : valid_point_ctns_u8_(0), trajectory_points_ar_()
  {
    for (bc::uint8_t i = 0; i < kMaxTrajectoryPoint; ++i)
    {
      trajectory_points_ar_[i] = Point2D();
    }
  }
  virtual ~Trajectory() = default;

  /** @brief Add new point to index 0 and remove the last one if the container is full
   *  @param new_point_cs (x, y)
   */
  void AddPoint(Point2D new_point_cs)
  {
    if (valid_point_ctns_u8_ > 0)
    {
      // Move each point backforward by one position
      if (valid_point_ctns_u8_ == kMaxTrajectoryPoint)
      {
        for (bc::uint8_t i = valid_point_ctns_u8_ - 1; i >= 1; --i)
        {
          trajectory_points_ar_[i] = trajectory_points_ar_[i - 1];
        }
      }
      else
      {
        for (bc::int8_t i = valid_point_ctns_u8_ - 1; i >= 0; --i)
        {
          trajectory_points_ar_[i + 1] = trajectory_points_ar_[i];
        }
      }
    }
    trajectory_points_ar_[0] = new_point_cs;
    // Update valid_point_ctns_u8_ if the container is not full
    if (valid_point_ctns_u8_ < kMaxTrajectoryPoint)
    {
      valid_point_ctns_u8_++;
    }
  };

  /** @brief Do coordinate transformation on each point in trajectory
   *  @param rotation_f Radian
   *  @param translation_dx_f
   *  @param translation_dy_f
   */
  void CoordTransformation(bc::float32_t rotation_f, bc::float32_t translation_dx_f, bc::float32_t translation_dy_f)
  {
    bc::float32_t t_dx = 0.f;
    bc::float32_t t_dy = 0.f;
    for (bc::uint8_t i = 0; i < valid_point_ctns_u8_; ++i)
    {
      // translation
      trajectory_points_ar_[i].x -= translation_dx_f;
      trajectory_points_ar_[i].y -= translation_dy_f;

      t_dx = cosf(rotation_f) * trajectory_points_ar_[i].x + sinf(rotation_f) * trajectory_points_ar_[i].y;
      t_dy = -sinf(rotation_f) * trajectory_points_ar_[i].x + cosf(rotation_f) * trajectory_points_ar_[i].y;
      trajectory_points_ar_[i].x = t_dx;
      trajectory_points_ar_[i].y = t_dy;
    }
  }

  // bc::uint8_t GetValidPoint() const { return valid_point_ctns_u8_; };
  // Point2D GetTrajectoryPoint(bc::uint8_t point_index_u8) const { return trajectory_points_ar_[point_index_u8]; };
 private:
  bc::uint8_t valid_point_ctns_u8_;
  bc::TCArray<Point2D, kMaxTrajectoryPoint> trajectory_points_ar_;
};

class ObjectTrajectoryCollection
{
 public:
  ObjectTrajectoryCollection() : valid_object_trajectory_ctns_u8_(0), object_id_ar_(), trajectory_collection_ar_()
  {
    for (bc::uint8_t i = 0; i < kFusMaxObjNum; ++i)
    {
      object_id_ar_[i] = kInvalidObjectId;
      trajectory_collection_ar_[i] = Trajectory();
    }
  }
  virtual ~ObjectTrajectoryCollection() = default;

  /** @brief If the given ID exists, set the related trajectory to the reference and return true;
   *  Othervise, return false.
   *  @param id_u16 Object ID
   *  @param trajectory reference of input Trajectory
   *  @return boolean
   */
  bc::bool_t GetTrajectoryById(bc::uint16_t id_u16, Trajectory& trajectory)
  {
    if (id_u16 == kInvalidObjectId || id_u16 >= kMaxObjectId)
    {
      return bc::false_v;
    }
    // Find and set value for given id_u16 in collection
    for (bc::uint8_t i = 0; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      if (object_id_ar_[i] == id_u16)
      {
        trajectory = trajectory_collection_ar_[i];
        return bc::true_v;
      }
    }
    return bc::false_v;
  }

  /** @brief If the given ID exists, add the new point to exists trajectory;
   *  Othervise, add new point to the end of container. If fail to add, return false.
   *  @param id_u16 Object ID
   *  @param point_cs (x, y)
   *  @return boolean
   */
  bc::bool_t AddPointToTrajectoryById(bc::uint16_t id_u16, Point2D point_cs)
  {
    if (id_u16 == kInvalidObjectId || id_u16 >= kMaxObjectId)
    {
      return bc::false_v;
    }
    // Add the point to trajectory if the given id_u16 is already in the collection
    for (bc::uint8_t i = 0U; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      if (object_id_ar_[i] == id_u16)
      {
        trajectory_collection_ar_[i].AddPoint(point_cs);
        return bc::true_v;
      }
    }
    // Add the point to the first empty trajectory if the given id_u16 is not in the collection
    if (valid_object_trajectory_ctns_u8_ < kFusMaxObjNum)
    {
      object_id_ar_[valid_object_trajectory_ctns_u8_] = id_u16;
      trajectory_collection_ar_[valid_object_trajectory_ctns_u8_].AddPoint(point_cs);
      valid_object_trajectory_ctns_u8_++;
      return bc::true_v;
    }
    return bc::false_v;
  }

  /** @brief If the given ID exists, reset the trajectory buffer and return true;
   *  Othervise, do nothing and return false.
   *  @param id_u16 Object ID
   *  @return boolean
   */
  bc::bool_t ResetTrajectoryById(bc::uint16_t id_u16)
  {
    // Invalid object
    if (id_u16 == kInvalidObjectId || id_u16 >= kMaxObjectId)
    {
      return bc::false_v;
    }
    for (bc::uint8_t i = 0U; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      if (object_id_ar_[i] == id_u16)
      {
        // Move rest valid object trajectory one position ahead to fill empty position
        for (; i < valid_object_trajectory_ctns_u8_ - 1; ++i)
        {
          trajectory_collection_ar_[i] = trajectory_collection_ar_[i + 1];
          object_id_ar_[i] = object_id_ar_[i + 1];
        }
        trajectory_collection_ar_[i] = Trajectory();
        object_id_ar_[i] = kInvalidObjectId;
        // Update valid_object_trajectory_ctns_u8_
        valid_object_trajectory_ctns_u8_--;
        return bc::true_v;
      }
    }
    return bc::false_v;
  }

  /** @brief Do coordinate transformation for each valid trajectory;
   *  @param rotation_f
   *  @param translation_dx_f
   *  @param translation_dy_f
   */
  void DoCoordTransToTrajectory(bc::float32_t rotation_f, bc::float32_t translation_dx_f,
                                bc::float32_t translation_dy_f)
  {
    for (bc::uint8_t i = 0; i < valid_object_trajectory_ctns_u8_; ++i)
    {
      trajectory_collection_ar_[i].CoordTransformation(rotation_f, translation_dx_f, translation_dy_f);
    }
  }

  bc::uint8_t GetValidObjectTractoryCtns() const { return valid_object_trajectory_ctns_u8_; }
  bc::TCArray<bc::uint16_t, kFusMaxObjNum> GetObjectIds() { return object_id_ar_; }
 private:
  bc::uint8_t valid_object_trajectory_ctns_u8_;
  bc::TCArray<bc::uint16_t, kFusMaxObjNum> object_id_ar_;
  bc::TCArray<Trajectory, kFusMaxObjNum> trajectory_collection_ar_;
};
}
}
#endif