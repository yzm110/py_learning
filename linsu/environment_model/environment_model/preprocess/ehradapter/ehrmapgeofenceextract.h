#pragma once
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"

class EhrMapGeofenceExtract
{
 private:
  /* data */
 public:
  EhrMapGeofenceExtract(/* args */);
  ~EhrMapGeofenceExtract();

  static void getMapGeofence(zone::data::em_data::MapGeoFence& map_geofence,
                             const zone::common::EhrToEmData& ehr_to_emdata);
};
