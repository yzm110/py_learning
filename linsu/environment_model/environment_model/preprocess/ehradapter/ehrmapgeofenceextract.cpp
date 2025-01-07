#include "ehrmapgeofenceextract.h"

EhrMapGeofenceExtract::EhrMapGeofenceExtract(/* args */) {}

EhrMapGeofenceExtract::~EhrMapGeofenceExtract() {}
void EhrMapGeofenceExtract::getMapGeofence(zone::data::em_data::MapGeoFence& map_geofence,
                                           const zone::common::EhrToEmData& ehr_to_emdata)
{
  const zone::common::EhrToEmGeofenceData& ehr2emGeofance = ehr_to_emdata.geofence_data_;

  map_geofence.is_in_map_ = ehr2emGeofance.is_in_map_;
  map_geofence.is_on_intersection_ = ehr2emGeofance.is_on_intersection_;
  map_geofence.is_on_ramp_ = ehr2emGeofance.is_on_ramp_;
}