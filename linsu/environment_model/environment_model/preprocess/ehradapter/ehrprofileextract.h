#pragma once
#include "ehradapterhelper.h"

#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"

using ::zone::data::em_data::kHDMapSpdLmtSegIdx;

class RhrProfileExtract
{
 private:
  /* data */
 public:
  RhrProfileExtract(/* args */);
  ~RhrProfileExtract();
  static void getEhrLaneProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                zone::data::em_data::ReferenceLine& referline_profile,
                                const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneIsOnRouteProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                         zone::data::em_data::ReferenceLine& referline_profile,
                                         const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneIsInIntersectionProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                zone::data::em_data::ReferenceLine& referline_profile,
                                                const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneSpeedLimitProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                          zone::data::em_data::ReferenceLine& referline_profile,
                                          const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneTypeProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                        zone::data::em_data::ReferenceLine& referline_profile,
                                        const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneSlopeProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                         zone::data::em_data::ReferenceLine& referline_profile,
                                         const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneTransitionDirectionProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                       zone::data::em_data::ReferenceLine& referline_profile,
                                                       const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneSuperElevationProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                              zone::data::em_data::ReferenceLine& referline_profile,
                                              const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneCurvatureProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                             zone::data::em_data::ReferenceLine& referline_profile,
                                             const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneHeadingProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                           zone::data::em_data::ReferenceLine& referline_profile,
                                           const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneWidthProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                         zone::data::em_data::ReferenceLine& referline_profile,
                                         const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneLaneMarkingProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                           zone::data::em_data::ReferenceLine& referline_profile,
                                           const zone::common::EhrToEmData& ehr_to_emdata);

  static void getEhrLaneSpecialSituationProfile(const zone::common::EhrToEmCenterLine ehr2em_centerline,
                                                zone::data::em_data::ReferenceLine& referline_profile,
                                                const zone::common::EhrToEmData& ehr_to_emdata);
};
