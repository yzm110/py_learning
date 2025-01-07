#pragma once
#include "common/data/ehr_to_em_data.h"
#include "common/data/em_data.h"
using zone::data::em_data::kMaxMapGuidePtNum;

class EhrMapGuidePointExtract
{
 private:
  /* data */
 public:
  EhrMapGuidePointExtract(/* args */);
  ~EhrMapGuidePointExtract();

  static void GuidePointInfo(bc::TCArray<zone::data::em_data::MapGuidePoint, kMaxMapGuidePtNum>& em_map_guide_pts,
                             const zone::common::EhrToEmData& ehr_to_emdata);
};
