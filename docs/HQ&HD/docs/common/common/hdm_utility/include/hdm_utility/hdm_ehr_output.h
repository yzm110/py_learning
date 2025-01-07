/******************************************************************************
/                              Copyright
/------------------------------------------------------------------------------
/    Copyright © 2020 SAIC MOTOR Z-ONE SOFTWARE COMPANY.  All rights reserved.
/
/    This software is furnished under a license and may be used and copied
/    only in accordance with the terms of such license and with the inclusion
/    of the above copyright notice. This software or any other copies thereof
/    may not be provided or otherwise made available to any other person.
/    No title to and ownership of the software is hereby transferred.
/
/    The information in this software is subject to change without notice
/    and should not be constructed as a commitment by
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY.
/
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY assumes no responsibility for the use
/    or reliability of its Software on equipment which is not supported by
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY.

/    Date: 2021-11-2
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_EHR_OUTPUT_H_
#define _CHDM_EHR_OUTPUT_H_

#include <memory>
#include "common/data/basic_type.h"
#include "common/data/localization_data.h"

namespace zone
{	
namespace common
{	
    class EHRCarPosition;
    class EHRSectionList;
    class EhrToEmData;
    class EHRSection;
}
}

namespace hdm_utility
{	

class CHdmEhrOutput {
public:
    CHdmEhrOutput();
    virtual ~CHdmEhrOutput();

    static bool OutputCarPosition(const char_t *file_path, const std::shared_ptr<zone::common::EHRCarPosition> &car_position);

	static bool OutputSectionList(const char_t *file_path, const std::shared_ptr<zone::common::EHRSectionList> &section_list);

    static bool OutputToEmData(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);

    static bool OutputGuidePoint(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);

    static bool OutputEhpInputInfo(const char_t *file_path, const std::shared_ptr<zone::common::EhpInputInfo> &ehp_input_info);

    static bool OutputSection(const char_t *file_path, const zone::common::EHRSection & section); 

private:
    static void OutputPtsOffset(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputMapPts(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputIsOnRout(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputIsInIntersection(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputSpeedLimit(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneType(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneSlop(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputSuperElevation(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneCurvature(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneHeading(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneWidth(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneTransitionDirection(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneMarking(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputSpecialSituation(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneBoundarySegment(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputRoadEdgeSegment(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneElements(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputGuidePoint2(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputTrafficLight(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
    static void OutputLaneData(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data);
};

#ifdef __QNX__

#define HDM_OUTPUT_CAR_POSITION(path, data)
#define HDM_OUTPUT_SECTION_LIST(path, data)
#define HDM_OUTPUT_TO_EM_DATA(path, data)
#define HDM_OUTPUT_GUIDE_POINT(path, data)
#define HDM_OUTPUT_SECTION(path, data) 

#else

#define HDM_OUTPUT_CAR_POSITION(path, data) \
    do { \
        hdm_utility::CHdmEhrOutput::OutputCarPosition(path, data); \
    } while (0)

#define HDM_OUTPUT_SECTION_LIST(path, data) \
    do { \
        hdm_utility::CHdmEhrOutput::OutputSectionList(path, data); \
    } while (0)

#define HDM_OUTPUT_TO_EM_DATA(path, data) \
    do { \
        hdm_utility::CHdmEhrOutput::OutputToEmData(path, data); \
    } while (0)

#define HDM_OUTPUT_GUIDE_POINT(path, data) \
    do { \
        hdm_utility::CHdmEhrOutput::OutputGuidePoint(path, data); \
    } while (0)

#define HDM_OUTPUT_SECTION(path, data) \
    do { \
        hdm_utility::CHdmEhrOutput::OutputSection(path, data); \
    } while (0)

#endif

}

#endif  // _CHDM_EHR_OUTPUT_H_
