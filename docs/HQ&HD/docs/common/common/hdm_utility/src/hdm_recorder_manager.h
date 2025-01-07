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

/    Date: 2022-8-4
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/


#ifndef _CHDM_RECORDER_MANAGER_H_
#define _CHDM_RECORDER_MANAGER_H_

#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation (use a .cpp suffix)
#endif

/*---------------------------------------------------------------------------*/
// Include files
#include <string>
#include <map>
#include "hdm_recorder_writer.h"
#include "hdm_recorder_replay.h"
#include "hdm_utility/hdm_recorder_manager_if.h"

/*---------------------------------------------------------------------------*/
// Namesapce
namespace hdm_utility
{

/*---------------------------------------------------------------------------*/
// Value define

/**
 * @class CHdmRecorderManager
 *
 * @brief This class is used to receive socket data
 *
 */
class CHdmRecorderManager : public IHdmRecorderManagerIF
{
public:
    CHdmRecorderManager();
    /*
     * Destructor
     */
    virtual ~CHdmRecorderManager();

    /*
     * Initialization
     *
     * @param   [out]   listener：Set the listener, when the data is received, send the data through the callback
     *
     * @return  true : initialized successfully
     *
     * @retval
     */
    virtual void Initialize(void);

    /*
     * Start
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Start(void);

    /*
     * Stop
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Stop(void);

    virtual void Deinitialize(void);

    virtual void RegisterListener(IHdmRecorderListener *pListener, int64_t LogFilterFlag);

    virtual void UnregisterListener(IHdmRecorderListener *pListener);

    virtual void WriteRecord(EHdmRecDataType eRecDataType, const void *pRecData, uint32_t RecSize);

private:

    // Disable copy constructor and operator=
    CHdmRecorderManager(CHdmRecorderManager&);
    CHdmRecorderManager& operator=(CHdmRecorderManager&);

    CHdmRecorderWriter *m_pRecorderWriter;
    CHdmRecorderReplay *m_pRecorderReplay;
};

}  // namespace hdm_utility
#endif /* CHDMDATADEALWITH_H */
/* EOF */