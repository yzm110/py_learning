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


#ifndef _IHDM_RECORDER_MANAGER_IF_H_
#define _IHDM_RECORDER_MANAGER_IF_H_


/*---------------------------------------------------------------------------*/
// Include files
#include <memory> // std::weak_ptr std::shared_ptr
#include <string>
#include <utility> // std::move
#include "hdm_utility/hdm_recorder_listener.h"


/*---------------------------------------------------------------------------*/
// Namesapce
namespace hdm_utility
{

/**
 * @class IHdmRecorderManagerIF
 *
 * @brief This class is used to receive socket data
 *
 */
class IHdmRecorderManagerIF
{
public:
    static IHdmRecorderManagerIF* GetInstance(void);

    /*
     * Destroy the instance
     *
     * @param   none
     *
     * @return none
     */
    static void Destroy(void);

    /*
     * Initialization
     *
     * @param   [out]   listener：Set the listener, when the data is received, send the data through the callback
     *
     * @return  true : initialized successfully
     *
     * @retval  <std::unique_ptr<IHdmRecorderManagerIF>>
     */
    virtual void Initialize(void) = 0;
    /*
     * Start
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Start(void) = 0;

    /*
     * Stop
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Stop(void) = 0;

    virtual void Deinitialize(void) = 0;

    virtual void RegisterListener(IHdmRecorderListener *pListener, int64_t LogFilterFlag) = 0;

    virtual void UnregisterListener(IHdmRecorderListener *pListener) = 0;


    virtual void WriteRecord(EHdmRecDataType eRecDataType, const void *pRecData, uint32_t RecSize) = 0;

    /*
     * Destructor
     */
    virtual ~IHdmRecorderManagerIF() {}
};

}  // namespace hdm_utility
#endif /* _IHDM_RECORDER_MANAGER_IF_H_ */
/* EOF */