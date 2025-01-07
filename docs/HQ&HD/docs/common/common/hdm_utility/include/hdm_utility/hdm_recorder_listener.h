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

#ifndef _IHDM_RECORDER_LISTENER_H
#define _IHDM_RECORDER_LISTENER_H

/*---------------------------------------------------------------------------*/
// Include files
#include <string>


/*---------------------------------------------------------------------------*/
// Namesapce
namespace hdm_utility
{

/*---------------------------------------------------------------------------*/
// Class define

enum EHdmRecDataType
{
    EHDM_RDT_UNKNOWN = 0,
    EHDM_RDT_CAR_POS_INFO = 1,
    EHDM_RDT_ROUTE_LIST = 2,
    EHDM_RDT_EHP_INPUT_INFO = 4,
    EHDM_RDT_NET_DATA = 8,
    EHDM_RDT_ADASISV3_MSG = 16,
};

/**
 * @class IHdmRecorderListener
 *
 * @brief class description
 */
class IHdmRecorderListener
{
public:

    /*
     * Destructor
     */
    virtual ~IHdmRecorderListener() {}

    /**
     * @brief   send data
     *
     * @param   [out] MsgData
     *
     * @return  none
     */
    virtual void OnRecordeMessage(EHdmRecDataType eRecorderType, const char *pBuf, uint32_t BufSize) = 0;

    /**
     * @brief   Reset data
     *
     * @param   none
     *
     * @return  none
     */
    virtual void OnReset() = 0;
};

/*---------------------------------------------------------------------------*/
// Namesapce
}

/*---------------------------------------------------------------------------*/
#endif  // _IHDM_RECORDER_LISTENER_H
/* EOF */
