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

#ifndef CHDM_RECORDER_REPLAY_H
#define CHDM_RECORDER_REPLAY_H

/*---------------------------------------------------------------------------*/
// Include files
#include <memory> // std::weak_ptr std::shared_ptr
#include <string>
#include <map>
#include <mutex>
#include "hdm_utility/hdm_recorder_listener.h"
#include "hdm_utility/hdm_thread.h"


/*---------------------------------------------------------------------------*/
// Namesapce
namespace hdm_utility
{

/**
 * @class CHdmRecorderReplay
 *
 * @brief This class is used to receive socket data
 *
 */

class CHdmRecorderReplay : public CHdmThread
{
public:

    /*
     * Constructor
     */
    CHdmRecorderReplay();

    /*
     * Destructor
     */
    virtual ~CHdmRecorderReplay();

    /*
     * Initialization
     *
     * @param   [out]   listener：Set the listener, when the data is received, send the data through the callback
     *
     * @return  true : initialized successfully
     *
     * @retval  <std::unique_ptr<CHdmRecorderReplay>>
     */
    virtual void Initialize(const std::string &strFilePath);

    /*
     * Start
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Start();

    /*
     * Run
     *
     * @param   none
     *
     * @return  none
     *
     */
    void run();

    /*
     * Stop
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Stop();

    virtual void Deinitialize(void);

    void RegisterListener(IHdmRecorderListener *pListener, int64_t LogFilterFlag);

    void UnregisterListener(IHdmRecorderListener *pListener);

protected:
    virtual int32_t DoAction();

private:
    // Disable the copy constructor
    CHdmRecorderReplay(CHdmRecorderReplay& rhs);

    // Disable the copy operator=
    CHdmRecorderReplay& operator= (CHdmRecorderReplay& rhs);

private:
    // Members
    std::string     m_strFilePath;
    std::mutex                               listeners_sync_mutex_;
    std::map<IHdmRecorderListener*, int32_t> m_mapRecorderListeners;
};

}  // namespace hdm_utility
#endif /* CHDMDATAREPLAY_H */
/* EOF */
