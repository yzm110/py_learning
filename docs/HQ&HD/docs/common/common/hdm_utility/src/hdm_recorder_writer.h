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

#ifndef _CHDM_RECORDER_WRITER_H_
#define _CHDM_RECORDER_WRITER_H_

/*---------------------------------------------------------------------------*/
// Include files
#include <memory> // std::weak_ptr std::shared_ptr
#include <string>
#include <list>
#include <mutex>
#include "hdm_utility/hdm_recorder_listener.h"
#include "hdm_utility/hdm_thread.h"


/*---------------------------------------------------------------------------*/
// Namesapce
namespace hdm_utility
{

class CHdmWriteData;

/**
 * @class CHdmRecorderWriter
 *
 * @brief This class is used to receive socket data
 *
 */
class CHdmRecorderWriter : public CHdmThread
{
public:

    /*
     * Constructor
     */
    CHdmRecorderWriter();

    /*
     * Destructor
     */
    virtual ~CHdmRecorderWriter();

    /*
     * Initialization
     *
     * @param   [out]   listener：Set the listener, when the data is received, send the data through the callback
     *
     * @return  true : initialized successfully
     *
     * @retval  <std::unique_ptr<CHdmRecorderWriter>>
     */
    virtual void Initialize();

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
     * Stop
     *
     * @param   none
     *
     * @return  none
     *
     */
    virtual void Stop();

    virtual void Deinitialize(void);



    void SendWriteTask(EHdmRecDataType rec_data_type, const void *pRecData, uint32_t RecSize);

protected:
    virtual int32_t DoAction();
    
private:
    // Disable the copy constructor
    CHdmRecorderWriter(CHdmRecorderWriter& rhs);

    // Disable the copy operator=
    CHdmRecorderWriter& operator= (CHdmRecorderWriter& rhs);

private:
    void CleanAllTasks();

    // Members
    std::mutex                 data_sync_mutex_;
    std::list<CHdmWriteData*>  write_datas_;

    uint64_t                   m_LastWriteTime;
    FILE                      *m_pRecorderFile;
};

}  // namespace hdm_utility
#endif /* CHDMDATAREPLAY_H */
/* EOF */
