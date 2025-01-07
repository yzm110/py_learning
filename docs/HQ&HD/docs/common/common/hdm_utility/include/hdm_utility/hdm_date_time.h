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

/    Date: 2021-8-17
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_DATE_TIME_H_
#define _CHDM_DATE_TIME_H_

#include <string>
#include <ctime>

namespace hdm_utility
{	

class CHdmDateTime {
public:
    CHdmDateTime();
    ~CHdmDateTime();
    
    static uint64_t GetTickCount();
    static uint32_t GetExeRunTime();
    static clock_t  GetCpuTickCount();
    static double   GetCpuTime(clock_t& s);
    static double   GetCpuTime(clock_t& s, clock_t& e);
    static std::string GetFormatDateTimeString();
    static void        GetFormatDateTimeString(char *data_time_buf, uint32_t data_time_buf_size, int64_t tick_count = -1);
    static std::string GetFormatTimeString();
    

};
}

#endif  // _CHDM_DATE_TIME_H_
