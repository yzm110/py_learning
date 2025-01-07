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

/    Date: 2021-11-17
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_SYSTEM_H_
#define _CHDM_SYSTEM_H_

#include <string>

namespace hdm_utility
{	

class CHdmSystem {
public:
    CHdmSystem();
    ~CHdmSystem();
    
    static int32_t  MkDir(const char *dir_path);
    static bool IsFileExist(const char *file_path);
    static uint64_t GetFileSize(const char *file_path);
    
    static void SetEhrReady(const bool is_ehr_ready);
    static bool GetEhrReady();
};
}

#endif  // _CHDM_SYSTEM_H_
