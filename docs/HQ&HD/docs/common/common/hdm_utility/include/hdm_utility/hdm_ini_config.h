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

/    Date: 2021-10-15
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_INI_CONFIG_H_
#define _CHDM_INI_CONFIG_H_

#include <string>
#include <map>
#include "common/data/basic_type.h"


namespace hdm_utility
{	

class CHdmIniConfig {
public:
    CHdmIniConfig();
    virtual ~CHdmIniConfig();

    static CHdmIniConfig* GetInstance();

    bool  Initialize(const std::string &file_path);
    std::string GetIniConfigPath();
    const std::string & GetConfigValueString(const char *section_name, const char *config_name, const std::string &default_value);
    int32_t   GetConfigValueInt(const char *section_name, const char *config_name, const int32_t default_value);   
    int64_t   GetConfigValueInt64(const char *section_name, const char *config_name, const int64_t default_value); 
    float32_t   GetConfigValueFloat(const char *section_name, const char *config_name, const float32_t default_value);
    void  Output();
private:
    bool ParseIniFile(const std::string &ini_file_path);
    bool ParseIniBuffer(const char *buffer, size_t size);
    bool Decode(char *data, size_t size);
    void RemoveSpace(char *&first, char *&end);
    std::string                                                file_path_;
    std::map<std::string, std::map<std::string, std::string> > ini_config_values_;
};

}

#endif  // _CHDM_INI_CONFIG_H_
