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

#ifndef _CHDM_DBG_LOG_H_
#define _CHDM_DBG_LOG_H_

#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <set>
#include <map>
#include <mutex>


namespace hdm_utility
{	

enum HdmLogLevel 
{
    HDM_LOG_UNKNOWN = 0,  
    HDM_LOG_DEBUG,
    HDM_LOG_INFO,
    HDM_LOG_WARN,
    HDM_LOG_ERROR,
    HDM_LOG_KEYERROR,
    HDM_LOG_FATAL
};

struct HdmLogFile
{
    FILE*       file_handle;
    // std::mutex  file_mutex;

    HdmLogFile() 
    : file_handle(nullptr)
    // , file_mutex() 
    {
    }
};

class IHdmLogNotify
{
public:
    /**@brief Destructor    
    */
	virtual ~IHdmLogNotify() {}
    
	/**@brief notify hdm log    
     */
	virtual void OnHdmLog(int32_t log_id, int32_t level, int64_t time, int32_t pid, int32_t tid, const char *application, const char *tag, const char *text) {}   
};

class CHdmDbgLog {
public:
    CHdmDbgLog();
    virtual ~CHdmDbgLog();

    static CHdmDbgLog* GetInstance();

    void RegisteNotify(IHdmLogNotify* hdm_log_notify);
    void UnregisteNotify(IHdmLogNotify* hdm_log_notify);

    const std::string &GetComLogName();

    const std::string &GetErrorLogName();

    void Write(const char *file_path, const bool flush, const char *out_text, ...);
    void Close(const char *file_path);
  
    void LogOut(HdmLogLevel log_level, const char *tag, const char *file_path, const char *func, int32_t line_number, const char *out_text, ...);

    // static void DebugLogPrint (int32_t a_Prio, const char *a_Tag, const char *a_OutText, ...);
    
    void WriteFile(const char *file_path, const char* data_buf, int32_t data_size);

    bool IsErrorDataFileOutput();

private:
    void LogOut(HdmLogLevel log_level, const char *tag, const char *file_path, const char *func, int32_t line_number, const char *out_text, va_list hdm_va_list);
    void WriteAsfLog(HdmLogLevel log_level, const char *out_text);
    
    int32_t                       log_id_;
    std::string                   exe_name_;
    std::string                   start_date_time_;
    std::string                   com_log_name_;
    std::string                   error_log_name_;
    std::set<IHdmLogNotify*>      hdm_log_notifys_;
    std::mutex                    log_map_mutex_;
    std::map<std::string, HdmLogFile*>  log_files_;

    std::string                       log_path_;
    int32_t                           console_log_level_;
    int32_t                           file_log_level_;
    int32_t                           console_log_output_;
    int32_t                           file_log_output_;
    int32_t                           error_data_file_output_;
    int32_t                           error_output_time_;

};

}

#define HDM_WRITE(path, flush, format, ...) \
    do { \
        if (hdm_utility::CHdmDbgLog::GetInstance()->IsErrorDataFileOutput()) { \
            hdm_utility::CHdmDbgLog::GetInstance()->Write(path, flush, format, ##__VA_ARGS__); \
        } \
    } while (0)

#define HDM_CLOSE(path) \
    do { \
        if (hdm_utility::CHdmDbgLog::GetInstance()->IsErrorDataFileOutput()) { \
            hdm_utility::CHdmDbgLog::GetInstance()->Close(path); \
        } \
    } while (0)

#define HDM_DEBUG(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_DEBUG, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#define HDM_INFO(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_INFO, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#define HDM_WARNING(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_WARN, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#define HDM_ERROR(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_ERROR, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#define HDM_KEYERROR(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_KEYERROR, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#define HDM_FATAL(tag, format, ...) \
    do { \
        hdm_utility::CHdmDbgLog::GetInstance()->LogOut(hdm_utility::HDM_LOG_FATAL, tag, __FILE__, __func__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)

#endif  // _CHDM_DBG_LOG_H_
