
#include <string.h>
#include <unistd.h>
#ifdef __QNX__
//#ifdef _QNX_SOURCE
#else
#include <sys/syscall.h>
#endif
#include "hdm_utility/hdm_dbg_log.h"
#include "hdm_utility/hdm_date_time.h"
#include "hdm_utility/hdm_system.h"
#include "hdm_utility/hdm_ini_config.h"
#ifndef LEGACY_LOG
#include "asf/log/log.h"
#endif//LEGACY_LOG
namespace hdm_utility
{	

static pid_t gettid()
{
#ifdef __QNX__
//#ifdef _QNX_SOURCE
    return 0;
#else
    return syscall(SYS_gettid);
#endif
}

std::string get_exe_name()
{
#ifdef __QNX__
//#ifdef _QNX_SOURCE
    return "map_filter";
#else
    char exe_path[512] = {'\0'};
    if (readlink("/proc/self/exe", exe_path, sizeof(exe_path)) <= 0) {
        return "";
    }

    char* path_end = strrchr(exe_path,  '/');
    if (nullptr == path_end)   {
        return "";
    }

    return ++path_end;
#endif
}

CHdmDbgLog* CHdmDbgLog::GetInstance()
{
    static CHdmDbgLog hdm_dbg_log;
    return &hdm_dbg_log;
}

CHdmDbgLog::CHdmDbgLog()
: log_id_(0)
, hdm_log_notifys_()
, log_files_()
{
#ifdef __QNX__
    std::string default_log_path = "/app/system/log/";
#else
    std::string default_log_path = "./map_log/";
#endif
    log_path_ = CHdmIniConfig::GetInstance()->GetConfigValueString("DbgLog", "LogPath", default_log_path);
    console_log_level_= CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "ConsoleLogLevel", HDM_LOG_DEBUG);
    file_log_level_   = CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "FileLogLevel", HDM_LOG_ERROR);
    console_log_output_ = CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "ConsoleLogOutput", 1);
    file_log_output_    = CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "FileLogOutput", 1);
    error_data_file_output_      = CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "ErrorDataFileOutput", 1);
    error_output_time_ = CHdmIniConfig::GetInstance()->GetConfigValueInt("DbgLog", "ErrorOutputTime", 1);
    printf("CHdmDbgLog::CHdmDbgLog console_log_level_[%d], file_log_level_[%d], console_log_output_[%d], file_log_output_[%d], error_data_file_output_[%d], error_output_time_[%d]\n", 
    console_log_level_, file_log_level_, console_log_output_, file_log_output_, error_data_file_output_, error_output_time_);

    // start_date_time_ =  hdm_utility::CHdmDateTime::GetFormatDateTimeString();
    start_date_time_ = "";
    exe_name_ = get_exe_name();
    com_log_name_ = start_date_time_ + exe_name_ + ".log";
    error_log_name_ = start_date_time_ + exe_name_ + "_error.log";
    if (0 != hdm_utility::CHdmSystem::MkDir(log_path_.c_str())) {
        printf("CHdmDbgLog::CHdmDbgLog create dir faile\n");
    }
}

CHdmDbgLog::~CHdmDbgLog()
{
    log_map_mutex_.lock();
    for (auto it_log_file = log_files_.begin(); it_log_file != log_files_.end(); ++it_log_file) {
        fflush(it_log_file->second->file_handle);
        fclose(it_log_file->second->file_handle);
        delete it_log_file->second;
        it_log_file->second = nullptr;
    }
    log_files_.clear();
    log_map_mutex_.unlock();
}

void CHdmDbgLog::RegisteNotify(IHdmLogNotify* hdm_log_notify)
{
    hdm_log_notifys_.insert(hdm_log_notify);
}

void CHdmDbgLog::UnregisteNotify(IHdmLogNotify* hdm_log_notify)
{
    hdm_log_notifys_.erase(hdm_log_notify);
}

const std::string &CHdmDbgLog::GetComLogName()
{
    return com_log_name_;
}
    
const std::string &CHdmDbgLog::GetErrorLogName()
{
    return error_log_name_;
}

void CHdmDbgLog::LogOut(HdmLogLevel log_level, const char *tag, const char *file_path, const char *func, int32_t line_number, const char *out_text, ...)
{
    if ((log_level >= console_log_level_ || log_level >= file_log_level_) && (console_log_output_ || file_log_output_))   {
        va_list	 hdm_arg_list;
        va_start(hdm_arg_list, out_text);       
        LogOut(log_level, tag,  file_path, func, line_number, out_text, hdm_arg_list);
        va_end(hdm_arg_list);
    }    
}

void CHdmDbgLog::Write(const char *file_path, const bool flush, const char *out_text, ...)
{
    if (nullptr != file_path && nullptr != out_text)  {
        std::map<std::string, HdmLogFile*>::iterator it_log_file;
        log_map_mutex_.lock();
        it_log_file = log_files_.find(file_path);
        if (it_log_file == log_files_.end()) {
            std::string str_file_path = file_path;
            if (str_file_path.npos == str_file_path.find("/")) {
                str_file_path = log_path_ + str_file_path;
            }
            HdmLogFile *log_file = new HdmLogFile();
            log_file->file_handle = fopen(str_file_path.c_str(), "w");
            if (nullptr == log_file->file_handle)    {
                printf("CHdmDbgLog::Write open file[%s] failed, cur_open_file_num[%lu]\n", str_file_path.c_str(), log_files_.size());
                delete log_file;
                log_map_mutex_.unlock();
                return;
            }
            else {
                printf("CHdmDbgLog::Write open file success, file_path[%s], cur_open_file_num[%lu]\n", str_file_path.c_str(), log_files_.size());
            }
            log_files_[file_path] = log_file;
            it_log_file = log_files_.find(file_path);
        }
        log_map_mutex_.unlock();

        va_list	  arglist;
        va_start(arglist, out_text);
        // it_log_file->second->file_mutex.lock();
        vfprintf(it_log_file->second->file_handle, out_text, arglist);
        va_end(arglist);
        if (flush) {
            fflush(it_log_file->second->file_handle);
        }
        // it_log_file->second->file_mutex.unlock();            
    }
}

void CHdmDbgLog::Close(const char *file_path)
{
    if (nullptr != file_path)  {
        log_map_mutex_.lock();
        auto it_log_file = log_files_.find(file_path);
        if (it_log_file != log_files_.end()) {
            // it_log_file->second->file_mutex.lock();
            fflush(it_log_file->second->file_handle);
            fclose(it_log_file->second->file_handle);
            // it_log_file->second->file_mutex.unlock();
            delete it_log_file->second;
            it_log_file->second = nullptr;
            log_files_.erase(it_log_file);
        } 
        log_map_mutex_.unlock();      
    }
}


void CHdmDbgLog::WriteFile(const char *file_path, const char* data_buf, int32_t data_size)
{
    FILE *file = fopen(file_path, "a+");
    if (nullptr != file)   {				
        fwrite(data_buf, 1, data_size, file);			
        fclose(file);
        file = nullptr;	
    }
}

void CHdmDbgLog::WriteAsfLog(HdmLogLevel log_level, const char *out_text)
{
#ifndef LEGACY_LOG
    switch (log_level)
    {
    case HDM_LOG_DEBUG:
        zone::sovp::asf::log::LogDebug("[hdmap]") << out_text;
        break;
    case HDM_LOG_INFO:
        zone::sovp::asf::log::LogInfo("[hdmap]") << out_text;
        break;
    case HDM_LOG_WARN:
        zone::sovp::asf::log::LogWarn("[hdmap]") << out_text;
        break;
    case HDM_LOG_ERROR:
    case HDM_LOG_KEYERROR:
        zone::sovp::asf::log::LogError("[hdmap]") << out_text;
        break;
    case HDM_LOG_FATAL:
        zone::sovp::asf::log::LogFatal("[hdmap]") << out_text;
        break;
    default:
        zone::sovp::asf::log::LogVerbose("[hdmap]") << out_text;
        break;
    }
#endif //LEGACY_LOG
}

void CHdmDbgLog::LogOut(HdmLogLevel log_level, const char *tag, const char *file_path, const char *func, int32_t line_number, const char *out_text, va_list hdm_va_list)
{
    char prior_str[20] = {'\0'};
    switch (log_level)
    {   
    case HDM_LOG_DEBUG:
        strncpy(prior_str, "D", sizeof(prior_str));
        break;
    case HDM_LOG_INFO:
        strncpy(prior_str, "I", sizeof(prior_str));
        break;
    case HDM_LOG_WARN:
        strncpy(prior_str, "W", sizeof(prior_str));
        break;
    case HDM_LOG_ERROR:
    case HDM_LOG_KEYERROR:
        strncpy(prior_str, "E", sizeof(prior_str));
        break;
    case HDM_LOG_FATAL:
        strncpy(prior_str, "F", sizeof(prior_str));
        break;    
    default:
        strncpy(prior_str, "UNKNOWN", sizeof(prior_str));
        break;
    }

#ifdef LEGACY_LOG
    char text_info[2048] = "[";
#else 
    char text_info[2048] = "";
#endif//LEGACY_LOG

    if (nullptr != out_text) {
#ifdef LEGACY_LOG
        CHdmDateTime::GetFormatDateTimeString(text_info + 1, sizeof(text_info) - 1, 0);
        snprintf(text_info + strlen(text_info), (sizeof(text_info) - strlen(text_info) - 1), "][%d#%d][%s:%s]:", getpid(), gettid(), tag, prior_str);
#else
        snprintf(text_info + strlen(text_info), (sizeof(text_info) - strlen(text_info) - 1), "[%s:%s]:", tag, prior_str);
#endif//LEGACY_LOG
        char *text_start = text_info + strlen(text_info);
        vsnprintf(text_start, (sizeof(text_info) - strlen(text_info) - 1), out_text, hdm_va_list);
        if (console_log_output_ && log_level >= console_log_level_) {
            if (!hdm_log_notifys_.empty()) {
                for (auto &hdm_log_notify : hdm_log_notifys_) {
                    hdm_log_notify->OnHdmLog(++log_id_, log_level, CHdmDateTime::GetTickCount() / 1000, getpid(), gettid(), exe_name_.c_str(), tag, text_start);
                }            
            }
            printf("%s", text_info);
        }
        if (file_log_output_) {
            if (log_level >= file_log_level_) {
#ifdef LEGACY_LOG
                CHdmDbgLog::GetInstance()->Write(CHdmDbgLog::GetInstance()->GetComLogName().c_str(), true, "%s", text_info);
                if (log_level >= HDM_LOG_ERROR) {
                    if (error_output_time_) {
                        text_start = text_info;
                    }
                    CHdmDbgLog::GetInstance()->Write(CHdmDbgLog::GetInstance()->GetErrorLogName().c_str(), true, "%s", text_start);
                }
#else
                WriteAsfLog(log_level, text_info);
#endif//LEGACY_LOG
            }
        }
    }    
}

bool CHdmDbgLog::IsErrorDataFileOutput()
{
    return 0 == error_data_file_output_ ? false : true;
}

}
/* EOF */