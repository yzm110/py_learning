#include "log/include/Log.h"
#include <cstdlib>
#include <ctime>
#include <time.h>
#include <thread>

std::ofstream Logger::m_log_file;
int Logger::status;
 
void initLogger(const std::string& log_filename, int& st){
   Logger::status = st;
   if (Logger::status == 0)
   {
	   return ;
   }
   Logger::m_log_file.open(log_filename.c_str());
}
 
std::ostream& Logger::getStream(log_rank_t log_rank){
   if (Logger::status == 0)
   {
	   return dummyStream;
   }	
   return  (m_log_file.is_open() ?m_log_file : std::cout);
}
 
std::ostream& Logger::start(log_rank_t log_rank,
                            const int32_t line,
                            const std::string&function) {
	if (Logger::status == 0)
	{
		return dummyStream;
	}	
	time_t tNowTime;
	time(&tNowTime);
 
	tm* tLocalTime = localtime(&tNowTime);
 	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	// int64_t us_count = ts.tv_nsec / 1000;
	
	//"2011-07-18 23:03:01.000000 ";  
	std::string strFormat = "%Y-%m-%d %H:%M:%S ";
 
	char strDateTime[30] = { '\0' };
	strftime(strDateTime, 30, strFormat.c_str(), tLocalTime);
	
	const int ndx_decimal_sec = 19;
	sprintf(&strDateTime[ndx_decimal_sec], ".%06ld \0", ts.tv_nsec/1000);

	std::string logRank = "";

	switch (log_rank)
	{
	case INFO:
		logRank = " [INFO] ";
		break;
	case WARNING:
		logRank = " [WARNING] ";
		break;	
	case ERROR:
		logRank = " [ERROR] ";
		break;
	default:
		logRank = " [INFO] ";
		break;
	}

   return getStream(log_rank) << strDateTime
							   << " PID: "
   							   << std::this_thread::get_id()
							   << logRank
                               << function << "("<< ")"
                               << "  LINE: " << line << "  "
                               <<std::flush;
}
 
Logger::~Logger(){
   getStream(m_log_rank) << std::endl << std::flush;
   
   if (FATAL == m_log_rank) {
       m_log_file.close();
       m_log_file.close();
       m_log_file.close();
       abort();
    }
}