
#ifndef  _MAPLOC_LOG_NUUL_STREAM_MODULE_
#define  _MAPLOC_LOG_NUUL_STREAM_MODULE_
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdint.h>

///
/// \brief 日志文件的类型
///
typedef enum log_rank {
   NOPRINT,
   INFO,
   WARNING,
   ERROR,
   FATAL
}log_rank_t;

class nullstream: public std::ostream{
    public:
        nullstream() : std::ostream(nullptr) {}
};

#endif