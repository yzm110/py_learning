/******************************************************************************
*                              Copyright
*------------------------------------------------------------------------------
* Copyright © 2020 SAIC MOTOR Z-ONE SOFTWARE COMPANY.  All rights reserved.
*
* This software is furnished under a license and may be used and copied
* only in accordance with the terms of such license and with the inclusion
* of the above copyright notice. This software or any other copies thereof
* may not be provided or otherwise made available to any other person.
* No title to and ownership of the software is hereby transferred.
*
* The information in this software is subject to change without notice
* and should not be constructed as a commitment by
* SAIC MOTOR Z-ONE SOFTWARE COMPANY.
*
* SAIC MOTOR Z-ONE SOFTWARE COMPANY assumes no responsibility for the use
* or reliability of its Software on equipment which is not supported by
* SAIC MOTOR Z-ONE SOFTWARE COMPANY.
*
*******************************************************************************
* File name     : Log.h
* Description   : log module
* Version       : v1.0
* Create Time   : 2021/10/07
* Author        : Wang ChenAn
* Modify history:
*******************************************************************************
* Modify Time   Modify person  Modification
* ------------------------------------------------------------------------------
*
*******************************************************************************/
/**
* @file  Log.h
* @brief  log模块
* @author      王晨安
* @date        2021-10-9
* @version     V1.0
**********************************************************************************
* @attention
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/10/9  <td>1.0      <td>王晨安  <td>创建初始版本
* </table>
*
**********************************************************************************
*/
 
#ifndef  _MAPLOC_LOG_MODULE_
#define  _MAPLOC_LOG_MODULE_
 
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdint.h>
#include "nullstream.h"
 
static nullstream dummyStream;
///
/// \brief 初始化日志文件
/// \param log_filename 信息文件的名字
void initLogger(const std::string& log_filename, int& st);
 
///
/// \brief 日志系统类
///
class Logger {
   friend void initLogger(const std::string& log_filename, int& st);
   
public:
         //构造函数
   Logger(log_rank_t log_rank) : m_log_rank(log_rank) {};
   
   ~Logger();   
   ///
   /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名
   /// \param log_rank 日志的等级
   /// \param line 日志发生的行号
   /// \param function 日志发生的函数
   static std::ostream& start(log_rank_t log_rank,
                               const int32_t line,
                               const std::string& function);
   
private:
   ///
   /// \brief 根据等级获取相应的日志输出流
   ///
   static std::ostream& getStream(log_rank_t log_rank);
   
   static std::ofstream m_log_file;                   ///< 信息日子的输出流
   static int status;
   log_rank_t m_log_rank;                             ///< 日志的信息的等级
};
 
 
///
/// \brief 根据不同等级进行用不同的输出流进行读写
///
#define LOG(log_rank)   \
((log_rank == NOPRINT)?dummyStream : Logger(log_rank).start(log_rank, __LINE__,__FUNCTION__))
 
///
/// \brief 利用日记进行检查的各种宏
///
#define CHECK(a)                                            \
   if(!(a)) {                                              \
       LOG(ERROR) << " CHECK failed " << endl              \
                   << #a << "= " << (a) << endl;          \
       abort();                                            \
   }                                                      \
 
#define CHECK_NOTNULL(a)                                    \
   if( NULL == (a)) {                                      \
       LOG(ERROR) << " CHECK_NOTNULL failed "              \
                   << #a << "== NULL " << endl;           \
       abort();                                            \
    }
 
#define CHECK_NULL(a)                                       \
   if( NULL != (a)) {                                      \
       LOG(ERROR) << " CHECK_NULL failed " << endl         \
                   << #a << "!= NULL " << endl;           \
       abort();                                            \
    }
 
 
#define CHECK_EQ(a, b)                                      \
   if(!((a) == (b))) {                                     \
       LOG(ERROR) << " CHECK_EQ failed "  << endl          \
                   << #a << "= " << (a) << endl           \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_NE(a, b)                                      \
   if(!((a) != (b))) {                                     \
       LOG(ERROR) << " CHECK_NE failed " << endl           \
                   << #a << "= " << (a) << endl           \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_LT(a, b)                                      \
   if(!((a) < (b))) {                                      \
       LOG(ERROR) << " CHECK_LT failed "                   \
                   << #a << "= " << (a) << endl           \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_GT(a, b)                                      \
   if(!((a) > (b))) {                                      \
       LOG(ERROR) << " CHECK_GT failed "  << endl          \
                  << #a <<" = " << (a) << endl            \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_LE(a, b)                                      \
   if(!((a) <= (b))) {                                     \
       LOG(ERROR) << " CHECK_LE failed "  << endl          \
                   << #a << "= " << (a) << endl           \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_GE(a, b)                                      \
   if(!((a) >= (b))) {                                     \
       LOG(ERROR) << " CHECK_GE failed "  << endl          \
                   << #a << " = "<< (a) << endl            \
                   << #b << "= " << (b) << endl;          \
       abort();                                            \
    }
 
#define CHECK_DOUBLE_EQ(a, b)                               \
   do {                                                    \
       CHECK_LE((a), (b)+0.000000000000001L);              \
       CHECK_GE((a), (b)-0.000000000000001L);              \
    }while (0)
 
#endif