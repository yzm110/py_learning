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

#ifndef _CHDM_THREAD_H_
#define _CHDM_THREAD_H_

#include <pthread.h>
#include <string>
namespace hdm_utility
{	

#define HDM_INFINITE (0xFFFFFFFFFFFFFFFF)

class CHdmThread
{
public:
	CHdmThread(const char *thread_name = "hdm_unknown_thread");
	virtual ~CHdmThread();

	void SetThreadName(const char *thread_name);
	const std::string& GetThreadName() const;
    
	bool  BeginThread();
	bool  IsStoped();
	bool  WaitAndCloseThread(const uint64_t milli_seconds = HDM_INFINITE);


protected:
	static  void* ThreadProc(void *param);
	virtual void PreProcess();
	virtual int32_t DoAction();

public:
	std::string			thread_name_;
	pthread_t           thread_id_;
	bool                is_stoped_;
};

}

#endif	// _CHDM_THREAD_H_
