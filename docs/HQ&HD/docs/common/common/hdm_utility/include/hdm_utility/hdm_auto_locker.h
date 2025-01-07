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

/    Date: 2021-12-14
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_AUTO_LOCKER_H_
#define _CHDM_AUTO_LOCKER_H_

#include "hdm_mutex.h"

namespace hdm_utility
{	

template< class T >
class CHdmAutoLocker
{
public:
	CHdmAutoLocker(T &t) : t_(t), is_locked_(false)
	{
		Lock();
	}

	~CHdmAutoLocker()
	{
		Unlock();
	}

private:
    void  Lock() 
	{
		if (!is_locked_) {
			t_.Lock();
			is_locked_ = true;
		}
    }

   void  Unlock() 
   {
	   if (is_locked_) {
			t_.Unlock();
			is_locked_ = false;
		}
   }
   T						&t_;
   bool					     is_locked_;
	
};

}

#endif	// _CHDM_AUTO_LOCKER_H_
