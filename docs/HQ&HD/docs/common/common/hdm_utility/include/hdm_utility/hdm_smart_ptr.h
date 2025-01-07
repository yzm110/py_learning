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

#ifndef _CHDM_SMART_PTR_H_
#define _CHDM_SMART_PTR_H_

#include <string>

namespace hdm_utility
{	

template<typename T>
class CHdmSmartPtr
{
public:
CHdmSmartPtr(T *p=0)
:ptr(p)
,pUse(new int(1))
{

}

CHdmSmartPtr(const CHdmSmartPtr &src)
:ptr(src.ptr)
,pUse(src.pUse)
{
    ++*pUse;
}

CHdmSmartPtr &operator=(const CHdmSmartPtr&rhs)
{

    ++*rhs.pUse;
    decrUse();
    ptr=rhs.ptr;
    pUse=rhs.pUse;
    return *this;
}

~CHdmSmartPtr()
{
    decrUse();
}

T* operator->()
{
    if(ptr)
        return ptr;
}

const T* operator->()const
{
    if(ptr)
    return ptr;
}

T &operator*()
{
    if(ptr)
        return *ptr;
}

const T &operator*()const
{
    if(ptr)
        return *ptr;
}

private:
void decrUse()
{
    if(--*pUse==0){
        printf("CHdmSmartPtr delete object[%p]\n", ptr);
        delete ptr;
        delete pUse;
    }
}

T   *ptr;
int *pUse;
};

}

#endif  // _CHDM_SMART_PTR_H_
