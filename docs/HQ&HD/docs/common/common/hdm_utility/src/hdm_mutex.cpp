
#include "hdm_utility/hdm_mutex.h"


namespace hdm_utility
{	

bool CHdmMutex::is_use_mutex_ = false;

CHdmMutex::CHdmMutex()
{

}

CHdmMutex::~CHdmMutex()
{

}

void CHdmMutex::Lock()
{
    if (is_use_mutex_) {
	     mutex_.lock();
	}
}

void CHdmMutex::Unlock()
{   
	if (is_use_mutex_) {
	    mutex_.unlock();
	}
}

}