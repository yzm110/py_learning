
#include "hdm_utility/hdm_thread.h"
#include "hdm_utility/hdm_dbg_log.h"

namespace hdm_utility
{	

CHdmThread::CHdmThread(const char *thread_name)
: thread_id_(0)
, is_stoped_(true)
{
	SetThreadName(thread_name);
}

CHdmThread::~CHdmThread()
{
	WaitAndCloseThread(HDM_INFINITE);
}

void CHdmThread::SetThreadName(const char *thread_name)
{
	if (nullptr != thread_name) {
		thread_name_ = thread_name;
	}
	else {
		thread_name_ = "hdm_unknown_thread";
	}
}

const std::string& CHdmThread::GetThreadName() const
{
	return thread_name_;
}

bool CHdmThread::BeginThread()
{
	is_stoped_ = false;
	int32_t return_code = pthread_create(&thread_id_, nullptr, ThreadProc, static_cast<CHdmThread*>(this));	
	if (0 != return_code) {		
		HDM_ERROR("CHdmThread","CHdmThread::Create failed return_code[%d]\n", return_code);
		return false;
	}
	
	if (0 != pthread_setname_np(thread_id_, thread_name_.c_str())) {	
		HDM_ERROR("CHdmThread","CHdmThread::Create pthread_setname_np failed thread_name_[%s]\n", thread_name_.c_str());
		return false;
	}
	HDM_INFO("CHdmThread","CHdmThread::BeginThread thread_name_[%s]\n", thread_name_.c_str());
	return true;
}

bool CHdmThread::IsStoped()
{
	return is_stoped_;
}

bool CHdmThread::WaitAndCloseThread(const uint64_t milli_seconds)
{
	is_stoped_ = true;
	if (0 != thread_id_) {
		pthread_join(thread_id_, nullptr);
		thread_id_ = 0;
		return true;	
	}
	return false;
}

void* CHdmThread::ThreadProc(void *param)
{
	HDM_INFO("CHdmThread","CHdmThread::ThreadProc start\n");
	CHdmThread	*hdm_thread = static_cast<CHdmThread*>(param);
	if (nullptr != hdm_thread) {
	    hdm_thread->PreProcess();
		HDM_INFO("CHdmThread","CHdmThread::ThreadProc IsStoped[%d], thread_name_[%s]\n", hdm_thread->IsStoped(), hdm_thread->thread_name_.c_str());
	    while (!hdm_thread->IsStoped()) {
		    hdm_thread->DoAction();
	    }
	}	
	return param;
}

void CHdmThread::PreProcess()
{

}

int32_t CHdmThread::DoAction()
{
	return 0;
}

}