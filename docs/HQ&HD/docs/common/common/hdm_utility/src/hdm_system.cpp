
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include "hdm_utility/hdm_system.h"

namespace hdm_utility
{	

static bool is_ehr_ready_ = false;

CHdmSystem::CHdmSystem()
{    
  
}

CHdmSystem::~CHdmSystem()
{

}


int32_t CHdmSystem::MkDir(const char *dir_path)
{
	if(0 != access(dir_path, F_OK))
	{
		return mkdir(dir_path, 0777);
	}
	return 0;
}

bool CHdmSystem::IsFileExist(const char *file_path)
{
	if (nullptr == file_path) {
		return false;
	}

	return 0 == access(file_path, 0);
}

uint64_t CHdmSystem::GetFileSize(const char *file_path)
{
	if (nullptr == file_path) {
		return 0;
	}

	struct stat	stStat;
	memset(&stStat, 0x0, sizeof(stStat));
	if (stat(file_path, &stStat) < 0) {	
		return 0;
	}
	return stStat.st_size;
}

void CHdmSystem::SetEhrReady(const bool is_ehr_ready)
{
	is_ehr_ready_ = is_ehr_ready;
}

bool CHdmSystem::GetEhrReady()
{
	return is_ehr_ready_;
}


}