

#include <string.h>
#include <time.h>
#include "hdm_utility/hdm_date_time.h"

namespace hdm_utility
{	

static uint64_t s_exe_start_tick = CHdmDateTime::GetTickCount();

CHdmDateTime::CHdmDateTime()
{
  
}

CHdmDateTime::~CHdmDateTime()
{

}

uint64_t CHdmDateTime::GetTickCount()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

uint32_t CHdmDateTime::GetExeRunTime()
{
	return (uint32_t)((GetTickCount() - s_exe_start_tick) / 1000000);
}


clock_t CHdmDateTime::GetCpuTickCount()
{
    return clock();
}

double  CHdmDateTime::GetCpuTime(clock_t& s)
{
    return (double)(clock() - s) * 1000000 / CLOCKS_PER_SEC;
}

double  CHdmDateTime::GetCpuTime(clock_t& e, clock_t& s)
{
    return (double)(e - s) * 1000000 / CLOCKS_PER_SEC;
}


std::string CHdmDateTime::GetFormatDateTimeString()
{
	char date_time[128] = {'\0'};
	GetFormatDateTimeString(date_time, sizeof(date_time));	
	return date_time;
}

void CHdmDateTime::GetFormatDateTimeString(char *data_time_buf, uint32_t data_time_buf_size, int64_t tick_count)
{
	if (nullptr == data_time_buf) {
		return;
	}
	time_t now;
	time(&now);
	struct tm *tm_now = localtime(&now);
	strftime(data_time_buf, data_time_buf_size, "%Y%m%d%H%M%S", tm_now);
	if (0 == tick_count) {
		tick_count = CHdmDateTime::GetTickCount();
	}
	if (tick_count >= 0) {
		snprintf(data_time_buf + strlen(data_time_buf), (data_time_buf_size - strlen(data_time_buf) - 1), ".%06ld", tick_count % 1000000);
	}
}

std::string CHdmDateTime::GetFormatTimeString()
{
	time_t now;
	time(&now);
	struct tm *tm_now = localtime(&now);

	char date_time[128] = {'\0'};
	strftime(date_time, 128, "%H%M%S", tm_now);
	return date_time;
}

}