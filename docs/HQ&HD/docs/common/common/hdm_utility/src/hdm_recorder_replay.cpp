#include <unistd.h>
#include <memory.h>
#include <stdio.h>
#include "hdm_recorder_replay.h"
#include "hdm_recorder_writer.h"
#include "hdm_utility/hdm_dbg_log.h"
#include "hdm_utility/hdm_date_time.h"
#include "hdm_utility/hdm_ini_config.h"
#include "hdm_utility/hdm_system.h"

namespace hdm_utility
{

/*---------------------------------------------------------------------------*/
// Functions

/*---------------------------------------------------------------------------*/
// Constructor
CHdmRecorderReplay::CHdmRecorderReplay()
: CHdmThread("hdm_rec_replay_thd")
{
}

/*---------------------------------------------------------------------------*/
// Destructor
CHdmRecorderReplay::~CHdmRecorderReplay()
{
}

/*---------------------------------------------------------------------------*/
// Initialization
void CHdmRecorderReplay::Initialize(const std::string &strFilePath)
{
    m_strFilePath = strFilePath;
}

/*---------------------------------------------------------------------------*/
// Start
void CHdmRecorderReplay::Start()
{
    // Start the thread, set the thread name
    is_stoped_ = false;
	BeginThread();
}

int32_t CHdmRecorderReplay::DoAction()
{
    // Create  pointer
    FILE *replay_file = fopen(m_strFilePath.c_str(), "r+");

    // Open the FILEPATH file as read-only
    if (nullptr == replay_file) {
        HDM_ERROR("RECORDER", "CHdmRecorderReplay::run open %s file faild\n", m_strFilePath.c_str());
        return 0;
    }
    // Get the log file size
    fseek(replay_file, 0L, SEEK_END);
	int64_t FileSize = ftell(replay_file);
    if (FileSize <= 12) {
        HDM_ERROR("RECORDER", "CHdmRecorderReplay::run file size error:%ld\n", FileSize);
        fclose(replay_file);
        return 0;
    }

    fseek(replay_file, 0L, SEEK_SET);

    int64_t LogFilterFlag = CHdmIniConfig::GetInstance()->GetConfigValueInt64("Record", "LogFilterFlag", 0);

    // Receive data buf
    uint32_t BufSize = 2048;
    char *pBuf = new char[BufSize]; // Receive data buf
    // Data sending interval and data length
    uint32_t ReadSize = 0;

    while (true) {

        // Check the condition of leaving, if the condition is met, the thread is launched
        if (IsStoped()) {
            break;
        }

        // When fileAllSize is less than or equal to 8 bytes, the AHP log has been read empty, no data will follow, the data will be reset from the beginning
        EHdmRecDataType eRecDataType = EHDM_RDT_UNKNOWN;
        int32_t               SleepTime = 0;
        uint32_t              RecSize = 0;
        uint32_t              ReadSuccess = false;

        ReadSize = fread(&eRecDataType, 1, sizeof(eRecDataType), replay_file);
        if (ReadSize == sizeof(eRecDataType)) {
            ReadSize = fread(&SleepTime, 1, sizeof(SleepTime), replay_file);
            if (0 > ReadSize) {
                break;
            }
            if ((ReadSize == sizeof(SleepTime)) && (1000000 > (SleepTime * 1000))) {
                usleep(SleepTime * 1000);
                ReadSize = fread(&RecSize, 1, sizeof(RecSize), replay_file);
                if (0 > ReadSize) {
                    break;
                }
                if (ReadSize == sizeof(RecSize)) {
                    if (RecSize + 1 > BufSize) {
                        delete[] pBuf;
                        pBuf = new char[RecSize + 1];
                        BufSize = RecSize + 1;
                    }
                    if (nullptr != pBuf) {
                        ReadSize = fread(pBuf, 1, RecSize, replay_file);
                        if (0 > ReadSize) {
                            break;
                        }
                        if (ReadSize == RecSize) {
                            pBuf[RecSize] = '\0';
                            ReadSuccess = true;
                            if (0 == LogFilterFlag || (LogFilterFlag & eRecDataType)) {
                                listeners_sync_mutex_.lock();
                                auto mapRecorderListeners = m_mapRecorderListeners;
                                listeners_sync_mutex_.unlock();
                                for (auto &Listener : mapRecorderListeners) {
                                    if (0 == (eRecDataType & Listener.second)) {
                                        continue;
                                    }
                                    if (nullptr != Listener.first) {
                                        Listener.first->OnRecordeMessage(eRecDataType, pBuf, RecSize);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (!ReadSuccess) {
            usleep(1000000);
            fseek(replay_file, 0L, SEEK_SET);
            listeners_sync_mutex_.lock();
            auto mapRecorderListeners = m_mapRecorderListeners;
            listeners_sync_mutex_.unlock();
            for (auto &Listener : mapRecorderListeners) {
                if (nullptr != Listener.first) {
                    Listener.first->OnReset();
                }
            }
        }

    }
    fclose(replay_file);
    delete[] pBuf;
    pBuf = nullptr;
    return 0;
}

/*---------------------------------------------------------------------------*/
// Stop
void CHdmRecorderReplay::Stop()
{
    // stop thread
    is_stoped_ = true;
	WaitAndCloseThread(1000);
}

void CHdmRecorderReplay::Deinitialize(void)
{

}

void CHdmRecorderReplay::RegisterListener(IHdmRecorderListener *pListener, int64_t LogFilterFlag)
{
    listeners_sync_mutex_.lock();
    m_mapRecorderListeners[pListener] = LogFilterFlag;
    listeners_sync_mutex_.unlock();
}

void CHdmRecorderReplay::UnregisterListener(IHdmRecorderListener *pListener)
{
    listeners_sync_mutex_.lock();
    m_mapRecorderListeners.erase(pListener);
    listeners_sync_mutex_.unlock();
}

}  // namespace hdm_utility
/* EOF */