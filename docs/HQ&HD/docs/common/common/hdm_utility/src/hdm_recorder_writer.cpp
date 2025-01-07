#include <unistd.h>
#include <memory.h>
#include "hdm_recorder_writer.h"
#include "hdm_utility/hdm_dbg_log.h"
#include "hdm_utility/hdm_date_time.h"
#include "hdm_utility/hdm_ini_config.h"
#include "hdm_utility/hdm_system.h"

namespace hdm_utility
{

/**
 * @class CHdmWriteData
 *
 * @brief This class is used to write data
 *
 */
class CHdmWriteData
{
public:
    uint64_t WriteTime;
    EHdmRecDataType rec_data_type;
    void *pRecData;
    uint32_t RecSize;

    CHdmWriteData()
    {
        WriteTime = 0;
        rec_data_type = EHDM_RDT_UNKNOWN;
        pRecData = nullptr;
        RecSize = 0;
    }

    void ReleaseBuffer()
    {
        if (nullptr != pRecData) {
            delete (char*)pRecData;
            pRecData = nullptr;
        }
    }

    virtual ~CHdmWriteData()
    {
        ReleaseBuffer();
    }

private:
    // Disable the copy constructor
    CHdmWriteData(CHdmWriteData& rhs);

    // Disable the copy operator=
    CHdmWriteData& operator= (CHdmWriteData& rhs);
};

/*---------------------------------------------------------------------------*/
// Functions

/*---------------------------------------------------------------------------*/
// Constructor
CHdmRecorderWriter::CHdmRecorderWriter()
: CHdmThread("hdm_rec_write_thd")
, m_LastWriteTime(0)
, m_pRecorderFile(nullptr)
{
}

/*---------------------------------------------------------------------------*/
// Destructor
CHdmRecorderWriter::~CHdmRecorderWriter()
{
    if (nullptr != m_pRecorderFile) {
        delete m_pRecorderFile;
        m_pRecorderFile = nullptr;
    }
}

/*---------------------------------------------------------------------------*/
// Initialization
void CHdmRecorderWriter::Initialize()
{
#ifdef __QNX__
    std::string default_log_path = "/app/system/log/";
#else
    std::string default_log_path = "./map_log/";
#endif
    std::string root_path_ = CHdmIniConfig::GetInstance()->GetConfigValueString("DbgLog", "LogPath", default_log_path);
   
    if (0 != hdm_utility::CHdmSystem::MkDir(root_path_.c_str())) {
        printf("CHdmRecorderWriter::Initialize mkdir faile\n");
    }
    std::string start_date_time =  hdm_utility::CHdmDateTime::GetFormatDateTimeString();
   
    std::string RecorderFilePath = root_path_ + "hdm_record_" +  start_date_time + ".bin";  

    m_pRecorderFile = fopen(RecorderFilePath.c_str(), "wb+");
    if (nullptr == m_pRecorderFile) {
        HDM_ERROR("RECORDER", "CHdmRecorderManager::Initialize open file failed, RecorderFilePath[%s]\n", RecorderFilePath.c_str());
        return;
    }
}

/*---------------------------------------------------------------------------*/
// Start
void CHdmRecorderWriter::Start()
{
    is_stoped_ = false;
	BeginThread();
}

int32_t CHdmRecorderWriter::DoAction()
{
    while (true) {

        // Check the condition of leaving, if the condition is met, the thread is launched
        if (IsStoped()) {
            break;
        }
        CHdmWriteData *pWriteData = nullptr;
        data_sync_mutex_.lock();
        if (!write_datas_.empty()) {
            pWriteData = *(write_datas_.begin());
            write_datas_.pop_front();
        }
        data_sync_mutex_.unlock();
        if (nullptr != pWriteData) {
            if (nullptr != m_pRecorderFile) {
                fwrite(&pWriteData->rec_data_type, 1, sizeof(pWriteData->rec_data_type), m_pRecorderFile);

                int32_t SleepTime = 0;
                if (0 != m_LastWriteTime) {
                    SleepTime = pWriteData->WriteTime - m_LastWriteTime;
                }
                m_LastWriteTime = pWriteData->WriteTime;

                fwrite(&SleepTime, 1, sizeof(SleepTime), m_pRecorderFile);
                fwrite(&pWriteData->RecSize, 1, sizeof(pWriteData->RecSize), m_pRecorderFile);
                fwrite(pWriteData->pRecData, 1, pWriteData->RecSize, m_pRecorderFile);
                fflush(m_pRecorderFile);
            }
            else {
                usleep(200000);
            }
            delete pWriteData;
            pWriteData = nullptr;
        }
        else {
            usleep(200000);
        }
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
// Stop
void CHdmRecorderWriter::Stop()
{
    // stop thread
    is_stoped_ = true;
	WaitAndCloseThread(1000);
}

void CHdmRecorderWriter::Deinitialize(void)
{
    if (nullptr != m_pRecorderFile) {
        fflush(m_pRecorderFile);
        fclose(m_pRecorderFile);
        m_pRecorderFile = nullptr;
    }
}

void CHdmRecorderWriter::CleanAllTasks()
{
    data_sync_mutex_.lock();
    for (auto pWriteData : write_datas_) {
        delete pWriteData;
        pWriteData = nullptr;
    }
    data_sync_mutex_.unlock();
}

void CHdmRecorderWriter::SendWriteTask(EHdmRecDataType rec_data_type, const void *pRecData, uint32_t RecSize)
{
    if (nullptr == m_pRecorderFile) {
        HDM_ERROR("RECORDER", "CHdmRecorderManager::SendWriteTask file not open\n");
        return;
    }

    if (nullptr == pRecData || 0 == RecSize) {
        return;
    }

    CHdmWriteData *pWriteData = new CHdmWriteData();
    if (nullptr == pWriteData) {
        return;
    }
    pWriteData->WriteTime = CHdmDateTime::GetTickCount() / 1000;
    pWriteData->rec_data_type = rec_data_type;
    pWriteData->pRecData = new char[RecSize];
    if (nullptr == pWriteData->pRecData) {
        delete pWriteData;
        pWriteData = nullptr;
        return;
    }
    memcpy(pWriteData->pRecData, pRecData, RecSize);
    pWriteData->RecSize = RecSize;
    data_sync_mutex_.lock();
    write_datas_.push_back(pWriteData);
    data_sync_mutex_.unlock();  
}

}  // namespace hdm_utility
/* EOF */