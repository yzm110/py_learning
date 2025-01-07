
#include "hdm_recorder_manager.h"
#include <fstream>
#include "hdm_utility/hdm_ini_config.h"

namespace hdm_utility
{

/*---------------------------------------------------------------------------*/
// Functions

IHdmRecorderManagerIF* IHdmRecorderManagerIF::GetInstance()
{
    static CHdmRecorderManager m_RecorderManager;
    return &m_RecorderManager;
}

/*---------------------------------------------------------------------------*/
// Constructor
CHdmRecorderManager::CHdmRecorderManager()
: m_pRecorderWriter(nullptr)
, m_pRecorderReplay(nullptr)
{ 

}

/*---------------------------------------------------------------------------*/
// Destructor
CHdmRecorderManager::~CHdmRecorderManager()
{
    if (nullptr != m_pRecorderReplay) {
        delete m_pRecorderReplay;
        m_pRecorderReplay = nullptr;
    }
}

void CHdmRecorderManager::Initialize(void)
{
    int32_t RunType = CHdmIniConfig::GetInstance()->GetConfigValueInt("Record", "RunType", 0);
    if (1 == RunType) {
        // write file
        m_pRecorderWriter = new CHdmRecorderWriter();
        if (nullptr == m_pRecorderWriter) {
            return;
        }
        m_pRecorderWriter->Initialize();

    }
    else if (2 == RunType) {
        // read record
        std::string RecorderFilePath = CHdmIniConfig::GetInstance()->GetConfigValueString("DbgLog", "LogPath", "/app/system/log/map_log/");
        RecorderFilePath += "hdm_record.bin";
        m_pRecorderReplay = new CHdmRecorderReplay();
        if (nullptr == m_pRecorderReplay) {
            return;
        }
        m_pRecorderReplay->Initialize(RecorderFilePath);
    }
}


/*---------------------------------------------------------------------------*/
// Start
void CHdmRecorderManager::Start(void)
{
    if (nullptr != m_pRecorderWriter) {
        m_pRecorderWriter->Start();
    }
    if (nullptr != m_pRecorderReplay) {
        m_pRecorderReplay->Start();
    }
}

/*---------------------------------------------------------------------------*/
// Stop
void CHdmRecorderManager::Stop(void)
{
    if (nullptr != m_pRecorderWriter) {
        m_pRecorderWriter->Stop();
    }
    if (nullptr != m_pRecorderReplay) {
        m_pRecorderReplay->Stop();
    }
}

void CHdmRecorderManager::Deinitialize(void)
{
    if (nullptr != m_pRecorderWriter) {
        m_pRecorderWriter->Deinitialize();
    }
}

void CHdmRecorderManager::RegisterListener(IHdmRecorderListener *pListener, int64_t LogFilterFlag)
{
    if (nullptr != m_pRecorderReplay) {
        m_pRecorderReplay->RegisterListener(pListener, LogFilterFlag);
    }
}

void CHdmRecorderManager::UnregisterListener(IHdmRecorderListener *pListener)
{
    if (nullptr != m_pRecorderReplay) {
        m_pRecorderReplay->UnregisterListener(pListener);
    }
}

void CHdmRecorderManager::WriteRecord(EHdmRecDataType eRecDataType, const void *pRecData, uint32_t RecSize)
{
    int32_t RunType = CHdmIniConfig::GetInstance()->GetConfigValueInt("Record", "RunType", 0);
    if (1 != RunType) {
        return;
    }
    int64_t LogFilterFlag = CHdmIniConfig::GetInstance()->GetConfigValueInt64("Record", "LogFilterFlag", 0);
    if (0 == LogFilterFlag || (LogFilterFlag & eRecDataType)) {
        if (nullptr != m_pRecorderWriter) {
            m_pRecorderWriter->SendWriteTask(eRecDataType, pRecData, RecSize);
        }
    }
}

}

/* EOF */

