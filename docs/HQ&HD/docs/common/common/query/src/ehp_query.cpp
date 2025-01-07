#include "ehp_query.hpp"
#include <iostream>

namespace ZONE {
namespace map_loc {
namespace query {

EhpQuery* EhpQuery::instance_ptr_ = nullptr;

 EhpQuery* EhpQuery::GetInstance() {
    printf("EhpQuery::GetInstance start instance_ptr_[%p]\n", instance_ptr_);
    if (nullptr == instance_ptr_) {
        instance_ptr_ = new EhpQuery();
    }
    printf("EhpQuery::GetInstance end instance_ptr_[%p]\n", instance_ptr_);
    return instance_ptr_;
}

bool EhpQuery::Init(const Navinfo::IMPAuto::SAIC::FUN_NotifyStateChanged_t &notify_state_changed,
                    const std::string &ini_config_path)
{
    printf("EhpQuery::Init start init_flag_[%d]\n", init_flag_);
    std::cout << "EhpQuery::Init ini_config_path: " << ini_config_path << std::endl;
    if (!init_flag_) {
      NI::ZONE::CCreateParam create_param;
      create_param.m_pFunNotifyStateChanged = notify_state_changed;
      Navinfo::IMPAuto::Common::RETURN_CODE ret_code = NI::ZONE::Create(create_param);
      if (Navinfo::IMPAuto::SAIC::RC_SUCCESS != ret_code) {
        printf("EhpQuery::Init CreateParam faild ret_code.m_iCode[%d]!\n", ret_code.m_iCode);
        return false;
      }

      NI::ZONE::CInitParam init_param;
      init_param.m_strConfigPath     = ini_config_path;
      init_param.m_strVehicleFactory = "1";
      init_param.m_strVehicleModel   = "1";
      init_param.m_strVehicleId      = "1";
      ret_code = NI::ZONE::Init(init_param);
      if (Navinfo::IMPAuto::SAIC::RC_SUCCESS != ret_code)
      {
        printf("EhpQuery::Init InitParam failed ret_code.m_iCode[%d]!\n", ret_code.m_iCode);
        return false;
      }
      printf("EhpQuery::Init Successed.\n");
      init_flag_ = true;
    }
    printf("EhpQuery::Init end init_flag_[%d]\n", init_flag_);
    return true;
}

} // query
} // map_loc
} // ZONE