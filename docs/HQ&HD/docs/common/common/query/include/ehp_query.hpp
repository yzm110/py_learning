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
#ifndef _CHDM_EHP_QUERY_HPP_
#define _CHDM_EHP_QUERY_HPP_

#include <string>
#include <memory>
#include "IMPAuto.hpp"
// #include "hdm_utility/hdm_dbg_log.h"


namespace ZONE {
namespace map_loc {
namespace query {
class EhpQuery
{
public:

  /**
   * @brief Init
   * 
   * @param notify_state_changed: calllback
   * @param ini_config_path 
   * @return true 
   * @return false 
   */
  virtual bool Init(const Navinfo::IMPAuto::SAIC::FUN_NotifyStateChanged_t &notify_state_changed,
                    const std::string &ini_config_path);

  /**
   * @brief Release
   * 
   */
  virtual void DeInit() {
    if (init_flag_) {
      // TODO(hcz):
      // NI::ZONE::Release();
      init_flag_ = false;
    }
  }

  virtual ~EhpQuery() {
    if (init_flag_) {
      // TODO(hcz):
      // NI::ZONE::Release();
      init_flag_ = false;
    }
  }

  static EhpQuery* GetInstance();

  static void Destory() {
    printf("EhpQuery::Destory");
    if (!instance_ptr_) {
      delete instance_ptr_;
      instance_ptr_ = nullptr; 
    }
  }

protected:
  EhpQuery(): init_flag_(false) {

  }
  static EhpQuery* instance_ptr_;
  
private:
  bool init_flag_;
};


} // query
} // map_loc
} // ZONE

#endif // _CHDM_EHP_QUERY_HPP_