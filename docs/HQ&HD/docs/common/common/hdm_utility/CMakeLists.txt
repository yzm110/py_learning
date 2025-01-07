cmake_minimum_required(VERSION 3.5.1)
project(hdm_utility VERSION 0.0.1)

# for correct library locations across platforms
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 14)
# set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
add_compile_options(-std=c++11)
add_compile_options(${COMMON_COMPILE} -fPIC)
add_definitions("-Wall" )

if(NOT DEFINED ZONE_ROS2)
  set(ZONE_ROS2 ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT DEFINED ZOP_PLATFORM)
  set(ZOP_PLATFORM x86)
endif()

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
    #add_definitions("-g")
    message("build with ros:" ${BUILD_ROS})
    find_package(catkin REQUIRED COMPONENTS roscpp geographic_transform)
    catkin_package(INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS roscpp geographic_transform)
  add_compile_options(-DPLATFORM_MARVEL_X_UBUNTU)
elseif(${ZONE_ROS2} STREQUAL ON)
  find_package(ament_cmake REQUIRED)
  find_package(geographic_transform REQUIRED)
  find_package(Geographic REQUIRED)
  add_compile_options(-DPLATFORM_EP33_QNX)
else()
  add_compile_options(-DPLATFORM_EP33_QNX)
endif()

if (NOT DEFINED LOCAL_3RD_PARTY)
  set(LOCAL_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_party)
endif()

if (NOT DEFINED ALGO_3RD_PARTY)
  set(ALGO_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party)
endif()

if (NOT DEFINED ALGO_COMMON_PATH)
  set(ALGO_COMMON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../../common)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ALGO_COMMON_PATH}/common/include
  ${ALGO_COMMON_PATH}/geographic_transform/include
  ${ALGO_3RD_PARTY}/include
  ${LOCAL_3RD_PARTY}/navinfo/queryquery/include
)

# # for .cmake find-scripts installs
# set(CONFIG_PACKAGE_LOCATION "share/${PROJECT_NAME}/cmake")
# # so installed solver libraries link to base/SNOPT
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# # so installed solver libraries link to libbase_core.so
# set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# # replicate default for catkin installs
# # set(base_INSTALL_BINDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

set(SRC_LIST
  src/hdm_coords_converter.cpp
  src/hdm_date_time.cpp
  src/hdm_dbg_log.cpp
  src/hdm_ehr_output.cpp
  src/hdm_geometry.cpp
  src/hdm_ini_config.cpp
  src/hdm_mutex.cpp
  src/hdm_recorder_manager.cpp
  src/hdm_recorder_replay.cpp
  src/hdm_recorder_writer.cpp
  src/hdm_system.cpp
  src/hdm_thread.cpp
)



# if(ZOP_PLATFORM STREQUAL "ep33_qnx" OR ZOP_PLATFORM STREQUAL "x86")
# add_compile_options(${COMMON_COMPILE} -fPIC)
# add_library(${PROJECT_NAME}_shared SHARED ${SRC_LIST})
# set_target_properties(${PROJECT_NAME}_shared PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
# else()
#   add_compile_options(-std=c++11 -fPIE)
#   add_compile_options(${COMMON_COMPILE} -fPIE)
# endif()

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
  message("build with ros:" ${BUILD_ROS})
  find_package(catkin REQUIRED COMPONENTS roscpp common geographic_transform)
  catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp )
  include_directories(${catkin_INCLUDE_DIRS})
  add_compile_options(${COMMON_COMPILE} -DPLATFORM_MARVEL_X_UBUNTU)
elseif(${ZONE_ROS2} STREQUAL ON)
  find_package(ament_cmake REQUIRED)
  find_package(geographic_transform REQUIRED)
else()
  add_compile_options(${COMMON_COMPILE} -DPLATFORM_EP33_QNX)
endif()

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
  # ROS BUILD
elseif(${ZONE_ROS2} STREQUAL ON)
  ament_target_dependencies(${PROJECT_NAME} geographic_transform)
  #target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../../../../common/GeographicLib-1.49/lib/x86/libGeographic.so)
  add_definitions(-DLEGACY_LOG)
else()
  if(ZOP_PLATFORM STREQUAL "qnx")
    target_link_libraries(${PROJECT_NAME} PRIVATE libgeographic_transform.so libGeographic.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/../../../../external/3rd_party/lib/qnx/libGeographic.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party/modules/hd_map/navinfo/query/lib/qnx/libimpauto.so
    )
  else()
    target_link_libraries(${PROJECT_NAME} PRIVATE libgeographic_transform.so libGeographic.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/../../../../external/3rd_party/lib/x86/libGeographic.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party/modules/hd_map/navinfo/query/lib/x86/libimpauto.so
    )
  endif()
  find_library(LIB_sovp_asf_log NAMES sovp_asf_log)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_sovp_asf_log})

  find_library(LIB_sovp-xlog NAMES sovp-xlog)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_sovp-xlog})
endif()

if(${ZONE_ROS2} STREQUAL OFF)
  set(CMAKE_INSTALL_INCLUDEDIR include)
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING 
    PATTERN "*.h" 
    PATTERN "*.hpp"
    )
else()
  install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
   )
   install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

if(${ZONE_ROS2} STREQUAL ON)
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(geographic_transform)
  ament_package()
endif()

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
  if(ZOP_PLATFORM STREQUAL "x86")
    include(test.cmake)
  endif()
endif()
