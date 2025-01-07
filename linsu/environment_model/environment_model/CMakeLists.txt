cmake_minimum_required(VERSION 3.5.1)
project(environment_model VERSION 0.0.1)

include(../../common/build/generate_version_file.cmake)
generate_version_file(${CMAKE_BINARY_DIR})
add_compile_options(-DPLATFORM_EP33_QNX -Werror)

set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED ZONE_ROS2)
  set(ZONE_ROS2 ON)
endif()

if(${ZONE_ROS2} STREQUAL OFF)
else()
  find_package(ament_cmake REQUIRED)
  find_package(common)
  find_package(time_coherent_cache)
  find_package(math)
  find_package(geographic_transform)
  find_package(diag_client)
endif()

include_directories(
  .
  include
  interface
  internal
  )

add_library(
  ${PROJECT_NAME} SHARED
  preprocess/preprocess.cpp
  hlmf/hlmf.cpp
  olr/olr.cpp
  static_obj_process/sop.cpp
  urban_scenario_recognition/isr.cpp
  environment_model.cpp
  em_interface.cpp
  weight_distribution/weight_distribution.cpp
  # ehr data adapter
  preprocess/ehradapter/ehrdatahandle.cpp
  preprocess/ehradapter/ehradapterhelper.cpp
  preprocess/ehradapter/ehrprofileextract.cpp
  preprocess/ehradapter/ehrmapguidepointextract.cpp
  preprocess/ehradapter/ehrmapgeofenceextract.cpp
  preprocess/ehradapter/ehr2emenum.cpp
  internal/convex_hull.cpp
  )

target_compile_options(${PROJECT_NAME}  PRIVATE -ffast-math -use_fast_math -pthread -fexceptions)

if(${ZONE_ROS2} STREQUAL OFF)
  target_link_libraries(${PROJECT_NAME} PRIVATE libcommon.so libmath.so libgeographic_transform.so lib_diag_client.so)

  find_library(LIB_sovp_asf_timer NAMES sovp_asf_timer)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_sovp_asf_timer})
else()
  ament_target_dependencies(
    ${PROJECT_NAME} 
    common 
    time_coherent_cache
    math
    geographic_transform
    lib_diag_client
    )
endif()

find_library(LIB_sovp_asf_log NAMES sovp_asf_log)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_sovp_asf_log})

find_library(LIB_sovp-xlog NAMES sovp-xlog)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_sovp-xlog})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

if(NOT DEFINED CMAKE_INSTALL_INCLUDEDIR)
  set(CMAKE_INSTALL_INCLUDEDIR include)
endif()
install(DIRECTORY interface/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(${ZONE_ROS2} STREQUAL OFF)
else()
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies()
  ament_package()
endif()

execute_process(
  COMMAND bash "-c" "git rev-parse --short HEAD"
  OUTPUT_VARIABLE GIT_COMMIT
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
string (STRIP ${GIT_COMMIT} GIT_COMMIT )
execute_process(
  COMMAND bash "-c"  "git log -1 --format=%cd"
  OUTPUT_VARIABLE GIT_DATE
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
string (STRIP ${GIT_DATE} GIT_DATE )

add_definitions(-DGIT_COMMIT="${GIT_COMMIT}")
add_definitions(-DGIT_DATE="${GIT_DATE}")


# include_directories(
#   include
#   interface
#   plm
#   common
#   preprocess
#   )
# 
# add_library(
#   ${PROJECT_NAME} SHARED
#   preprocess/preprocess.cpp
#   hlmf/hlmf_filter.cpp
#   hlmf/hlmf.cpp
#   olr/olr.cpp
#   environment_model.cpp
#   em_interface.cpp
#   plm/plm.cpp
#   plm/plm_conf.cpp
#   plm/otl/otllane/otllane.cpp
#   plm/otl/otlpoint/otlpoint.cpp
#   plm/otl/otlfilter/otlfilter.cpp
#   plm/otl/otltrajectoryreader/otltrajectoryreader.cpp
#   plm/otl/otl.cpp
#   plm/stl/stl.cpp
#   plm/etl/etl.cpp
#   plm/common/utils/utilities.cpp
#   plm/common/portInterfaces/riroadgeometryandlanesrx.cpp
#   lmf/lmf_course.cpp
#   lmf/lmf_helper.cpp
#   lmf/lmf_lane.cpp
#   lmf/lmf.cpp
#   lmf/lmf_lane_border.cpp
#   weight_distribution/weight_distribution.cpp
#   # ehr data adapter
#   preprocess/ehradapter/ehrdatahandle.cpp
#   preprocess/ehradapter/ehradapterhelper.cpp
#   preprocess/ehradapter/ehrprofileextract.cpp
#   preprocess/ehradapter/ehrmapguidepointextract.cpp
#   preprocess/ehradapter/ehrmapgeofenceextract.cpp
#   preprocess/ehradapter/ehr2emenum.cpp
#   tv/tracked_veh.cpp
#   swarm/swarm.cpp
#   )
# 
# ament_target_dependencies(
#     ${PROJECT_NAME} 
#     common 
#     time_coherent_cache
#     math
#     geographic_transform
#   )
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   )
# 
# install(DIRECTORY interface/
#   DESTINATION include/
#   )
# 
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies( 
#   common 
#   time_coherent_cache
#   math
#   geographic_transform
# )
# ament_package()


#  if(UNIT_TEST STREQUAL "ON")
#    target_include_directories(
#      ${PROJECT_NAME}
#      PUBLIC
#      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#      $<INSTALL_INTERFACE:include>
#      )
#  
#    target_link_libraries(
#      ${PROJECT_NAME}
#      ${WS_DIR}/src/3rd_party/lib/${ZOP_PLATFORM}/libgsl.a
#      ${WS_DIR}/src/3rd_party/lib/${ZOP_PLATFORM}/libgslcblas.a
#      ${WS_DIR}/src/3rd_party/lib/${ZOP_PLATFORM}/libglog.a
#      ${LIB_gmock_main}
#      ${LIB_gmock}
#      ${LIB_gtest}
#      )
#  
#    # INSTALL
#    # Copy header files to usr/local/include/environment_modelling_algo/*
#    install(
#      DIRECTORY include/${PROJECT_NAME}
#      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#      )
#  
#    # generate the file baseConfigVersion.cmake
#    include(CMakePackageConfigHelpers)
#    write_basic_package_version_file(
#      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
#      VERSION ${PROJECT_VERSION}
#      COMPATIBILITY SameMajorVersion
#      )
#  
#    # install the two files in a place where cmake looks for them
#    install(
#      FILES
#      "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-config.cmake"         # self-written
#      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake" # generated
#      DESTINATION
#      ${CONFIG_PACKAGE_LOCATION}
#      )
#  
#    # Install a 'package.xml' file. This is required by REP-136.
#    install(FILES package.xml DESTINATION share/${PROJECT_NAME})
#  endif()
