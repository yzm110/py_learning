cmake_minimum_required(VERSION 3.5.1)
project(ehp_query VERSION 0.0.1)

# for correct library locations across platforms
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)
add_compile_options(${COMMON_COMPILE} -fPIC -fpermissive)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

if (NOT DEFINED LOCAL_3RD_PARTY)
  set(LOCAL_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_party)
endif()

if (NOT DEFINED ALGO_3RD_PARTY)
  set(ALGO_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party)
endif()

if (NOT DEFINED ALGO_COMMON_PATH)
  set(ALGO_COMMON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../common)
endif()

# if (NOT DEFINED LOCAL_COMMON_PATH)
#   set(LOCAL_COMMON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
# endif()

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
  message("build with ros:" ${BUILD_ROS})
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp )
  include_directories(${catkin_INCLUDE_DIRS})
  add_compile_options(${COMMON_COMPILE} -DPLATFORM_MARVEL_X_UBUNTU)
else()
  add_compile_options(${COMMON_COMPILE} -DPLATFORM_EP33_QNX)
endif()

include_directories(
  include
  ${LOCAL_3RD_PARTY}/navinfo/query/include
  ${LOCAL_3RD_PARTY}/navinfo/query/include/dataquery
)

set(SRC_LIST src/ehp_query.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})

if(${BUILD_ROS} AND ${BUILD_ROS} STREQUAL ON)
  # ROS BUILD
else()
  if(ZOP_PLATFORM STREQUAL "qnx")
    target_link_libraries(${PROJECT_NAME} PRIVATE libimpauto.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party/modules/hd_map/navinfo/query/lib/qnx/libimpauto.so
    )
  else()
    target_link_libraries(${PROJECT_NAME} PRIVATE libimpauto.so
      # ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_CURRENT_SOURCE_DIR}/../../../zop/src/3rd_party/modules/hd_map/navinfo/query/lib/x86/libimpauto.so
    )
  endif()
endif()

message("CMAKE_INSTALL_LIBDIR:" ${CMAKE_INSTALL_LIBDIR})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING 
  PATTERN "*.h" 
  PATTERN "*.hpp"
  )


# if(ZOP_PLATFORM STREQUAL "qnx" OR ZOP_PLATFORM STREQUAL "x86")
#   add_compile_options(${COMMON_COMPILE} -fPIC)
#   add_library(${PROJECT_NAME}_shared SHARED ${SRC_LIST})
#   set_target_properties(${PROJECT_NAME}_shared PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
# else()
#   add_compile_options(${COMMON_COMPILE} -fPIE)
# endif()