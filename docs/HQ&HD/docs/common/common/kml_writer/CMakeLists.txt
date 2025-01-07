cmake_minimum_required(VERSION 3.5.1)
project(kml_writer)

include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)
add_compile_options(${COMMON_COMPILE} -fPIC -fpermissive)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

include_directories(
  include
  # ${WS_DIR}/${ALGO_OUTPUT_INCLUDE}/
)

set(SRC_LIST
  src/kml_tool.cpp
  src/text_kml_writer.cpp
)

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


add_library(${PROJECT_NAME} SHARED ${SRC_LIST})

message("CMAKE_INSTALL_LIBDIR:" ${CMAKE_INSTALL_LIBDIR})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING 
  PATTERN "*.h" 
  PATTERN "*.hpp"
  )
