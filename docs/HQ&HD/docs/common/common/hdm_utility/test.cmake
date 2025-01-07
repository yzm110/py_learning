

set(ALGO_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd_party)

find_library(LIB_gmock NAMES gmock PATHS PATHS ${ALGO_3RD_PARTY}/lib/x86 NO_DEFAULT_PATH)
find_library(LIB_gtest NAMES gtest PATHS PATHS ${ALGO_3RD_PARTY}/lib/x86 NO_DEFAULT_PATH)
find_library(LIB_gmock_main NAMES gmock_main PATHS ${ALGO_3RD_PARTY}/lib/x86 NO_DEFAULT_PATH)
find_library(LIB_gtest_main NAMES gtest_main PATHS ${ALGO_3RD_PARTY}/lib/x86 NO_DEFAULT_PATH)

include_directories( 
  ${WS_DIR}/third_party/include
)

add_executable(${PROJECT_NAME}_test
  test/main_test.cpp
  test/hdm_geometry_test.cpp
  test/hdm_smart_ptr_test.cpp
)


target_link_libraries(
  ${PROJECT_NAME}_test
  ${LIB_gmock}
  ${LIB_gtest}
  ${LIB_gmock_main}
  hdm_utility
  ${CMAKE_BINARY_DIR}/../../devel/lib/libgeographic_transform.so
  ${CMAKE_BINARY_DIR}/../../src/3rd_party/lib/x86/Geographic/libGeographic.a
  pthread
)

target_include_directories(
    ${PROJECT_NAME}_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

target_compile_options(
  ${PROJECT_NAME}_test
  PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  )





