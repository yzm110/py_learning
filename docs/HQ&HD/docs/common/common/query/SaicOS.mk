LOCAL_PATH := $(call my-dir)
my_makefile := $(lastword $(MAKEFILE_LIST))

include $(CLEAR_VARS)

LOCAL_MODULE := libehp_query
LOCAL_MODULE_CLASS := CMAKE
LOCAL_CPPFLAGS := -Wno-return-type -std=c++14 -ffast-math -use_fast_math -pthread -fexceptions
LOCAL_BUILT_SHARED_LIBRARIES := \
     libehp_query.so

LOCAL_NOTICE_FILE := LICENSE

LOCAL_BINARY_RELEASE_MK := $(my_makefile)

# LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/include \
#         $(LOCAL_PATH)/include/math 
LOCAL_EXPORT_CMAKE_CONFIG_DIRS := $$<PREFIX>

LOCAL_3RD_PARTY := ${LOCAL_PATH}/../../3rd_party
ALGO_3RD_PARTY := ${LOCAL_PATH}/../../../zop/src/3rd_party
ALGO_COMMON_PATH := $(LOCAL_PATH)/../../../common
# LOCAL_COMMON_PATH := $(LOCAL_PATH)/../../common

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/include \
    ${LOCAL_3RD_PARTY}/navinfo/query/include \
    ${LOCAL_3RD_PARTY}/navinfo/query/include/dataquery 


ifeq ($(TARGET_DEVICE),  x86_64)
    # LOCAL_SHARED_LIBRARIES := libcommon  # x86 call static lib for gsl
    LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=x86 -DUNIT_TEST=OFF
else ifeq ($(TARGET_DEVICE), xavier-qnx)
    # LOCAL_SHARED_LIBRARIES := libcommon libgsl libgslcblas
    LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=qnx -DUNIT_TEST=OFF
endif

LOCAL_SHARED_LIBRARIES := libimpauto

include $(BUILD_EXTERNAL_PROJECT)
