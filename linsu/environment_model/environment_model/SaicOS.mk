###############################################################################
#                              Copyright
#------------------------------------------------------------------------------
#    Copyright © 2023 SAIC MOTOR Z-ONE SOFTWARE COMPANY.  All rights reserved.
#
#    This software is furnished under a license and may be used and copied
#    only in accordance with the terms of such license and with the inclusion
#    of the above copyright notice. This software or any other copies thereof
#    may not be provided or otherwise made available to any other person.
#    No title to and ownership of the software is hereby transferred.
#
#    The information in this software is subject to change without notice
#    and should not be constructed as a commitment by
#    SAIC MOTOR Z-ONE SOFTWARE COMPANY.
#
#    SAIC MOTOR Z-ONE SOFTWARE COMPANY assumes no responsibility for the use
#    or reliability of its Software on equipment which is not supported by
#    SAIC MOTOR Z-ONE SOFTWARE COMPANY.
#------------------------------------------------------------------------------
###############################################################################
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libenvironment_model
LOCAL_MODULE_CLASS := CMAKE
LOCAL_CPPFLAGS :=
LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/plm \
    $(LOCAL_PATH)/preprocess \
    $(LOCAL_PATH)/common
LOCAL_SHARED_LIBRARIES := \
    libcommon \
    libsovp_asf_log \
    libsovp-xlog \
    libmath \
    libgeographic_transform \
    lib_diag_client

LOCAL_EXPORT_SHARED_LIBRARIES := \
    lib_diag_client

LOCAL_INCREMENTAL_FLAGS := true

LOCAL_CONFIG_FLAGS := -DZONE_ROS2=OFF
ifeq ($(TARGET_DEVICE), lxc)
    LOCAL_CONFIG_FLAGS += -DCMAKE_BUILD_TYPE=Debug 
    LOCAL_CPPFLAGS += -UNDEBUG -DDEBUG -DSTD_COUT_ENABLE
endif

ifeq ($(TARGET_DEVICE),  x86_64)
    LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=x86 -DUNIT_TEST=OFF
else ifeq ($(TARGET_DEVICE), xavier-qnx)
    LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=qnx -DUNIT_TEST=OFF
endif

ifneq ($(findstring es37_ipd, $(TARGET_PRODUCT)), )
    LOCAL_CFLAGS += -DPRODUCT_ES33
endif
ifneq ($(findstring zpd_sj5, $(TARGET_PRODUCT)), )
    LOCAL_CFLAGS += -DPRODUCT_ES33
endif
ifneq ($(findstring zpd_dj5, $(TARGET_PRODUCT)), )
    LOCAL_CFLAGS += -DPRODUCT_ES37
endif

LOCAL_PREFIX                    := /usr/app
LOCAL_BUILT_SHARED_LIBRARIES    := libenvironment_model.so
LOCAL_EXPORT_C_INCLUDE_DIRS     := $(LOCAL_PATH)/interface
#LOCAL_EXPORT_CMAKE_CONFIG_DIRS := $$<PREFIX>

include $(BUILD_EXTERNAL_PROJECT)
