#
# Copyright (C) 2015 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_PATH := $(call my-dir)

# Example app that uses sensors HAL.
include $(CLEAR_VARS)
LOCAL_MODULE := hal-example-app
#LOCAL_C_INCLUDES:=
LOCAL_SRC_FILES := hal-example-app.cpp hardware.c
LOCAL_CFLAGS := -Wall -Wextra -fvisibility=hidden -D_FORTIFY_SOURCE=2 -DDEBUG -I./android/hardware/libhardware/include -I./android/system/core/include -I./android/system/core/libsync/include 
LOCAL_LDFLAGS := -Wl,-O2 -Wl,--as-needed -Wl,-Bsymbolic
LOCAL_ARM_NEON := true
#LOCAL_SHARED_LIBRARIES := hardware
include $(BUILD_EXECUTABLE)
