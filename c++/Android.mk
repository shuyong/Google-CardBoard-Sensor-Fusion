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

# Example app that uses NDK sensors API.
include $(CLEAR_VARS)

LOCAL_MODULE := test_sensorfusion

LOCAL_C_INCLUDES:= sdk/ sdk/sensors/ sdk/util/

general_srcs:=$(wildcard sdk/*.cc)
sensors_srcs:=$(wildcard sdk/sensors/*.cc)
sensors_android_srcs:=$(wildcard sdk/sensors/android/*.cc)
util_srcs:=$(wildcard sdk/util/*.cc)
test_srcs:= $(wildcard *.cc)

LOCAL_SRC_FILES := $(general_srcs) $(sensors_srcs) $(sensors_android_srcs) $(util_srcs) $(test_srcs)
LOCAL_CFLAGS := -Wall -Wextra -fvisibility=hidden -D_FORTIFY_SOURCE=2 -DDEBUG
LOCAL_LDFLAGS := -Wl,-O2 -Wl,--as-needed -Wl,-Bsymbolic -landroid -llog
LOCAL_ARM_NEON := true

include $(BUILD_EXECUTABLE)
