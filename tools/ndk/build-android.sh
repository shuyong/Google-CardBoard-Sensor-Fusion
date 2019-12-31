#!/bin/sh

NDK_DIR=${HOME}/android/android-ndk-r19c

#	V=1 \
#	-B \
$NDK_DIR/ndk-build \
	NDK_PROJECT_PATH=. \
	APP_BUILD_SCRIPT=./Android.mk \
	NDK_APPLICATION_MK=./Application.mk \
	${empty}

exit 0

 


