/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This file contains an example app that uses sensors NDK.
 * It accepts a sensor type as an input argument and reads data from a
 * sensor of that type.
 *
 * For any specified sensor type, there may be multiple such sensors defined
 * in the HAL. Some or all of the defined sensors will actually be equipped.
 * This test program will scan the HAL for the first equipped sensor of
 * the specified sensor type and dump data from that sensor.
 */

#include <assert.h>
#include <getopt.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <android/looper.h>
#include <android/sensor.h>

#ifndef SENSOR_TYPE_ORIENTATION
#define SENSOR_TYPE_ORIENTATION                      (3)
#endif
#ifndef SENSOR_TYPE_TEMPERATURE
#define SENSOR_TYPE_TEMPERATURE                      (7)
#endif

#define LOOPER_ID_USER	3

#define DEFAULT_ASENSOR_TYPE ASENSOR_TYPE_ACCELEROMETER

// Structure to hold the decoded command line options
struct pgm_options {
  int    sensor_type;
};

// Be sure to keep the options for longopts and shortopts in the same order
// so that Usage() is correct.
static struct option longopts[] = {
  {"help",     no_argument,  NULL,  '?'},
  {"accel",    no_argument,  NULL,  'a'},
  {"gyroscope",no_argument,  NULL,  'g'},
  {"light",    no_argument,  NULL,  'l'},
  {"magnetic", no_argument,  NULL,  'm'},
  {"orient",   no_argument,  NULL,  'o'},
  {"pressure", no_argument,  NULL,  'p'},
  {"rotation", no_argument,  NULL,  'r'},
  {"motion",   no_argument,  NULL,  's'},
  {"temp",     no_argument,  NULL,  't'},
  {NULL,       0,            NULL,   0}
};
static char shortopts[] = "?aglmoprst";

// Describes the options for this program.
void Usage(char *pgm_name) {
  printf("Usage: %s [options...]\n", pgm_name);
  printf("Exercises the sensors NDK by calling into the sensorserver.\n");
  printf("Options:\n");
  for (int i = 0; longopts[i].name; i++) {
    printf("  --%-6s or -%c\n", longopts[i].name, shortopts[i]);
  }
}

// Processes all command line options.
//   sets the options members for commnd line options
//   returns (0) on success, -1 otherwise.
int ReadOpts(int argc, char **argv, struct pgm_options *options) {
  int ch = 0;

  if (!options) {
    fprintf(stderr, "Invalid options pointer\n");
    return 1;
  }

  while ((ch = getopt_long(argc, argv, shortopts, longopts, NULL)) != -1) {
    switch (ch) {
    case 'a':
      options->sensor_type  = ASENSOR_TYPE_ACCELEROMETER;
      break;
    case 'g':
      options->sensor_type  = ASENSOR_TYPE_GYROSCOPE;
      break;
    case 'l':
      options->sensor_type  = ASENSOR_TYPE_LIGHT;
      break;
    case 'm':
      options->sensor_type  = ASENSOR_TYPE_MAGNETIC_FIELD;
      break;
     case 'o':
      options->sensor_type  = SENSOR_TYPE_ORIENTATION;
      break;
    case 'p':
      options->sensor_type  = ASENSOR_TYPE_PRESSURE;
      break;
    case 'r':
      options->sensor_type  = ASENSOR_TYPE_ROTATION_VECTOR;
      break;
    case 's':
      options->sensor_type  = ASENSOR_TYPE_SIGNIFICANT_MOTION;
      break;
    case 't':
      options->sensor_type  = SENSOR_TYPE_TEMPERATURE;
      break;
    default:
      Usage(argv[0]);
      return -1;
    }
  }
  argc -= optind;
  argv += optind;
  return 0;
}

// Prints data associated with each supported sensor type.
// Be sure to provide a case for each sensor type supported in
// the ReadOpts() function.
void DisplaySensorData(int sensor_type, const ASensorEvent *data) {
  switch (sensor_type) {
    case ASENSOR_TYPE_ACCELEROMETER:
      printf("Acceleration (m/s^2): x = %f, y = %f, z = %f\n",
             data->acceleration.x, data->acceleration.y, data->acceleration.z);
      break;
    case ASENSOR_TYPE_GYROSCOPE:
      printf("Gyroscope (rad/s): x = %f, y = %f, z = %f\n",
             data->vector.x, data->vector.y, data->vector.z);
      break;
    case ASENSOR_TYPE_LIGHT:
      printf("Light: %f\n", data->light);
      break;
    case ASENSOR_TYPE_MAGNETIC_FIELD:
      printf("Magnetic (uT): x = %f, y = %f, z = %f\n",
             data->magnetic.x, data->magnetic.y, data->magnetic.z);
      break;
    case SENSOR_TYPE_ORIENTATION: {
      float heading =
        atan2(static_cast<double>(data->vector.y),
              static_cast<double>(data->vector.x)) * 180.0 / M_PI;
      if (heading < 0.0)
        heading += 360.0;
      printf("Heading: %f, Orientation: x = %f, y = %f, z = %f\n",
             heading, data->vector.x, data->vector.y, data->vector.z);
      }
      break;
    case ASENSOR_TYPE_PRESSURE:
      printf("Pressure in hPa (millibar): %f\n", data->pressure);
      break;
    case ASENSOR_TYPE_ROTATION_VECTOR: {
      float w;
      w = sqrtf(1 - (data->vector.x * data->vector.x +
	             data->vector.y * data->vector.y +
	             data->vector.z * data->vector.z));
      printf("Quaternion: w = %f, x = %f, y = %f, z = %f\n",
             w, data->vector.x, data->vector.y, data->vector.z);
      }
      break;
    case ASENSOR_TYPE_SIGNIFICANT_MOTION:
      if (data->data[0] == 1) {
        printf("Significant motion detected\n");
      }
      break;
    case SENSOR_TYPE_TEMPERATURE:
      printf("Temperature: %f\n", data->temperature);
      break;
   }
}

/*
 * AcquireASensorManagerInstance(void)
 *    Workaround ASensorManager_getInstance() deprecation false alarm
 *    for Android-N and before, when compiling with NDK-r15
 */
#include <dlfcn.h>
ASensorManager* AcquireASensorManagerInstance(const char* packageName) {

  if(!packageName)
    return NULL;

  typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
  void* androidHandle = dlopen("libandroid.so", RTLD_NOW);
  PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc = (PF_GETINSTANCEFORPACKAGE)
      dlsym(androidHandle, "ASensorManager_getInstanceForPackage");
  if (getInstanceForPackageFunc) {
    ASensorManager* mgr = getInstanceForPackageFunc(packageName);
    if (mgr) {
      dlclose(androidHandle);
      return mgr;
    }
  }

  typedef ASensorManager *(*PF_GETINSTANCE)();
  PF_GETINSTANCE getInstanceFunc = (PF_GETINSTANCE)
      dlsym(androidHandle, "ASensorManager_getInstance");
  // by all means at this point, ASensorManager_getInstance should be available
  assert(getInstanceFunc);
  dlclose(androidHandle);

  return getInstanceFunc();
}

int main(int argc, char* argv[]) {
  pgm_options options = {DEFAULT_ASENSOR_TYPE};
  if (ReadOpts(argc, argv, &options) < 0)
    return 1;

  const char kPackageName[] = "ndk-example-app";
  ASensorManager* sensor_manager =
    AcquireASensorManagerInstance(kPackageName);
  if (!sensor_manager) {
    fprintf(stderr, "Failed to get a sensor manager\n");
    return 1;
  }

  ASensorList sensor_list = nullptr;
  int sensor_count = ASensorManager_getSensorList(sensor_manager, &sensor_list);
  printf("Found %d supported sensors\n", sensor_count);
  for (int i = 0; i < sensor_count; i++) {
    printf("HAL supports sensor: %s, minimum delay (microseconds) = %d\n", ASensor_getName(sensor_list[i]), ASensor_getMinDelay(sensor_list[i]));
  }

  const int kLooperId = LOOPER_ID_USER;
  ALooper* looper = ALooper_forThread();
  ASensorEventQueue* queue = ASensorManager_createEventQueue(
      sensor_manager,
      looper ? looper : ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS),
      kLooperId,
      NULL, /* no callback */
      NULL  /* no private data for a callback  */);
  if (!queue) {
    fprintf(stderr, "Failed to create a sensor event queue\n");
    return 1;
  }

  // Find the first sensor of the specified type that can be opened
  const int kTimeoutMicroSecs = 1000000;
  const int kTimeoutMilliSecs = 1000;
  ASensorRef sensor = nullptr;
  bool sensor_found = false;
  for (int i = 0; i < sensor_count; i++) {
    sensor = sensor_list[i];
    if (ASensor_getType(sensor) != options.sensor_type)
      continue;
    if (ASensorEventQueue_enableSensor(queue, sensor) < 0)
      continue;
    if (ASensorEventQueue_setEventRate(queue, sensor, kTimeoutMicroSecs) < 0) {
      fprintf(stderr, "Failed to set the %s sample rate\n",
          ASensor_getName(sensor));
      return 1;
    }

    // Found an equipped sensor of the specified type.
    sensor_found = true;
    break;
  }

  if (!sensor_found) {
    fprintf(stderr, "No sensor of the specified type found\n");
    int ret = ASensorManager_destroyEventQueue(sensor_manager, queue);
    if (ret < 0)
      fprintf(stderr, "Failed to destroy event queue: %s\n", strerror(-ret));
    return 1;
  }
  printf("\nSensor %s activated\n", ASensor_getName(sensor));

  const int kNumEvents = 1;
  const int kNumSamples = 20;
  const int kWaitTimeSecs = 1;
  for (int i = 0; i < kNumSamples; i++) {
    ASensorEvent data[kNumEvents];
    memset(data, 0, sizeof(data));
    int ident = ALooper_pollAll(
        kTimeoutMilliSecs,
        NULL /* no output file descriptor */,
        NULL /* no output event */,
        NULL /* no output data */);
    if (ident != kLooperId) {
      fprintf(stderr, "Incorrect Looper ident read from poll.\n");
      continue;
    }
    if (ASensorEventQueue_getEvents(queue, data, kNumEvents) <= 0) {
      fprintf(stderr, "Failed to read data from the sensor.\n");
      continue;
    }

    DisplaySensorData(options.sensor_type, data);
    sleep(kWaitTimeSecs);
  }

  int ret = ASensorEventQueue_disableSensor(queue, sensor);
  if (ret < 0) {
    fprintf(stderr, "Failed to disable %s: %s\n",
            ASensor_getName(sensor), strerror(-ret));
  }

  ret = ASensorManager_destroyEventQueue(sensor_manager, queue);
  if (ret < 0) {
    fprintf(stderr, "Failed to destroy event queue: %s\n", strerror(-ret));
    return 1;
  }

  return 0;
}
