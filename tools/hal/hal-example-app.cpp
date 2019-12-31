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

/* This file contains an example app that uses sensors HAL.
 * It accepts a sensor type as an input argument and reads data from a
 * sensor of that type.
 *
 * For any specified sensor type, there may be multiple such sensors defined
 * in the HAL. Some or all of the defined sensors will actually be equipped.
 * This test program will scan the HAL for the first equipped sensor of
 * the specified sensor type and dump data from that sensor.
 */

#include <getopt.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include <sys/types.h>

#define DEFAULT_SENSOR_TYPE SENSOR_TYPE_ACCELEROMETER

// Structure to hold the decoded command line options
struct pgm_options {
  int    sensor_type;
};

// Be sure to keep the options for longopts and shortopts in the same order
// so that Usage() is correct.
static struct option longopts[] = {
  {"help",     no_argument,  NULL,  '?'},
  {"accel",    no_argument,  NULL,  'a'},
  {"temp",     no_argument,  NULL,  't'},
  {"light",    no_argument,  NULL,  'l'},
  {"orient",   no_argument,  NULL,  'o'},
  {"prox",     no_argument,  NULL,  'p'},
  {"motion",   no_argument,  NULL,  'm'},
  {NULL,       0,            NULL,   0}
};
static char shortopts[] = "?atlopm";

// Describes the options for this program.
void Usage(char *pgm_name) {
  printf("Usage: %s [options...]\n", pgm_name);
  printf("Exercises the sensors HAL by direct calling.\n");
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
      options->sensor_type  = SENSOR_TYPE_ACCELEROMETER;
      break;
    case 't':
      options->sensor_type  = SENSOR_TYPE_TEMPERATURE;
      break;
    case 'l':
      options->sensor_type  = SENSOR_TYPE_LIGHT;
      break;
    case 'o':
      options->sensor_type  = SENSOR_TYPE_ORIENTATION;
      break;
    case 'p':
      options->sensor_type  = SENSOR_TYPE_PROXIMITY;
      break;
    case 'm':
      options->sensor_type  = SENSOR_TYPE_SIGNIFICANT_MOTION;
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
void DisplaySensorData(int sensor_type, const sensors_event_t *data) {
  switch (sensor_type) {
    case SENSOR_TYPE_PROXIMITY:
      printf("Proximity distance: %f\n", data->distance);
      break;
    case SENSOR_TYPE_SIGNIFICANT_MOTION:
      if (data->data[0] == 1) {
        printf("Significant motion detected\n");
      }
      break;
    case SENSOR_TYPE_ACCELEROMETER:
      printf("Acceleration: x = %f, y = %f, z = %f\n",
             data->acceleration.x, data->acceleration.y, data->acceleration.z);
      break;
    case SENSOR_TYPE_TEMPERATURE:
      printf("Temperature: %f\n", data->temperature);
      break;
    case SENSOR_TYPE_LIGHT:
      printf("Light: %f\n", data->light);
      break;
    case SENSOR_TYPE_ORIENTATION: {
      float heading =
        atan2(static_cast<double>(data->orientation.y),
              static_cast<double>(data->orientation.x)) * 180.0 / M_PI;
      if (heading < 0.0)
        heading += 360.0;
      printf("Heading: %f, Orientation: x = %f, y = %f, z = %f\n",
             heading, data->orientation.x, data->orientation.y,
             data->orientation.z);
      }
      break;
  }
}

int main(int argc, char* argv[]) {
  pgm_options options = {DEFAULT_SENSOR_TYPE};
  if (ReadOpts(argc, argv, &options) < 0)
    return 1;

  sensors_module_t* sensor_module = nullptr;
  int ret = hw_get_module(SENSORS_HARDWARE_MODULE_ID,
      const_cast<hw_module_t const**>(
      reinterpret_cast<hw_module_t**>(&sensor_module)));
  if (ret || !sensor_module) {
    fprintf(stderr, "Failed to load %s module: %s\n",
            SENSORS_HARDWARE_MODULE_ID, strerror(-ret));
    return 1;
  }

  const sensor_t *sensor_list = nullptr;
  int sensor_count =
      sensor_module->get_sensors_list(sensor_module, &sensor_list);
  printf("Found %d supported sensors\n", sensor_count);
  for (int i = 0; i < sensor_count; i++) {
    printf("HAL supports sensor %s\n", sensor_list[i].name);
  }

  // sensors_open_1 is used in HAL versions >= 1.0.
  sensors_poll_device_1_t* sensor_device = nullptr;
  ret = sensors_open_1(&sensor_module->common, &sensor_device);
  if (ret || !sensor_device) {
    fprintf(stderr, "Failed to open the sensor HAL\n");
    return 1;
  }

  // Find the first sensor of the specified type that can be opened
  bool sensor_found = false;
  const sensor_t *sensor = nullptr;
  for (int i = 0; i < sensor_count; i++) {
    sensor = &sensor_list[i];
    if (sensor->type != options.sensor_type)
      continue;
    if (sensor_device->activate(
        reinterpret_cast<sensors_poll_device_t*>(sensor_device),
        sensor->handle, 1 /* enabled */) < 0) {
      continue;
    }

    // Found an equipped sensor of the specified type.
    sensor_found = true;
    break;
  }

  if (!sensor_found) {
    fprintf(stderr, "No sensor of the specified type found\n");
    ret = sensors_close_1(sensor_device);
    if (ret)
      fprintf(stderr, "Failed to close the sensor device\n");
    return 1;
  }
  printf("\nSensor %s activated\n", sensor->name);

  const int kNumEvents = 1;
  const int kNumSamples = 10;
  const int kWaitTimeSecs = 1;
  for (int i = 0; i < kNumSamples; i++) {
    sensors_event_t data[kNumEvents];
    memset(data, 0, sizeof(data));
    int event_count = sensor_device->poll(
        reinterpret_cast<sensors_poll_device_t*>(sensor_device),
        data, kNumEvents);
    if (!event_count) {
      fprintf(stderr, "Failed to read data from the sensor.\n");
      continue;
    }

    DisplaySensorData(options.sensor_type, data);
    sleep(kWaitTimeSecs);
  }

  ret = sensor_device->activate(
      reinterpret_cast<sensors_poll_device_t*>(sensor_device),
      sensor->handle, 0 /* disabled */);
  if (ret) {
    fprintf(stderr, "Failed to disable %s: %s\n", sensor->name, strerror(ret));
    return 1;
  }

  // sensors_close_1 is used in HAL versions >= 1.0.
  ret = sensors_close_1(sensor_device);
  if (ret) {
    fprintf(stderr, "Failed to close the sensor device\n");
    return 1;
  }

  return 0;
}
