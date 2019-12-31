/* */
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "head_tracker.h"

// Copy from cardboard.h

#ifdef __cplusplus
extern "C" {
#endif

/// An opaque Head Tracker object.
typedef struct CardboardHeadTracker CardboardHeadTracker;

/// Head Tracker functions.

/// Creates a new head tracker object.
///
/// @return         head tracker object pointer
CardboardHeadTracker* CardboardHeadTracker_create();

/// Destroys and releases memory used by the provided head tracker object.
///
/// @param[in]      head_tracker            Head tracker object pointer.
void CardboardHeadTracker_destroy(CardboardHeadTracker* head_tracker);

/// Pauses head tracker and underlying device sensors.
///
/// @param[in]      head_tracker            Head tracker object pointer.
void CardboardHeadTracker_pause(CardboardHeadTracker* head_tracker);

/// Resumes head tracker and underlying device sensors.
///
/// @param[in]      head_tracker            Head tracker object pointer.
void CardboardHeadTracker_resume(CardboardHeadTracker* head_tracker);

/// Gets the predicted head pose for a given timestamp.
///
/// @param[in]      head_tracker            Head tracker object pointer.
/// @param[in]      timestamp_ns            The timestamp for the pose in
///     nanoseconds in system monotonic clock.
/// @param[out]     position                3 floats for (x, y, z).
/// @param[out]     orientation             4 floats for quaternion
void CardboardHeadTracker_getPose(CardboardHeadTracker* head_tracker,
                                  int64_t timestamp_ns, float* position,
                                  float* orientation);

#ifdef __cplusplus
}
#endif

// Copy from cardboard.cc

struct CardboardHeadTracker : cardboard::HeadTracker {};

extern "C" {

CardboardHeadTracker* CardboardHeadTracker_create() {
  return reinterpret_cast<CardboardHeadTracker*>(new cardboard::HeadTracker());
}

void CardboardHeadTracker_destroy(CardboardHeadTracker* head_tracker) {
  delete head_tracker;
}

void CardboardHeadTracker_pause(CardboardHeadTracker* head_tracker) {
  static_cast<cardboard::HeadTracker*>(head_tracker)->Pause();
}

void CardboardHeadTracker_resume(CardboardHeadTracker* head_tracker) {
  static_cast<cardboard::HeadTracker*>(head_tracker)->Resume();
}

void CardboardHeadTracker_getPose(CardboardHeadTracker* head_tracker,
                                  int64_t timestamp_ns, float* position,
                                  float* orientation) {
  std::array<float, 3> out_position;
  std::array<float, 4> out_orientation;
  static_cast<cardboard::HeadTracker*>(head_tracker)
      ->GetPose(timestamp_ns, out_position, out_orientation);
  std::memcpy(position, &out_position[0], 3 * sizeof(float));
  std::memcpy(orientation, &out_orientation[0], 4 * sizeof(float));
}

}  // extern "C"

/*********************************************************************/
// Get the current time in nanoseconds.
extern "C" uint64_t nanoTime();

/** orientation can be non-normalised quaternion */
void toEuler(float orientation[4], float euler[3]) {
  float x = orientation[0];
  float y = orientation[1];
  float z = orientation[2];
  float w = orientation[3];
  float sqw = w * w;
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;
  float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
  float test = x * y + z * w;
#define heading		euler[0]
#define attitude	euler[1]
#define bank		euler[2]
  if (test > 0.499 * unit) { // singularity at north pole
    heading  = 2 * atan2(x, w);
    attitude = M_PI / 2;
    bank     = 0;
    return;
  }
  if (test < -0.499 * unit) { // singularity at south pole
    heading  = -2 * atan2(x, w);
    attitude = -M_PI / 2;
    bank     =  0;
    return;
  }

  heading  = atan2(2 * y * w - 2 * x * z ,  sqx - sqy - sqz + sqw);
  attitude = asin (2 * test / unit);
  bank     = atan2(2 * x * w - 2 * y * z , -sqx + sqy - sqz + sqw);
#undef heading
#undef attitude
#undef bank
}

int main(int argc, char *argv[])
{
  CardboardHeadTracker* head_tracker = CardboardHeadTracker_create();

  CardboardHeadTracker_resume(head_tracker);
  sleep(1);

  printf ("start testing...\n");

  for (int i = 0; i < 20; i++) {
    int64_t timestamp_ns;
    float position[3];
    float orientation[4];
    float euler[3];

    timestamp_ns = nanoTime();
    CardboardHeadTracker_getPose(head_tracker, timestamp_ns, position, orientation);
    // orientation is a normalized quaternion, the vector part is in the 
    // first 3 elements, and the scalar part is in the last element.
    printf("orientation: x = %f, y = %f, z = %f, w = %f\n", orientation[0], orientation[1], orientation[2], orientation[3]);
    toEuler(orientation, euler);
    printf("yaw = %f◦, pitch = %f◦, roll = %f◦\n", euler[0] * 180.0 / M_PI, euler[1] * 180.0 / M_PI, euler[2] * 180.0 / M_PI);

    sleep(1);
  }

  CardboardHeadTracker_pause(head_tracker);

  CardboardHeadTracker_destroy(head_tracker);

  printf ("end of test.\n");

  return 0;
}
