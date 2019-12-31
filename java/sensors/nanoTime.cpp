
#include <stdint.h>
#include <time.h>

extern "C" uint64_t nanoTime();

// Number of nanoseconds in a second.
#define NS_PER_SEC              1000000000

// Get the current time in nanoseconds.
uint64_t nanoTime() {
  struct timespec t;

  t.tv_sec = t.tv_nsec = 0;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<uint64_t>(t.tv_sec) * NS_PER_SEC + t.tv_nsec;
}

