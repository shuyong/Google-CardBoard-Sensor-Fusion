#ifndef SENSOREVENT_H
#define SENSOREVENT_H

enum {
    SENSOR_TYPE_ACCELEROMETER      = 1,
    SENSOR_TYPE_MAGNETIC_FIELD     = 2,
    SENSOR_TYPE_GYROSCOPE          = 4,
};

typedef struct SensorEvent {
    int type;
    int flag;
    long timestamp;
    float values[4];
} SensorEvent;

#endif // SENSOREVENT_H


