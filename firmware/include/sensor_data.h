/*
 * sensor_data.h - one sample of sensor readings, shared by every module.
 *
 * The hardware layer fills one of these per sample and passes it in. The
 * flight software never reads sensors itself.
 */
#ifndef TVR1_SENSOR_DATA_H
#define TVR1_SENSOR_DATA_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t time_ms;        /* ms since boot */
    float    accel_axial_g;  /* acceleration along the long axis, +ve toward
                                the nose, in g; reads about +1 at rest on the pad */
    float    accel_mag_g;    /* magnitude of the 3-axis acceleration, in g;
                                about 1 at rest in any orientation */
    float    altitude_m;     /* barometric altitude above the launch site, m */
    bool     accel_valid;    /* false if the IMU reading is missing or bad
                                (covers both acceleration fields) */
    bool     baro_valid;     /* false if the barometer reading is missing or bad */
} SensorData;

#endif /* TVR1_SENSOR_DATA_H */
