#ifndef kalman_filter_h
#define kalman_filter_h

/*
 * R_ANGLE represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect R_ANGLE radians of jitter
 * from the accelerometer.
 */
#define R_ANGLE 0.3

/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the accelerometers
 * relative to the gyros.
 */
#define Q_ACCEL 0.005
#define Q_GYRO 0.015

struct kalman_filter_data {
    float P[2][2];
    float angle;
    float q_bias;
    float rate;
    float Pdot[4];
    float err;
};

void kalman_filter_init(kalman_filter_data p_kd);

void kalman_filter_state_update (
    const float		q_m, // measured rotation rate from gyro, in radians/sec
    kalman_filter_data p_kd,
    float dt
);

void kalman_filter_angle_update (
    const float		angle_m, // measured angle from accelerometer, in radians
    kalman_filter_data p_kd
);

#endif
