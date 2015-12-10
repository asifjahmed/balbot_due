// LIBRARY INCLUDES
#include <Encoder.h>
#include <DueFlashStorage.h>
#include <DualVNH5019MotorShieldDue.h> // Using 12-bit PWM and 20,000KHz frequency
#include <RC_RX.h>

// LOCAL INCLUDES
#include "kalman_filter.h"

// DEFINES
#define gyroYPin A5
#define accelZPin A4
#define accelYPin A3
#define accelYDirection -1
#define accelZDirection 1
#define gyroYDirection 1
// TODO: Do better calibration (these are very rough values), and store these in flash
//#define accelYOffset 2030
//#define accelZOffset 2554
//#define gyroYOffset 1659
// TODO: Verify this calculation (based on ADXL335 datasheet)
#define gravity 409
#define gyroDegrees(x) (x / 2.4824)
#define gyroRadians(x) (x * 0.007031) // Gyro raw in radians/sec
#define toRadians(x) (x / 57.2958)
#define toDegrees(x) (x * 57.2958)
// TODO: Make calibration routine, and store in flash
#define steeringMin 1030
#define steeringMax 2025
#define steeringCenterOn 1534
#define steeringCenterOff 1515
#define throttleMin 1005
#define throttleMax 2040
#define throttleCenterOn 1527
#define throttleCenterOff 1507

#define SERIAL_STREAM 1
#define PRINT_RAW_SENSOR_VALUES 0
#define PRINT_NORMALIZED_SENSOR_VALUES 1
#define PRINT_SCALED_SENSOR_VALUES 1
#define PRINT_KALMAN_FILTER_VALUES 1
#define PRINT_ENCODER_VALUES 1
#define PRINT_RC_RX_RAW_VALUES 0
#define PRINT_RC_RX_FILTERED_VALUES 1
#define PRINT_PID_GAIN_VALUES 1


// GLOBAL VARS
int16_t accelYOffset = 0;
int16_t accelZOffset = 0;
int16_t gyroYOffset = 0;

int16_t accelY = 0;
int16_t accelZ = 0;
int16_t gyroY = 0;

uint32_t last;
uint32_t oneKilohertz = 0;
uint32_t tenHertz = 0;

long leftEncLastPosition = 0;
long rightEncLastPosition = 0;
int16_t leftEncSpeed = 0;
int16_t rightEncSpeed = 0;

int16_t steeringVal = 0;
int16_t throttleVal = 0;

struct reg_map {
    float pgain;
    float igain;
    float dgain;
};

static union {
    reg_map map;
    uint8_t bytes[];
} _buffer;

// GLOBAL OBJECTS
Encoder leftEnc(38, 39);
Encoder rightEnc(43, 42);
DualVNH5019MotorShield motors;
RC_RX steering(24);
RC_RX throttle(26);
DueFlashStorage flash;
kalman_filter_data kalData;
