/*
 * Balbot Due - my latest attempt at a balancing robot
 * 12/2015
 * Jamie Ahmed
 */

// Put all defines/constants/declarations in a header file for cleanliness
#include "balbot_due.h"

void setup() {
    // Initialize serial port
    Serial.begin(38400);
    // Set ADC to 12-bit resolution
    analogReadResolution(12);
    // Initialize Pololu dual motor shield
    motors.init();
    // Initialize kalman filter struct
    kalman_filter_init(&kalData);
    // Load any stored parameters from flash
    loadDataFromFlash();

    // to keep me from having to calibrate after each sketch :-)
    if(accelYOffset == 0) {
        accelYOffset = 2021;
        accelZOffset = 2555;
        gyroYOffset = 1659;
        // TODO: Put the rest of the PID values et al here.
    }
    
}

void loop() {
    float dt;

    // 1KHz loop
    if (micros() - oneKilohertz >= 1000) {
        oneKilohertz = micros();
        
        // Read and normalize our gyro and accel sensors
        updateIMUData();

        // Calculate dt (delta time) - how long since the last loop, in seconds
        dt = (float)(micros() - last) / 1000000.0;
        last = micros();
      
        // Update kalman filter state with new gyro data
        kalman_filter_state_update(gyroRadians(gyroY), &kalData, dt);
      
        // Calculate the accelerometer angle using both Y & Z axes
        float angleY = atan2(accelY, accelZ);
      
        // Update kalman filter with accelrometer angle
        kalman_filter_angle_update(angleY, &kalData);
    }
  
    // 10Hz loop
    if (micros() - tenHertz >= 100000) {
        dt = (float)(micros() - tenHertz) / 1000000.0;
        tenHertz = micros();
    
        // Get current speed readings from motor encoders
        updateEncoderSpeeds(dt);

        // Update steering & throttle inputs from RC receiver
        readRC();
    
        // Print serial stream data and process any incoming data
        printSerialData();
        readSerialData();
    }
}
