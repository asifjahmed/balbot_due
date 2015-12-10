void updateIMUData() {
    accelY = (analogRead(accelYPin) - accelYOffset) * accelYDirection;
    accelZ = (analogRead(accelZPin) - (accelZOffset - gravity)) * accelZDirection;
    gyroY = (analogRead(gyroYPin) - gyroYOffset) * gyroYDirection;
}

void calibrateSensors() {
    long accelYCalibrationVal = 0;
    long accelZCalibrationVal = 0;
    long gyroYCalibrationVal = 0;
  
    Serial.println("Sensor Calibration - Make sure IMU board is held still! This will take 20 seconds or so.");
    delay(1000);
    
    for (int x=0; x<1000; x++) {
        accelYCalibrationVal += analogRead(accelYPin);
        delay(1);
        accelZCalibrationVal += analogRead(accelZPin);
        delay(1);
        gyroYCalibrationVal += analogRead(gyroYPin);
        delay(20);
    }
    accelYOffset = (int16_t)abs(accelYCalibrationVal / 1000);
    accelZOffset = (int16_t)abs(accelZCalibrationVal / 1000);
    gyroYOffset = (int16_t)abs(gyroYCalibrationVal / 1000);
    writeFloatToFlash((float)accelYOffset, ACCELY_OFFSET_ADR);
    writeFloatToFlash((float)accelZOffset, ACCELZ_OFFSET_ADR);
    writeFloatToFlash((float)gyroYOffset, GYROY_OFFSET_ADR);
    Serial.println("Finished. Results:");
    Serial.print("aY:");
    Serial.print(accelYOffset);
    Serial.print("\taZ:");
    Serial.print(accelZOffset);
    Serial.print("\tgY:");
    Serial.println(gyroYOffset);
    delay(2000);
    kalman_filter_init(&kalData);
}

