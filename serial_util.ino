void printSerialData() {
    // TODO: Break this out into logical areas, and make each optional through DEFINEs
    if (SERIAL_STREAM) {
      
        if (PRINT_RAW_SENSOR_VALUES) {
            Serial.print("\tgY:");
            Serial.print(analogRead(gyroYPin));
            Serial.print("\taY:");
            Serial.print(analogRead(accelYPin));
            Serial.print("\taZ:");
            Serial.print(analogRead(accelZPin));
        }
        
        if (PRINT_NORMALIZED_SENSOR_VALUES) {
            Serial.print("\tgY:");
            Serial.print(gyroY);
            Serial.print("\taY:");
            Serial.print(accelY);
            Serial.print("\taZ:");
            Serial.print(accelZ);
        }
    
        if (PRINT_SCALED_SENSOR_VALUES) {
            Serial.print("\trateY:");
            Serial.print(gyroDegrees(gyroY));
            Serial.print("\tangleY:");
            Serial.print(toDegrees(atan2(accelY, accelZ)));
        }
    
        if (PRINT_KALMAN_FILTER_VALUES) {
            Serial.print("\tkalman:");
            Serial.print(toDegrees(kalData.angle));
        }
    
        if (PRINT_ENCODER_VALUES) {
            Serial.print("\tleftPos:");
            Serial.print(leftEnc.read());
            Serial.print("\trightPos:");
            Serial.print(rightEnc.read());
            Serial.print("\tleftSpeed:");
            Serial.print(leftEncSpeed);
            Serial.print("\trightSpeed:");
            Serial.print(rightEncSpeed);
        }
    
        if (PRINT_RC_RX_RAW_VALUES) {
            Serial.print("\tsteeringRaw:");
            Serial.print(steering.read());
            Serial.print("\tthrottleRaw:");
            Serial.print(throttle.read());
        }
    
        if (PRINT_RC_RX_FILTERED_VALUES) {
            Serial.print("\tsteering:");
            Serial.print(steeringVal);
            Serial.print("\tthrottle:");
            Serial.print(throttleVal);
        }
    
        if (PRINT_PID_GAIN_VALUES) {
          Serial.print("\tpgain:");
          Serial.print(_buffer.map.pgain);
          Serial.print("\tigain:");
          Serial.print(_buffer.map.igain);
          Serial.print("\tdgain:");
          Serial.print(_buffer.map.dgain);
        }

        Serial.println();
    }
}

void readSerialData() {
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'p':
                if (Serial.available() == 12) {
                    for (int x=0; x<12;x++) {
                        _buffer.bytes[x] = Serial.read();
                    }
                    writeFloatToFlash(_buffer.map.pgain, PGAIN_ADR);
                    writeFloatToFlash(_buffer.map.igain, IGAIN_ADR);
                    writeFloatToFlash(_buffer.map.dgain, DGAIN_ADR);
                }
                break;
            case 'o':
                writeFloatToFlash((float)accelYOffset, ACCELY_OFFSET_ADR);
                writeFloatToFlash((float)accelZOffset, ACCELZ_OFFSET_ADR);
                writeFloatToFlash((float)gyroYOffset, GYROY_OFFSET_ADR);
                break;
            case 'c':
                calibrateSensors();
                break;
        }
    }
}
