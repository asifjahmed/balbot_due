void readRC() {
    // disposable var, gets used for both steering and throttle
    int16_t tmpVal = steering.read();
    
    if (tmpVal < min(steeringCenterOn, steeringCenterOff)-2 || tmpVal > max(steeringCenterOn, steeringCenterOff)+2) {
        steeringVal = map(tmpVal, steeringMin, steeringMax, -500, 500);
    } else {
        steeringVal = 0;
    }

    tmpVal = throttle.read();

    if (tmpVal < min(throttleCenterOn, throttleCenterOff)-2 || tmpVal > max(throttleCenterOn, throttleCenterOff)+2) {
        throttleVal = map(tmpVal, throttleMin, throttleMax, -500, 500);
    } else {
        throttleVal = 0;
    }
}

