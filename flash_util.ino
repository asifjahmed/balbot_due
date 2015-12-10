/*
 * 
 *  NOTE: On Arduino Due, flash storage is used instead of the usual EEPROM.
 *  This means that the stored flash data will be ERASED with each new sketch upload!
 *  
 */

// Flash storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define ACCELY_OFFSET_ADR 12
#define ACCELZ_OFFSET_ADR 16
#define GYROY_OFFSET_ADR 20

// Utilities for writing and reading from the Due's Flash Storage
float readFloatFromFlash(int address) {
    union floatStore {
        byte floatByte[4];
        float floatVal;
    } floatOut;
    
    for (int i = 0; i < 4; i++) {
        floatOut.floatByte[i] = flash.read(address + i);
    }
    return floatOut.floatVal;
}

void writeFloatToFlash(float value, int address) {
    union floatStore {
        byte floatByte[4];
        float floatVal;
    } floatIn;
    
    floatIn.floatVal = value;
    for (int i = 0; i < 4; i++) {
        flash.write(address + i, floatIn.floatByte[i]);
    }
}

void loadDataFromFlash() {
    _buffer.map.pgain = readFloatFromFlash(PGAIN_ADR);
    _buffer.map.igain = readFloatFromFlash(IGAIN_ADR);
    _buffer.map.dgain = readFloatFromFlash(DGAIN_ADR);
    accelYOffset = (int16_t)readFloatFromFlash(ACCELY_OFFSET_ADR);
    accelZOffset = (int16_t)readFloatFromFlash(ACCELZ_OFFSET_ADR);
    gyroYOffset = (int16_t)readFloatFromFlash(GYROY_OFFSET_ADR);
}
