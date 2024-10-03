#include "ibus.h"

byte iBus[BUFFERSIZE] = {0}; // iBUS packet buffer

bool readIBUS() {
    static byte iBusIndex = 0;
    static uint16_t checksum = 0xFFFF;
    while (Serial1.available()) {
        byte val = Serial1.read();
        if (iBusIndex == 0 && val != FIRST_BYTE) continue;  // Check for first byte
        if (iBusIndex == 1 && val != SECOND_BYTE) {         // Check for second byte
            iBusIndex = 0;
            checksum = 0xFFFF;
            continue;
        }

        iBus[iBusIndex] = val;
        checksum -= (iBusIndex < BUFFERSIZE - 2) ? val : 0;  // Update checksum
        iBusIndex++;

        if (iBusIndex == BUFFERSIZE) {
            if (checksum == (uint16_t)((iBus[31] << 8) | iBus[30])) {
                iBusIndex = 0;
                checksum = 0xFFFF;
                return true;  // Packet is complete and checksum is correct
            }
            iBusIndex = 0;
            checksum = 0xFFFF;  // Reset on checksum failure
        }
    }
    return false;
}

uint16_t getChannelData(byte channel) {
    if (channel < 1 || channel > MAX_CHANNELS) return 0;  // Check channel bounds
    byte index = 2 * (channel - 1) + 2;
    return iBus[index] | (iBus[index + 1] << 8);  // Little endian conversion
}

int convertSteeringData(int channel, int minLimit, int maxLimit, int defaultValue) {
    uint16_t chData = getChannelData(channel);
    if (chData < MIN_PULSE || chData > MAX_PULSE) return defaultValue;  // Validate channel range
    int mappedValue = map(chData, MIN_PULSE, MAX_PULSE, minLimit, maxLimit);
    return constrain(mappedValue, minLimit, maxLimit);  // Constrain the value to the provided range
}

int convertThrottleData(int channel, int minLimit, int maxLimit, int defaultValue) {
    uint16_t chData = getChannelData(channel);
    if (chData < MIN_PULSE || chData > MAX_PULSE) return defaultValue;  // Validate channel range
    int mappedValue = map(chData, MIN_PULSE, MAX_PULSE, minLimit, maxLimit);
    return constrain(mappedValue, minLimit, maxLimit);  // Constrain the value to the provided range
}
