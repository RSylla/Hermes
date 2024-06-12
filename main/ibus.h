#ifndef IBUS_H
#define IBUS_H

#include <Arduino.h>

#define BUFFERSIZE 32
#define FIRST_BYTE 0x20
#define SECOND_BYTE 0x40
#define MAX_CHANNELS 14
#define MIN_PULSE 1000  // min valid impulse us
#define MAX_PULSE 2000  // max valid impulse us

extern byte iBus[BUFFERSIZE];

bool readIBUS();
uint16_t getChannelData(byte channel);
int convertSteeringData(int channel, int minLimit, int maxLimit, int defaultValue);
int convertThrottleData(int channel, int minLimit, int maxLimit, int defaultValue);

#endif
