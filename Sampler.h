#pragma once

#include "daisy_seed.h"
#include "daisysp.h"

void setup(void);

void reset(void);

void startRecording(void);
void stopRecording(void);

void incrementPlayHead(void);

bool determinePlaySpeed(void);

void clearBuffer(void);
float readFromBuffer(int);


void flashLed(size_t);