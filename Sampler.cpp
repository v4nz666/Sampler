// #define DEBUG


#include "Sampler.h"

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

#define L 0
#define R 1
#define A0 15



#define SAMPLE_RATE 48000
#define BUFFER_LENGTH 8


// sampler buffer
#define BUFFER_SIZE (SAMPLE_RATE * BUFFER_LENGTH)

float DSY_SDRAM_BSS sBuffer[2][BUFFER_SIZE];


// index into buffer, in 28.4 fixed point notation
// 	integer portion = playIndex >> 4;
// 	decimal portion = (playIndex & 0xf) / 16.0f;
uint32_t playIndex = 0;
float playSpeed = 0.5;

uint32_t recordIndex = 0;
uint32_t loopLength = 0;

bool loopingActive  = false;
bool recordingActive = false;



bool reachedEnd = false;
bool ledState = true;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	float recL, recR = 0.0f;
	float loopL, loopR = 0.0f;
	float loopDecay = 1.0f;

	for ( size_t i = 0; i < size; i++ ) {
		
		if ( recordingActive ) {
			recL = in[0][i];
			recR = in[1][i];

			sBuffer[L][recordIndex] = recL;
			sBuffer[R][recordIndex] = recR;

			recordIndex++;
			recordIndex %= BUFFER_SIZE;
			loopDecay = 0.9f;

			// If we're recording the first loop into the buffer, keep track of the length as we record
			if ( ! loopingActive ) {
				// And if we hit the end of the buffer, start looping, and cap our loopLength
				if ( ++loopLength >= BUFFER_SIZE ) {
					loopLength = BUFFER_SIZE;
					stopRecording();
					reachedEnd = true;
				}
				
			}
		}

		if ( loopingActive ) {	
			loopL = readFromBuffer(L);
			loopR = readFromBuffer(R);
			incrementPlayHead();
		}
		
		out[L][i] = recL + (loopL * loopDecay);
		out[R][i] = recR + (loopR * loopDecay);

		
		
	}
}

int main(void)
{
	setup();
	
	hw.StartAudio(AudioCallback);
	startRecording();
	
	while(1) {
		float pot = hw.adc.GetFloat(0);
		if ( pot < 0.01 ) {
			playSpeed = 0;
		} else if ( pot < 0.2 ) {
			playSpeed = 0.25;
		} else if ( pot < 0.4 ) {
			playSpeed = 0.5;
		} else if ( pot < 0.6 ) {
			playSpeed = 1;
		} else if ( pot < 0.8 ) {
			playSpeed = 2;
		} else {
			playSpeed = 4;
		}

		if ( reachedEnd ) {
			flashLed();
			reachedEnd = false;
		}

		hw.SetLed(recordingActive);
	}
}

void setup(void) {
	hw.Init();
	hw.SetAudioBlockSize(4);
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	
	#ifdef DEBUG
	hw.StartLog(true);
	hw.PrintLine("Log initialized");
	#endif
	
	AdcChannelConfig adcConfig;
	adcConfig.InitSingle(hw.GetPin(A0));
    hw.adc.Init(&adcConfig, 1);
    hw.adc.Start();

	reset();
	
}

void reset(void) {
	clearBuffer();
	playIndex = 0;
	recordIndex = 0;
	loopLength = 0;
	loopingActive = false;
	recordingActive = false;
}

void clearBuffer(void) {
	#ifdef DEBUG
	hw.StartLog(true);
	hw.PrintLine("Clearing buffer...");
	#endif
	for ( int i = 0; i < BUFFER_SIZE; i++ ) {
		sBuffer[L][i] = 0.f;
		sBuffer[R][i] = 0.f;
	}
}

float readFromBuffer(int ch) {

	int32_t playSpeedInt   = static_cast<int32_t>(playSpeed);
	float   playSpeedFraction = playSpeed - static_cast<float>(playSpeedInt);
	
	float index = static_cast<float>((playIndex >> 4) + (playIndex & 0xf) / 16.0f);

	int32_t     t     = static_cast<int>(index + playSpeedInt + BUFFER_SIZE);
	const float xm1   = sBuffer[ch][(t - 1) % BUFFER_SIZE];
	const float x0    = sBuffer[ch][(t) % BUFFER_SIZE];
	const float x1    = sBuffer[ch][(t + 1) % BUFFER_SIZE];
	const float x2    = sBuffer[ch][(t + 2) % BUFFER_SIZE];
	const float c     = (x1 - xm1) * 0.5f;
	const float v     = x0 - x1;
	const float w     = c + v;
	const float a     = w + v + (x2 - x0) * 0.5f;
	const float b_neg = w + a;
	const float f     = playSpeedFraction;

	return (((a * f) - b_neg) * f + c) * f + x0;
}

void incrementPlayHead() {
	playIndex += static_cast<int>(playSpeed * 16);

	size_t max = loopLength << 4;
	if ( playIndex > max ) {
		playIndex -= max;
		reachedEnd = true;
	}
	playIndex %= (loopLength << 4);
}

void startRecording() {
	recordingActive = true;
	// Start recording at the last integer playIndex we crossed
	recordIndex = playIndex >> 4;
	if ( playSpeed < 0 ) {
		// If we're playing in reverse, we need to round the other way
		recordIndex += 1;
	}
}

void stopRecording() {
	recordingActive = false;
	
	if ( ! loopingActive ) {
		loopingActive = true;
	}
}


void flashLed() {
	hw.SetLed(true);
	System::Delay(200);
	hw.SetLed(false);
	System::Delay(200);
	hw.SetLed(true);
	System::Delay(200);
	hw.SetLed(false);
	System::Delay(200);
	hw.SetLed(true);
	System::Delay(200);
	hw.SetLed(false);
	System::Delay(200);
}