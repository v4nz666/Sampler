// #define DEBUG


#include "Sampler.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

DaisySeed hw;

#define L 0
#define R 1
#define IN_SPEED_1 D15

#define LED_SPEED_1 D16


#define SAMPLE_RATE 48000
#define BUFFER_LENGTH 16


// sampler buffer
#define BUFFER_SIZE (SAMPLE_RATE * BUFFER_LENGTH)

float DSY_SDRAM_BSS sBuffer[2][BUFFER_SIZE];


// index into buffer, in 28.4 fixed point notation
// 	integer portion = playIndex >> 4;
// 	decimal portion = (playIndex & 0xf) / 16.0f;
uint32_t playIndex = 0;
float playSpeed = 1.0f;

uint32_t recordIndex = 0;
uint32_t loopLength = 0;

bool loopingActive  = false;
bool recordingActive = false;

float loopDecay = 1.0f;


GPIO led_speed_1;
	


bool reachedEnd = false;
bool ledState = true;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	float recL = 0.0f,
		  recR = 0.0f,
		  loopL = 0.0f,
		  loopR = 0.0f;
	
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
		
		out[L][i] = recL + loopL;
		out[R][i] = recR + loopR;
	}
}

int main(void)
{
	setup();
	
	hw.StartAudio(AudioCallback);
	startRecording();
	System::Delay(4000);
	stopRecording();
	while(1) {
		float pot = hw.adc.GetFloat(0);
		//playSpeed = pot * 8 - 4;
		playSpeed = pot * 4 - 2;
		bool ledState = false;

		if ( playSpeed <= -3.95 ) { // Not in use
			playSpeed = -4.f;
			ledState = true;
		} else if ( playSpeed >= -2.05 && playSpeed <= -1.95 ) {
			playSpeed = -2.f;
			ledState = true;
		} else if ( playSpeed >= -1.05 && playSpeed <= -0.95 ) {
			playSpeed = -1.f;
			ledState = true;
		} else if ( playSpeed >= -0.55 && playSpeed <= -0.45 ) {
			playSpeed = -0.5f;
			ledState = true;
		} else if ( playSpeed >= -0.275 && playSpeed <= -0.225 ) {
			playSpeed = -0.25;
			ledState = true;
		} else if ( playSpeed >= -0.05 && playSpeed <= 0.05 ) {
			playSpeed = 0;
			ledState = true;
		} else if ( playSpeed >= 0.225 && playSpeed <= 0.275 ) {
			playSpeed = 0.25f;
			ledState = true;
		} else if ( playSpeed >= 0.45 && playSpeed <= 0.55 ) {
			playSpeed = 0.5f;
			ledState = true;
		} else if ( playSpeed >= 0.95 && playSpeed <= 1.05 ) {
			playSpeed = 1.f;
			ledState = true;
		} else if ( playSpeed >= 1.95 && playSpeed <= 2.05 ) {
			playSpeed = 2.f;
			ledState = true;
		} else if ( playSpeed >= 3.95) { // Not in use
			playSpeed = 4.f;
			ledState = true;
		}

		led_speed_1.Write(ledState);


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
	adcConfig.InitSingle(IN_SPEED_1);
	hw.adc.Init(&adcConfig, 1);
    hw.adc.Start();
	
	led_speed_1.Init(LED_SPEED_1, GPIO::Mode::OUTPUT);


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

	int32_t     t     = static_cast<int>(index + playSpeedInt + loopLength);
	const float xm1   = sBuffer[ch][(t - 1) % loopLength];
	const float x0    = sBuffer[ch][(t) % loopLength];
	const float x1    = sBuffer[ch][(t + 1) % loopLength];
	const float x2    = sBuffer[ch][(t + 2) % loopLength];
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
	//playIndex %= (loopLength << 4);
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