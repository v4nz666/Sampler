//#define DEBUG


#include "Sampler.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

DaisySeed hw;

#define L 0
#define R 1
#define IN_SPEED_1 A10
#define IN_REC_1 D29

#define LED_SPEED_1 D30

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

bool inputMonitoring = true;

bool loopingActive  = false;
bool recordingActive = false;

float loopDecay = 0.9f;

Switch in_rec_1;
GPIO led_speed_1;
	


bool reachedEnd = false;
bool ledState = true;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for ( size_t i = 0; i < size; i++ ) {
		float inL = 0.0f,
		  inR = 0.0f,
		  loopL = 0.0f,
		  loopR = 0.0f,
		  outL = 0.0f,
		  outR = 0.0f;		
		
		if ( loopingActive ) {	
			loopL = readFromBuffer(L);
			loopR = readFromBuffer(R);
			incrementPlayHead();
		}
		
		inL = in[0][i];
		inR = in[1][i];

		if ( recordingActive ) {
			
			// Add our incoming sample to what's already in the buffer at the record head
			sBuffer[L][recordIndex] = (sBuffer[L][recordIndex] * loopDecay) + inL;
			sBuffer[R][recordIndex] = (sBuffer[R][recordIndex] * loopDecay) + inR;

			recordIndex++;
			
			// If we're recording the first loop into the buffer, keep track of the length as we record
			if ( ! loopingActive ) {
				// And if we hit the end of the buffer, start looping, and cap our loopLength
				if ( ++loopLength >= BUFFER_SIZE ) {
					loopLength = BUFFER_SIZE;
					stopRecording();
					reachedEnd = true;
				}
			} else {
				recordIndex %= loopLength;
			}
		}

		outL = loopL;
		outR = loopR;

		if ( inputMonitoring ) {
			outL += inL;
			outR += inR;
		}

		out[L][i] = outL;
		out[R][i] = outR;
	}
}

int main(void)
{
	setup();
	
	hw.StartAudio(AudioCallback);

	while(1) {
		
		led_speed_1.Write(determinePlaySpeed());
		
		in_rec_1.Debounce();

		if ( in_rec_1.RisingEdge() ) {
			flashLed(1);
			recordingActive ? stopRecording() : startRecording();
		}

		if ( reachedEnd ) {
			flashLed(2);
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

	in_rec_1.Init(IN_REC_1);


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
	#ifdef DEBUG
	if ( (playIndex >> 4) % 4800 == 0 ) {
		hw.PrintLine("Reading buffer at index [%f]. Play speed [%f]", index, playSpeed);
	}
	#endif

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
	} else if ( playIndex < 0 ) {
		playIndex += max;
		reachedEnd = true;
	}
}

void startRecording() {
	#ifdef DEBUG
		hw.PrintLine("Starting recording...");
	#endif
	recordingActive = true;
	// Start recording at the last integer playIndex we crossed
	recordIndex = playIndex >> 4;
	if ( playSpeed < 0 ) {
		// If we're playing in reverse, we need to round the other way
		recordIndex += 1;
	}
}

void stopRecording() {
	#ifdef DEBUG
		hw.PrintLine("Stopping recording...");
	#endif
	recordingActive = false;
	
	if ( ! loopingActive ) {
		loopingActive = true;
	}
}

bool determinePlaySpeed() {
	float pot = hw.adc.GetFloat(0) - 0.5;
		
		bool negative = false;
		
		if ( pot < 0 ) {
			negative = true;
			pot = abs(pot);
		}
		pot *= 8;
		playSpeed = (pow(2, pot) - 1) / 7.5;

		if ( negative ) {
			playSpeed *= -1;
		}

		bool ledState = false;

		if ( playSpeed <= -3.95 ) { // Not in use
			playSpeed = -4.f;
			ledState = true;
		} else if ( playSpeed >= -2.1 && playSpeed <= -1.9 ) {
			playSpeed = -2.f;
			ledState = true;
		} else if ( playSpeed >= -1.05 && playSpeed <= -0.95 ) {
			playSpeed = -1.f;
			ledState = true;
		} else if ( playSpeed >= -0.525 && playSpeed <= -0.475 ) {
			playSpeed = -0.5f;
			ledState = true;
		} else if ( playSpeed >= -0.2625 && playSpeed <= -0.2375 ) {
			playSpeed = -0.25;
			ledState = true;
		} else if ( playSpeed >= -0.025 && playSpeed <= 0.025 ) {
			playSpeed = 0;
			ledState = true;
		} else if ( playSpeed >= 0.2735 && playSpeed <= 0.2625 ) {
			playSpeed = 0.25f;
			ledState = true;
		} else if ( playSpeed >= 0.475 && playSpeed <= 0.525 ) {
			playSpeed = 0.5f;
			ledState = true;
		} else if ( playSpeed >= 0.95 && playSpeed <= 1.05 ) {
			playSpeed = 1.f;
			ledState = true;
		} else if ( playSpeed >= 1.9 && playSpeed <= 2.0 ) {
			playSpeed = 2.f;
			ledState = true;
		} else if ( playSpeed >= 3.95) { // Not in use
			playSpeed = 4.f;
			ledState = true;
		}
	return ledState;
}


void flashLed(size_t repeats) {
	for ( size_t x = 0; x < repeats; x++) {
		hw.SetLed(true);
		System::Delay(200);
		hw.SetLed(false);
		System::Delay(200);
	}
}