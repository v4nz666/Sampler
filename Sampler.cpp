// #define DEBUG


#include "Sampler.h"

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

#define L 0
#define R 1
#define A0 15



#define SAMPLE_RATE 48000

// sampler buffer
#define BUFFER_SIZE (SAMPLE_RATE * 60) // 60 secs; 48k * 2 * 4 = 384k/s 

float DSY_SDRAM_BSS sBufferR[BUFFER_SIZE];
float DSY_SDRAM_BSS sBufferL[BUFFER_SIZE];

// index into buffer, in 28.4 fixed point notation
// 	integer portion = playIndex >> 4;
// 	decimal portion = (playIndex & 0xf) / 16.0f;
uint32_t playIndex = 0;
float playSpeed = 0.5;



uint32_t recordIndex = 0;
uint32_t recordLength = BUFFER_SIZE;

bool recordingActive = true;




daisysp::Oscillator osc;

bool ledState = true;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		
		if ( recordingActive ) {
			
			sBufferL[recordIndex] = in[0][i];
			sBufferR[recordIndex] = in[1][i];
			recordIndex++;
			if ( recordIndex >= recordLength ) {
				recordingActive = false;
				recordIndex = 0;
				
				#ifdef DEBUG
				hw.PrintLine("Recording stopped.");
				#endif
			}
			out[L][i] = 0.f;
			out[R][i] = 0.f;
		} else {
			float sample = readFromBuffer();
			out[L][i] = sample;
			out[R][i] = sample;
		}

		
	}
}

int main(void)
{
	setup();
	
	hw.StartAudio(AudioCallback);
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
		hw.SetLed(recordingActive);
	}
}

void setup(void) {
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	
	#ifdef DEBUG
	hw.StartLog(true);
	hw.PrintLine("Log initialized");
	#endif

	
	AdcChannelConfig adcConfig;
	adcConfig.InitSingle(hw.GetPin(A0));
    hw.adc.Init(&adcConfig, 1);
    hw.adc.Start();





	// osc.Init(hw.AudioSampleRate());
	// osc.SetWaveform(osc.WAVE_POLYBLEP_SQUARE);	
	// osc.SetFreq(440);
	
	// for ( int i = 0; i < BUFFER_SIZE; i++ ) {
	// 	float smp = osc.Process();
	// 	sBufferR[i] = smp;
	// 	sBufferL[i] = smp;
	// }
}


float readFromBuffer(void) {

	int32_t playSpeedInt   = static_cast<int32_t>(playSpeed);
	float   playSpeedFraction = playSpeed - static_cast<float>(playSpeedInt);
	
	float index = static_cast<float>((playIndex >> 4) + (playIndex & 0xf) / 16.0f);

	int32_t     t     = static_cast<int>(index + playSpeedInt + BUFFER_SIZE);
	const float xm1   = sBufferL[(t - 1) % BUFFER_SIZE];
	const float x0    = sBufferL[(t) % BUFFER_SIZE];
	const float x1    = sBufferL[(t + 1) % BUFFER_SIZE];
	const float x2    = sBufferL[(t + 2) % BUFFER_SIZE];
	const float c     = (x1 - xm1) * 0.5f;
	const float v     = x0 - x1;
	const float w     = c + v;
	const float a     = w + v + (x2 - x0) * 0.5f;
	const float b_neg = w + a;
	const float f     = playSpeedFraction;

	playIndex += static_cast<int>(playSpeed * 16);
	playIndex %= BUFFER_SIZE * 16;

	return (((a * f) - b_neg) * f + c) * f + x0;
}
