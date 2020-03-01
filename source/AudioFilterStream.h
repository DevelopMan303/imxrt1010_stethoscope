/*
 * AudioFilterStream.h
 *
 *      Author: cons
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef AUDIOFILTERSTREAM_H_
#define AUDIOFILTERSTREAM_H_

#include <stdint.h>
#include <stdbool.h>

int16_t GetMonoValueFromStereoAudioStream(const uint8_t * data);

bool Test_GetMonoValueFromStereoAudioStream();

void PutMonoValue2StereoAudioStream(const int16_t value, uint8_t * data);

bool Test_PutMonoValue2StereoAudioStream();

void SimpleTestSuit();

double filterloop(double in);

double BandPass_100Hz_800Hz(double invar);

double LowPass_5kHz(double invar);

double BandPass_50Hz_300Hz(double invar);

extern const int NUMBER_OF_MODES;

int16_t AudioFilterAmplificateLimitINT16(double d, const int mode);

#endif /* AUDIOFILTERSTREAM_H_ */


