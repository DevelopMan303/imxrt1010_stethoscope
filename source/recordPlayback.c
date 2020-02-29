/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <main.h>
#include "AudioFilterStream.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern sai_edma_handle_t txHandle;
extern sai_edma_handle_t rxHandle;
extern uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM];
extern volatile bool g_istxFinished;
extern volatile bool g_isrxFinished;
extern volatile uint32_t g_beginCount_OrBucketNbrMax;
extern volatile uint32_t g_sendCount;
extern volatile uint32_t g_receiveCount;
extern volatile uint32_t g_fullBlock;
extern volatile uint32_t g_emptyBlock;

extern volatile  int g_numberOfButtonPress;

/*******************************************************************************
 * Code
 ******************************************************************************/
void RecordPlayback(I2S_Type *base, uint32_t time_s)
{
    sai_transfer_t xfer    = {0};
    uint32_t playbackCount = 0;
    uint32_t recordCount = 0;

    uint32_t txindex = 0;
    uint32_t rxindex = 0;

//SimpleTestSuit();

    /* First clear the buffer */
    memset(audioBuff, 0, BUFFER_SIZE * BUFFER_NUM);
    g_istxFinished = false;
    g_isrxFinished = false;
    g_sendCount    = 0;
    g_receiveCount = 0;

    /* Reset SAI internal logic */
    SAI_TxSoftwareReset(base, kSAI_ResetTypeSoftware);
    SAI_RxSoftwareReset(base, kSAI_ResetTypeSoftware);

    /* Compute the begin count */
    g_beginCount_OrBucketNbrMax = time_s * DEMO_AUDIO_SAMPLE_RATE * 4u / BUFFER_SIZE;

    xfer.dataSize = BUFFER_SIZE;
    /* Wait for playback finished */
    while (    (recordCount < g_beginCount_OrBucketNbrMax)
    		|| (playbackCount < g_beginCount_OrBucketNbrMax) )
    {
    	/* rx receive part */
        if ((g_emptyBlock > 0) && (recordCount < g_beginCount_OrBucketNbrMax))
        {
            xfer.data = audioBuff + rxindex * BUFFER_SIZE;
            if (SAI_TransferReceiveEDMA(base, &rxHandle, &xfer) == kStatus_Success)
            {
                rxindex = (rxindex + 1U) % BUFFER_NUM;
                g_emptyBlock--;
                recordCount++;
            }
        }
        /* tx transmitt part */
        if ((g_fullBlock > 0) && (playbackCount < g_beginCount_OrBucketNbrMax))
        {
            xfer.data = audioBuff + txindex * BUFFER_SIZE;

            /* Data in data out. For Later modification */
			for (int i = 0; i < xfer.dataSize; i=i+4)
			{
				int mode = GetNumberOfButtonPresses() % NUMBER_OF_MODES;
				static int lastMode=0;
				if (lastMode != mode){
					lastMode = mode;

					//PRINTF("New mode: %d; \r\n", mode);
					if (mode==0){
						PRINTF("Mode: %d; No filtering\r\n", mode);
					}
					else if (mode==1){
						PRINTF("Mode: %d; Filtering 50-300Hz\r\n", mode);
					}
					else if (mode==2){
						PRINTF("Mode: %d; Filtering 50-300Hz - Light non linear processing \r\n", mode);
					}
					else if (mode==3){
						PRINTF("Mode: %d; Filtering 50-300Hz - Light non linear processing\r\n", mode);
					}
				}

				double value = GetMonoValueFromStereoAudioStream(&xfer.data[i]);

				if (mode!=0){
					/* No filtering if audio mode is 0 */
					value = BandPass_50Hz_300Hz(value);
				}

				int16_t valueI16 = AudioFilterAmplificateLimitINT16(value, mode);

				PutMonoValue2StereoAudioStream(valueI16, &xfer.data[i]);
			}

            if (SAI_TransferSendEDMA(base, &txHandle, &xfer) == kStatus_Success)
            {
            	/* go to next block, after last go to first.... */
                txindex = (txindex + 1U) % BUFFER_NUM;
                g_fullBlock--;
                playbackCount++;
            }
        }
    }

    /* Wait for record and playback finished */
    while ((g_istxFinished != true) || (g_isrxFinished != true))
    {
    }
}
