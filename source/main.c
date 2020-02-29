/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <main.h>
#include "fsl_wm8960.h"
#include "pin_mux.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_codec_common.h"
#include "fsl_codec_adapter.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI SAI1
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_BITWIDTH (kSAI_WordWidth16bits)
#define DEMO_SAI_IRQ SAI1_IRQn
#define SAI_UserIRQHandler SAI1_IRQHandler

/* IRQ */
#define DEMO_SAI_TX_IRQ SAI1_IRQn
#define DEMO_SAI_RX_IRQ SAI1_IRQn

#define EXAMPLE_SW_GPIO BOARD_USER_BUTTON_GPIO

/* DMA */
#define EXAMPLE_DMA DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define EXAMPLE_TX_CHANNEL (0U)
#define EXAMPLE_RX_CHANNEL (1U)
#define EXAMPLE_SAI_TX_SOURCE kDmaRequestMuxSai1Tx
#define EXAMPLE_SAI_RX_SOURCE kDmaRequestMuxSai1Rx

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);
static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);

void SysTick_Handler(void);

void GPIO2_Combined_0_15_IRQHandler(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
wm8960_config_t wm8960Config = {
    .i2cConfig = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .route     = kWM8960_RoutePlaybackandRecord,
    .rightInputSource = kWM8960_InputDifferentialMicInput2,
    .playSource       = kWM8960_PlaySourceDAC,
    .slaveAddress     = WM8960_I2C_ADDR,
    .bus              = kWM8960_BusI2S,
    .format = {.mclk_HZ = 6144000U, .sampleRate = kWM8960_AudioSampleRate16KHz, .bitWidth = kWM8960_AudioBitWidth16bit},
    .master_slave = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8960, .codecDevConfig = &wm8960Config};
/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaTxHandle                                = {0};
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle) = {0};
edma_handle_t dmaRxHandle                                = {0};
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
extern codec_config_t boardCodecConfig;
volatile bool g_istxFinished     = false;
volatile bool g_isrxFinished     = false;
volatile uint32_t g_beginCount_OrBucketNbrMax   = 0;
volatile uint32_t g_sendCount    = 0;
volatile uint32_t g_receiveCount = 0;
volatile bool sdcard           = false;
volatile uint32_t g_fullBlock    = 0;
volatile uint32_t g_emptyBlock   = BUFFER_NUM;

sai_transceiver_t config;
codec_handle_t codecHandle;

volatile int g_numberOfButtonPress = 2;

const int DEBOUNCE = 300;

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

/* Callback for users while transfer finish or error occurs */
static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    g_receiveCount++;
    g_fullBlock++;

    if (g_receiveCount == g_beginCount_OrBucketNbrMax)
    {
        g_isrxFinished = true;
        SAI_TransferTerminateReceiveEDMA(base, handle);
        g_receiveCount = 0;
    }
}

/* Callback for users while transfer finish or error occurs */
static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    g_sendCount++;
    g_emptyBlock++;

    if (g_sendCount == g_beginCount_OrBucketNbrMax)
    {
        g_istxFinished = true;
        SAI_TransferTerminateSendEDMA(base, handle);
        g_sendCount = 0;
    }
}

/**
 *
 * @brief	Systic timer. Count upwards in ms
 *
 */
volatile static unsigned int g_timer_systic = 0;
void SysTick_Handler(void)
{
	g_timer_systic++;
}
/**
 * @brief 	Return actual timer. A copy is returned, so no worry regarding concurrency
 *
 * @return actual time in ms
 */
unsigned int GetTimerSystick(){
	return g_timer_systic;
}

/**
 *  @brief	IRQ for button pressed. Handle debouncing becaus IRQ is so fast
 *  override default handler
 */
void GPIO2_Combined_0_15_IRQHandler(void)
{
	/* handle EXAMPLE_SW_GPIO */
	if (GPIO_GetPinsInterruptFlags(EXAMPLE_SW_GPIO) & ( 1U << BOARD_USER_BUTTON_GPIO_PIN) )
	{
		/* clear the interrupt status */
		GPIO_PortClearInterruptFlags(EXAMPLE_SW_GPIO, 1U << BOARD_USER_BUTTON_GPIO_PIN);

		static unsigned int timeLastValidButtonPress = 0;
		unsigned int actualTime = GetTimerSystick();
		/* Do de-bounce -> Can be moved out of IRQ contect.   */
		if ( actualTime - timeLastValidButtonPress > DEBOUNCE){
			g_numberOfButtonPress++;
			timeLastValidButtonPress = actualTime;
		}
	}

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}


/**
 * @brief Number of button pressed (debounced)
 */
int GetNumberOfButtonPresses(void){
	return g_numberOfButtonPress;
}

/*!
 * @brief Main function
 */
int main(void)
{
    edma_config_t dmaConfig = {0};

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    CLOCK_InitAudioPll(&audioPllConfig);
    BOARD_InitDebugConsole();

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);

    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);

    PRINTF("Stethoscope demo \n\r");

    /* Create EDMA handle */

    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxHandle, EXAMPLE_DMA, EXAMPLE_TX_CHANNEL);
    EDMA_CreateHandle(&dmaRxHandle, EXAMPLE_DMA, EXAMPLE_RX_CHANNEL);

    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL, (uint8_t)EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL, (uint8_t)EXAMPLE_SAI_RX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL);

    /* SAI init */
    SAI_Init(DEMO_SAI);

    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, txCallback, NULL, &dmaTxHandle);
    SAI_TransferRxCreateHandleEDMA(DEMO_SAI, &rxHandle, rxCallback, NULL, &dmaRxHandle);

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, kSAI_Channel0Mask);
    SAI_TransferTxSetConfigEDMA(DEMO_SAI, &txHandle, &config);
    config.syncMode = kSAI_ModeSync;
    SAI_TransferRxSetConfigEDMA(DEMO_SAI, &rxHandle, &config);

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);
    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);

/* master clock configurations */
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
#if defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
    mclkConfig.mclkHz          = DEMO_AUDIO_MASTER_CLOCK;
    mclkConfig.mclkSourceClkHz = DEMO_SAI_CLK_FREQ;
#endif
    SAI_SetMasterClockConfig(DEMO_SAI, &mclkConfig);
#endif

    /* Use default setting to init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);

    /* Enable interrupt to handle FIFO error */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    EnableIRQ(DEMO_SAI_TX_IRQ);
    EnableIRQ(DEMO_SAI_RX_IRQ);

    /** Config IRQ for button press */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
        kGPIO_IntRisingEdge,
    };

    /* Init input switch GPIO. */
    EnableIRQ(GPIO2_Combined_0_15_IRQn);

    GPIO_PinInit(GPIO2, BOARD_USER_BUTTON_GPIO_PIN, &sw_config);

    /* Enable GPIO pin interrupt */
    GPIO_PortEnableInterrupts(GPIO2, 1U << BOARD_USER_BUTTON_GPIO_PIN);


    /* Systick  */
    /* Set systick reload value to generate 1ms interrupt */
    SysTick_Config(SystemCoreClock / 1000U);

	while(true){
		RecordPlayback(DEMO_SAI, 60);
	}
}

void SAI_UserTxIRQHandler(void)
{
    /* Clear the FEF flag */
    SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SAI_UserRxIRQHandler(void)
{
    SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SAI_UserIRQHandler(void)
{
    if (SAI_TxGetStatusFlag(DEMO_SAI) & kSAI_FIFOErrorFlag)
    {
        SAI_UserTxIRQHandler();
    }

    if (SAI_RxGetStatusFlag(DEMO_SAI) & kSAI_FIFOErrorFlag)
    {
        SAI_UserRxIRQHandler();
    }
    __DSB();
}
