/*
 *   Copyright 2026 NXP
 *
 *   NXP Proprietary. This software is owned or controlled by NXP and may only be
 *   used strictly in accordance with the applicable license terms.  By expressly
 *   accepting such terms or by downloading, installing, activating and/or otherwise
 *   using the software, you are agreeing that you have read, and that you agree to
 *   comply with and are bound by, such license terms.  If you do not agree to be
 *   bound by the applicable license terms, then you may not retain, install,
 *   activate or otherwise use the software.
 *
 *   This file contains sample code only. It is not part of the production code deliverables.
 */

/**
 *   @file    main.c
 *
 *   @brief   ADC sampling with DMA transfer plus UART over DMA on FRDM-A-S32K358.
 *
 *   @details
 *   This example combines two DMA-driven data paths using the AUTOSAR MCAL drivers:
 *
 *   1. ADC + DMA
 *      A software triggered one-shot ADC group samples the on-board potentiometer
 *      (ADC2_S19, PTA17). The conversion result is moved by the eDMA engine directly
 *      into the application buffer ResultBufferDma, without any CPU copy and without
 *      using the BCTU, a software-trigger interrupt group, or the SDADC.
 *
 *   2. UART + DMA
 *      Asynchronous LPUART transmit and receive where the byte movement between the
 *      application buffers and the LPUART data register is performed by the eDMA
 *      engine and not by the CPU.
 *
 *   DATA FLOW (ADC over DMA)
 *   ------------------------
 *   - Adc_StartGroupConversion arms the ADC group; on conversion complete the ADC
 *     hardware raises a DMA request that copies the raw sample into ResultBufferDma.
 *   - The group notification (Notification_2) signals that a new sample is available.
 *   - Adc_ReadGroup returns the 14-bit right aligned value; the raw DMA sample is
 *     shifted by one bit so both representations match.
 *
 *   DATA FLOW (UART over DMA)
 *   -------------------------
 *   - Uart_AsyncSend / Uart_AsyncReceive only program an eDMA channel and start it.
 *   - The LPUART TX/RX requests are routed to the DMA channels through the DMAMUX.
 *   - On transfer completion a DMA channel interrupt (DMATCD16 for TX, DMATCD17 for
 *     RX) calls back into the UART driver, which invokes UART_event_cbk().
 *
 *   INITIALIZATION ORDER (critical for the DMA paths)
 *   -------------------------------------------------
 *   Mcl -> Port -> Rm (DMAMUX) -> Platform -> DMA channel IRQs -> Uart_Init / Adc_Init.
 *
 *   @addtogroup main_module main module documentation
 *   @{
 */

#ifdef __cplusplus
extern "C"{
#endif

/*==================================================================================================
 *                                        INCLUDE FILES
 ==================================================================================================*/
#include "Mcal.h"
#include "OsIf.h"
#include "Mcu.h"
#include "Mcl.h"
#include "Port.h"
#include "Dio.h"
#include "CDD_Uart.h"
#include "CDD_Rm.h"
#include "Platform.h"
#include "Adc.h"
#include <string.h>
#include <stdio.h>

/*==================================================================================================
 *                                      LOCAL MACROS
 ==================================================================================================*/
/*
 * Logical UART channel ID as defined in the tool (Uart_ChannelConfig_0). It is
 * internally mapped to the physical LPUART hardware. Do NOT use the physical
 * instance number here.
 */
#define UART_LPUART_INTERNAL_CHANNEL   0U

/* Number of bytes moved by DMA for each raw UART pattern transfer. */
#define UART_TRANSFER_SIZE             5U

/* Welcome banner sent once at startup. */
#define UART_WELCOME_MSG               "Hello from FRDM_A_S32K358 ADC + UART DMA\r\n"

/* Period between two periodic messages, in milliseconds. */
#define UART_PERIODIC_DELAY_MS         1000U

/*==================================================================================================
 *                                      ADC + DMA MACROS
 * Only the software triggered one-shot conversion whose result is moved by the
 * eDMA engine is used here. No BCTU, no interrupt-only group, no SDADC.
 ==================================================================================================*/
/* Number of result slots in the ADC buffers. */
#define NUM_RESULTS         (3U)
/* Init pattern written into the DMA result buffer before each conversion. */
#define RESULT_BUFF_VAL     (0xAAAAU)
/* Init pattern written into the read-back buffer before each conversion. */
#define ADC_RESULT_BUFF_VAL (0xBBBBU)
/* Maximum 14-bit right aligned ADC value. */
#define ADC_VREFH           (16383U)


/* Interrupt sources used by the ADC and its DMA channel (K358 mapping, no BCTU). */
#define ADC_DMA_IRQ         DMATCD1_IRQn
#define ADC_DMA_IRQ_HANDLER Dma0_Ch1_IRQHandler

/* ADC and ADC-DMA channel interrupt service routines (provided by the driver). */
extern ISR(Adc_Sar_0_Isr);
extern ISR(ADC_DMA_IRQ_HANDLER);

/*==================================================================================================
 *                                      GLOBAL VARIABLES
 ==================================================================================================*/
volatile int exit_code = 0;

/* Set from the UART callback when the TX register becomes empty. */
boolean dmaDone = FALSE;

/*
 * UART RX/TX buffers, placed in a non-cacheable, 32-byte aligned section because the
 * DMA accesses them directly (avoids cache coherency issues and satisfies the DMA
 * descriptor alignment requirement).
 */
#pragma GCC section bss ".mcal_bss_no_cacheable"
__attribute__(( aligned(32) )) uint8_t rx_data[52];
/* DMA-capable buffer for string messages (welcome / periodic). */
__attribute__(( aligned(32) )) uint8_t tx_msg[64];
#pragma GCC section bss

/* Counts how many full UART RX transfers have completed. */
uint32_t rx_counter = 0U;

/*
 * TX busy flag. Set to 1 before starting a DMA transmission, cleared to 0 from the
 * UART callback on LPUART_UART_IP_EVENT_END_TRANSFER.
 */
volatile uint8 tx_busy = 0U;

/*==================================================================================================
 *                                      ADC + DMA GLOBALS
 ==================================================================================================*/
/* Read-back buffer filled by Adc_ReadGroup (14-bit right aligned result). */
Adc_ValueGroupType AdcReadGroupResult[NUM_RESULTS] =
{
    ADC_RESULT_BUFF_VAL, ADC_RESULT_BUFF_VAL, ADC_RESULT_BUFF_VAL
};

/*
 * DMA target buffer: the eDMA engine writes the raw ADC result directly here.
 * Placed in a non-cacheable, 32-byte aligned section (same reasoning as the UART
 * DMA buffers).
 */
#define ADC_START_SEC_VAR_INIT_UNSPECIFIED_NO_CACHEABLE
#include "Adc_MemMap.h"

__attribute__(( aligned(32) )) Adc_ValueGroupType ResultBufferDma[NUM_RESULTS] =
{
    RESULT_BUFF_VAL, RESULT_BUFF_VAL, RESULT_BUFF_VAL
};

#define ADC_STOP_SEC_VAR_INIT_UNSPECIFIED_NO_CACHEABLE
#include "Adc_MemMap.h"

/* Group notification counters for the DMA group. */
volatile uint8 VarNotification_1 = 0U;
volatile uint8 VarNotification_2 = 0U;

/* Latest potentiometer sample read through the DMA group (ADC2_S19, PTA17). */
volatile uint16 PotentiometerValue = 0U;

/*==================================================================================================
 *                                      ADC NOTIFICATIONS
 ==================================================================================================*/
void Notification_1(void)
{
    VarNotification_1++;
}

void Notification_2(void)
{
    VarNotification_2++;
}

/*==================================================================================================
 *                                      HELPER: DELAY
 ==================================================================================================*/
/**
 * @brief   Blocking delay based on the OsIf system counter.
 *
 * @param[in] delayMs  Delay duration, converted with OsIf_MicrosToTicks.
 */
static void Delay(uint32 delayMs)
{
    uint32 cur = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    uint32 elapsed = 0U;
    uint32 timeout = OsIf_MicrosToTicks(delayMs, OSIF_COUNTER_SYSTEM);

    while (elapsed < timeout)
    {
        elapsed += OsIf_GetElapsed(&cur, OSIF_COUNTER_SYSTEM);
    }
}

/*==================================================================================================
 *                                      UART CALLBACK
 ==================================================================================================*/
/**
 * @brief   UART event callback, invoked by the driver from the DMA completion path.
 *
 * @param[in] HwInstance  Physical LPUART instance that generated the event.
 * @param[in] Event       Event type reported by the driver.
 * @param[in] UserData    Optional user pointer (not used here).
 */
void UARTCallback(const uint8 HwInstance, const Lpuart_Uart_Ip_EventType Event, const void *UserData)
{
    (void)HwInstance;
    (void)UserData;

    if (Event == LPUART_UART_IP_EVENT_END_TRANSFER)
    {
        /* A full TX (or RX) DMA transfer has finished: release the TX busy flag. */
        tx_busy = 0U;
    }
    else if (Event == LPUART_UART_IP_EVENT_TX_EMPTY)
    {
        /* TX register became empty during the transfer. */
        dmaDone = TRUE;
    }
    else if (Event == LPUART_UART_IP_EVENT_RX_FULL)
    {
        /* Requested RX bytes received via DMA: re-arm reception for the next block. */
        rx_counter++;
        (void)Uart_SetBuffer(UART_LPUART_INTERNAL_CHANNEL, rx_data, UART_TRANSFER_SIZE, UART_RECEIVE);
    }
    else if (Event == LPUART_UART_IP_EVENT_ERROR)
    {
        /* Reception error: restart an asynchronous receive. */
        (void)Uart_AsyncReceive(UART_LPUART_INTERNAL_CHANNEL, rx_data, UART_TRANSFER_SIZE);
    }
    else
    {
        /* No action for other events. */
    }
}

/*==================================================================================================
 *                                      HELPER: SEND STRING
 ==================================================================================================*/
/**
 * @brief   Send a null-terminated string over UART using DMA.
 *
 * @details The string is copied into the DMA-capable tx_msg buffer and the transfer
 *          is started with Uart_AsyncSend. The byte movement is performed by the
 *          eDMA engine, not by the CPU.
 *
 * @param[in] str  Null-terminated ASCII string to transmit.
 *
 * @return  E_OK if the transfer was started, E_NOT_OK otherwise.
 */
static Std_ReturnType Uart_SendString(const char *str)
{
    Std_ReturnType status;
    uint32 length = (uint32)strlen(str);

    if ((length == 0U) || (length > sizeof(tx_msg)))
    {
        return (Std_ReturnType)E_NOT_OK;
    }

    (void)memcpy((void*)tx_msg, (const void*)str, length);

    tx_busy = 1U;
    status = Uart_AsyncSend(UART_LPUART_INTERNAL_CHANNEL, tx_msg, length);
    if (E_OK == status)
    {
        dmaDone = FALSE;
    }

    return status;
}

/*==================================================================================================
 *                                      ADC + DMA ACQUISITION
 ==================================================================================================*/
/**
 * @brief   Initialize and (optionally) calibrate the DMA-driven ADC group.
 *
 * @details Routes and enables the ADC and its DMA channel interrupts, initializes
 *          the ADC driver, calibrates the hardware unit used for the DMA group and
 *          binds ResultBufferDma to the group. Must be called once before the
 *          acquisition loop. No BCTU, no software-trigger interrupt group, no SDADC.
 */
static void Adc_Dma_Setup(void)
{
    /* Route and enable the ADC and the ADC-DMA channel interrupts. */
    Platform_InstallIrqHandler(ADC0_IRQn, Adc_Sar_0_Isr, NULL_PTR);
    Platform_InstallIrqHandler(ADC_DMA_IRQ, ADC_DMA_IRQ_HANDLER, NULL_PTR);
    Platform_SetIrq(ADC0_IRQn, TRUE);
    Platform_SetIrq(ADC_DMA_IRQ, TRUE);

    /* Initialize the ADC driver. */
#if (ADC_PRECOMPILE_SUPPORT == STD_ON)
    Adc_Init(NULL_PTR);
#else
    Adc_Init(&Adc_Config);
#endif /* (ADC_PRECOMPILE_SUPPORT == STD_ON) */

#if (ADC_CALIBRATION == STD_ON)
    {
        Adc_CalibrationStatusType CalibStatus;
        uint8 attempt;

        /* Calibrate the hardware unit used for the DMA group. Retry a few times to
           mitigate instability of the board reference source. */
        for (attempt = 0U; attempt <= 5U; attempt++)
        {
            Adc_Calibrate(AdcHwUnitDma, &CalibStatus);
            if (CalibStatus.AdcUnitSelfTestStatus == E_OK)
            {
                break;
            }
        }
    }
#endif /* (ADC_CALIBRATION == STD_ON) */

    /* Bind the DMA result buffer to the group and enable its notification. */
    Adc_SetupResultBuffer(AdcGroupSoftwareOneShotDma, ResultBufferDma);
    Adc_EnableGroupNotification(AdcGroupSoftwareOneShotDma);
}

/**
 * @brief   Arm one software triggered one-shot conversion moved by the eDMA.
 *
 * @details Only issues the ADC start command and returns immediately. The CPU does
 *          NOT wait here: the eDMA engine moves the result into ResultBufferDma in
 *          the background and Notification_2 fires on completion. This is the
 *          non-blocking counterpart of the old busy-wait implementation.
 */
static void Adc_Dma_Start(void)
{
    /* MCU -> ADC Start. */
    VarNotification_2 = 0U;
    Adc_StartGroupConversion(AdcGroupSoftwareOneShotDma);
}

/**
 * @brief   Non-blocking check for a completed DMA-driven conversion.
 *
 * @return  TRUE once the DMA-completion notification (Notification_2) has fired.
 */
static boolean Adc_Dma_ResultReady(void)
{
    return (VarNotification_2 != 0U) ? TRUE : FALSE;
}

/**
 * @brief   Read back the DMA-moved conversion result.
 *
 * @details Called only after Adc_Dma_ResultReady() returns TRUE. The DMA transfers
 *          the full 15-bit width of the CDR register, while Adc_ReadGroup returns the
 *          14-bit right aligned result. The raw DMA value is shifted by one bit so
 *          both representations match. The CPU does not copy the sample itself.
 *
 * @return  14-bit right aligned potentiometer value (0 .. ADC_VREFH).
 */
static uint16 Adc_Dma_GetValue(void)
{
    uint16 value;

    (void)Adc_ReadGroup(AdcGroupSoftwareOneShotDma, AdcReadGroupResult);
    value = (uint16)(ResultBufferDma[0U] >> 1U);

    /* Reset the result slot before the next conversion. */
    ResultBufferDma[0U]    = RESULT_BUFF_VAL;
    AdcReadGroupResult[0U] = ADC_RESULT_BUFF_VAL;

    return value;
}



/*==================================================================================================
 *                                      MAIN FUNCTION
 ==================================================================================================*/
/**
 * @brief   Application entry point.
 */
int main(void)
{
    char lineBuffer[32];
    Std_ReturnType uartStatus;
    uint16 adcValue;


    /* --- Clock / core initialization --- */
    OsIf_Init(NULL_PTR);
    Mcu_Init(&Mcu_Config);
    Mcu_InitClock(McuClockSettingConfig_0);
    while (Mcu_GetPllStatus() != MCU_PLL_LOCKED)
    {
        /* Wait until the PLL is locked. */
    }
    Mcu_DistributePllClock();
    Mcu_SetMode(McuModeSettingConf_0);

    /* --- DMA path initialization (order matters, see file header) --- */
    Mcl_Init(&Mcl_Config);              /* eDMA engine and channels               */
    Port_Init(NULL_PTR);                /* pin muxing for the LPUART pads          */
    Rm_Init(&Rm_Config);                /* DMAMUX request routing                  */
    Platform_Init(Platform_Config[0]);  /* interrupt controller configuration      */

    /*
     * Route and enable the UART DMA channel interrupts. Point the DMATCD16/17
     * vectors at the per-channel handlers and enable the IRQs. Without them the UART
     * transmit-complete event never fires and the status stays "operation ongoing".
     */
    Platform_InstallIrqHandler(DMATCD16_IRQn, Dma0_Ch16_IRQHandler, NULL_PTR);
    Platform_InstallIrqHandler(DMATCD17_IRQn, Dma0_Ch17_IRQHandler, NULL_PTR);
    Platform_SetIrq(DMATCD16_IRQn, TRUE);
    Platform_SetIrq(DMATCD17_IRQn, TRUE);

    /* UART driver (DMA path is ready at this point). */
    Uart_Init(NULL_PTR);

    /* Start the first asynchronous (DMA based) reception. */
    uartStatus = Uart_AsyncReceive(UART_LPUART_INTERNAL_CHANNEL, rx_data, UART_TRANSFER_SIZE);
    if (E_OK != uartStatus)
    {
        while (1)
        {
            __asm volatile ("nop");
        }
    }

    /* Send the welcome banner once at startup. */
    (void)Uart_SendString(UART_WELCOME_MSG);

    /* --- ADC + DMA path setup (run once before the acquisition loop) --- */
    Adc_Dma_Setup();

    /*
     * --- Main loop (ADC over DMA -> UART over DMA), non-blocking state machine ---
     *
     * The CPU never busy-waits on the DMA. Each iteration only arms a transfer and
     * then keeps polling the completion flags without blocking, so the core stays
     * free while the eDMA works in the background.
     *
     * LED convention (visualizes the CPU off-load):
     *   RED LED ON  -> CPU is idle, waiting on a background DMA transfer.
     *   RED LED OFF -> CPU is actively doing work in main (arm / format / issue TX).
     *
     * Per cycle:
     *   ST_ARM_ADC : MCU -> ADC Start                (Adc_Dma_Start)
     *   ST_WAIT_ADC: RED ON, poll Adc_Dma_ResultReady (eDMA writes ResultBufferDma)
     *   ST_READ    : CPU reads the DMA sample and puts it into the UART buffer
     *   ST_ISSUE_TX: MCU issues the transmit command (eDMA moves the UART bytes)
     *   ST_DELAY   : RED ON during the readability delay, then back to ST_ARM_ADC
     */
    enum
    {
        ST_ARM_ADC = 0,
        ST_WAIT_ADC,
        ST_READ,
        ST_ISSUE_TX,
        ST_DELAY
    } state = ST_ARM_ADC;


    /* Start with the RED LED off: the CPU is about to do active work. */
    Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);

    while (1)
    {
        switch (state)
        {
            case ST_ARM_ADC:
                /* CPU active: MCU -> ADC Start. */
                Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);
                Adc_Dma_Start();
                state = ST_WAIT_ADC;
                break;

            case ST_WAIT_ADC:
                /* CPU idle: the eDMA moves the sample into ResultBufferDma. */
                Dio_WriteChannel(DioConf_DioChannel_RED, STD_HIGH);
                if (Adc_Dma_ResultReady() == TRUE)
                {
                    state = ST_READ;
                }
                break;

            case ST_READ:
                /* CPU active: read the DMA-moved sample and put it in the UART buffer. */
                Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);
                adcValue = Adc_Dma_GetValue();
                PotentiometerValue = adcValue;
                (void)sprintf(lineBuffer, "ADC: %u\r\n", (unsigned int)adcValue);
                state = ST_ISSUE_TX;
                break;

            case ST_ISSUE_TX:
                /* CPU active: MCU issues the transmit command (bytes moved by eDMA).
                   The transfer runs in the background; the CPU does not wait for it. */
                Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);
                (void)Uart_SendString(lineBuffer);
                state = ST_DELAY;
                break;

            case ST_DELAY:

            default:
                /* CPU idle: readability delay before the next conversion. */
                Dio_WriteChannel(DioConf_DioChannel_RED, STD_HIGH);
                Delay(UART_PERIODIC_DELAY_MS);
                state = ST_ARM_ADC;
                break;
        }
    }

    return exit_code;

}


#ifdef __cplusplus
}
#endif

/** @} */
