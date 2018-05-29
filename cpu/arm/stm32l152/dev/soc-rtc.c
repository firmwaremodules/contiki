/*
 * Copyright (c) 2018, Firmware Modules Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "soc-rtc.h"

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_rtc.h"
#include "stm32l1xx_hal_rcc.h"


#include <stdio.h>

/* Define the external low speed clock as the default */
#if !defined(RTC_CLOCK_SOURCE_LSI) && !defined(RTC_CLOCK_SOURCE_LSE)
#define RTC_CLOCK_SOURCE_LSE
#endif

#if defined(RTC_CLOCK_SOURCE_LSI)
 /* ck_apre=LSIFreq/(ASYNC prediv + 1) with LSIFreq=37 kHz RC */
#define RTC_ASYNCH_PREDIV    0x0
 /* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV     0x7FFF
#elif defined(RTC_CLOCK_SOURCE_LSE)
 /* ck_apre=LSEFreq/(ASYNC prediv + 1) = 32768Hz with LSEFreq=32768Hz */
#define RTC_ASYNCH_PREDIV  0x0
 /* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV   0x7FFF
#else 
#error "configure clock for STM32L152 RTC"
#endif

#define SOC_RTC_TEST 0


/*---------------------------------------------------------------------------*/

/* Located in stm32cube_hal_init.c */
extern RTC_HandleTypeDef RtcHandle;

/*---------------------------------------------------------------------------*/


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
    printf("\nsoc-rtc: ERROR!\n");
    while (1) {}
}
/*---------------------------------------------------------------------------*/

/**
* @brief  Configure the current time and date.
* @param  None
* @retval None
*/
static void 
RTC_AlarmConfig(uint32_t channel, rtimer_clock_t time)
{
    RTC_AlarmTypeDef salarmstructure;

    /*##-3- Configure the RTC Alarm peripheral #################################*/
    if (channel == SOC_RTC_CHANNEL_CLOCK) {
        salarmstructure.Alarm = RTC_ALARM_A;
    }
    else if (channel == SOC_RTC_CHANNEL_RTIMER) {
        salarmstructure.Alarm = RTC_ALARM_B;
    }
    else {
        return;
    }

    /* match all other fields except date weekday and hours */
    salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;

    /* use all bits of the subsecond register */
    salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM; /* don't care */
    salarmstructure.AlarmTime.Hours = 0; /* don't care */
    salarmstructure.AlarmTime.Minutes = time * 60 * RTIMER_SECOND;
    salarmstructure.AlarmTime.Seconds = time * RTIMER_SECOND;
    salarmstructure.AlarmTime.SubSeconds = time;

    if (HAL_RTC_SetAlarm_IT(&RtcHandle, &salarmstructure, FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}


/*---------------------------------------------------------------------------*/
/* The RTC interrupt handler */
void
soc_rtc_isr(uint8_t channel)
{
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    if (channel == SOC_RTC_CHANNEL_RTIMER) {
        HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_B);
        rtimer_run_next();
    }

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/

/**
* @brief  Alarm callback
* @param  hrtc : RTC handle
* @retval None
*/
void 
HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    soc_rtc_isr(SOC_RTC_CHANNEL_CLOCK);
}

/**
* @brief  Alarm callback
* @param  hrtc : RTC handle
* @retval None
*/
void 
HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
    soc_rtc_isr(SOC_RTC_CHANNEL_RTIMER);
}


/**
* @brief  This function handles RTC Alarm interrupt request.
* @param  None
* @retval None
*/
void 
RTC_Alarm_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&RtcHandle);
}


/*---------------------------------------------------------------------------*/

void
soc_rtc_init(void)
{

#if 0
    if (HAL_RTC_DeInit(&RtcHandle) != HAL_OK) {
        Error_Handler();
    }

    //printf("\nRTC INIT: RTC_SYNCH_PREDIV=%d\n", RTC_SYNCH_PREDIV);

    /* Re-configure the RTC for proper prescaler.
     * the RTC is started in the
     * stm32cube-lib submodule supplied by ST as a consequence
     * of calling stm32cube_hal_init().
     * Therefore, this function soc_rtc_init() must be called
     * after stm32cube_hal_init().
     *
     * The RTC is used only to source RTIMER ticks
     * and schedule RTIMER events as requested.
     */

    /* Alarm A (channel 0) is reserved for the system clock.
     * Alarm B (channel 1) is reserved for the rtimer clock.
     */

     /*##-1- Configure the RTC peripheral #######################################*/
    RtcHandle.Instance = RTC;

    /* Configure RTC prescaler and RTC data registers */
    /* RTC configured as follow:
    - Hour Format    = Format 12
    - Asynch Prediv  = Value according to source clock
    - Synch Prediv   = Value according to source clock
    - OutPut         = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType     = Open Drain */
    RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    /* This normally calls HAL_RTC_MspInit for the */
    if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }


    RCC_OscInitTypeDef        RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

    /*##-1- Configue LSE as RTC clock soucre ###################################*/
#ifdef RTC_CLOCK_SOURCE_LSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
#elif defined (RTC_CLOCK_SOURCE_LSI)  
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
#else
#error Please select the RTC Clock source inside the stm32cube_hal_init.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/

    /*##-2- Enable RTC peripheral Clocks #######################################*/
    /* Enable RTC Clock */
    __HAL_RCC_RTC_ENABLE();

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

#endif
}

/*---------------------------------------------------------------------------*/


void soc_rtc_schedule_one_shot(uint8_t channel, rtimer_clock_t t)
{
    if ((channel != SOC_RTC_CHANNEL_CLOCK) && (channel != SOC_RTC_CHANNEL_RTIMER)) {
        return;
    }

    /* Set the channel to fire a one-shot compare event at time==ticks */
    RTC_AlarmConfig(channel, t);
}

/* Get current time in rtimer clock units
*/
rtimer_clock_t 
soc_rtc_get_current_time()
{
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;
    rtimer_clock_t time;

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructure, FORMAT_BIN);

    /* This is a global absolute time down to resolution of
     * RTIMER_SECONDS.
     */
    time =
        (RTC_SYNCH_PREDIV - stimestructure.SubSeconds) + // already in RTIMER_SECOND units
        (stimestructure.Seconds +
         stimestructure.Minutes * 60 +
         stimestructure.Hours * 60 * 60) * RTIMER_SECOND;

#if SOC_RTC_TEST

    printf("GET TIME: %lu %d %lu %lu:%d:%d:%d %lu\n",
        (uint32_t)(RtcHandle.Instance->SSR),
        RTC_SYNCH_PREDIV,
        (RTC_SYNCH_PREDIV - stimestructure.SubSeconds),
        stimestructure.SubSeconds,
        stimestructure.Seconds,
        stimestructure.Minutes,
        stimestructure.Hours,
        (uint32_t)time);
#endif

    return time;
}



