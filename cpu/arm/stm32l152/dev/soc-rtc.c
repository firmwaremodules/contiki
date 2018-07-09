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

#define HWREG(x)  (*((volatile unsigned long *)(x)))

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

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


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
    salarmstructure.AlarmTime.Minutes = time / 60 / RTIMER_SECOND;
    salarmstructure.AlarmTime.Seconds = time / RTIMER_SECOND;
    salarmstructure.AlarmTime.SubSeconds = time % RTIMER_SECOND;

    PRINTF("soc-rtc: alarm set: %u:%u:%u\n",
        (uint32_t)salarmstructure.AlarmTime.Minutes,
        (uint32_t)salarmstructure.AlarmTime.Seconds,
        (uint32_t)salarmstructure.AlarmTime.SubSeconds);
    HAL_PWR_EnableBkUpAccess();
    if (HAL_RTC_SetAlarm_IT(&RtcHandle, &salarmstructure, FORMAT_BIN) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    HAL_PWR_DisableBkUpAccess();
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
    PRINTF("soc-rtc: ALARM\n");
    HAL_RTC_AlarmIRQHandler(&RtcHandle);
}


static void configure_rtc_clock()
{
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
}

/*---------------------------------------------------------------------------*/

/*
 * This initialization function is designed to work with the
 * external stm32cube-lib subrepo.  It assumes that
 * stm32cube_hal_init() in stm32cube-lib/stm32cube-prj/Src/stm32cube_hal_init.c
 * is executed prior to this call.
 *
 * The main purpose of this function is to re-configure the
 * RTC prescalers for maximum resolution (1/32768 s) and
 * to synchronize the prescaler (subsecond counter) with the
 * time count.
 */
void
soc_rtc_init(void)
{
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;

    RTC_HandleTypeDef *hrtc = &RtcHandle;

    configure_rtc_clock();

    HAL_PWR_EnableBkUpAccess();
    __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
    RTC_EnterInitMode(hrtc);

    hrtc->Instance->PRER = (uint32_t)(RTC_SYNCH_PREDIV);
    hrtc->Instance->PRER |= (uint32_t)(RTC_ASYNCH_PREDIV << 16); 

    memset(&sTime, 0, sizeof(sTime));
    memset(&sDate, 0, sizeof(sDate));

    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.Year = 0x18;
    sDate.Month = 0x6;
    sDate.Date = 0x25;
    sDate.WeekDay = 0x2;

    uint32_t datetmpreg = (((uint32_t)RTC_ByteToBcd2(sDate.Year) << 16) | \
        ((uint32_t)RTC_ByteToBcd2(sDate.Month) << 8) | \
        ((uint32_t)RTC_ByteToBcd2(sDate.Date)) | \
        ((uint32_t)sDate.WeekDay << 13));

    hrtc->Instance->DR = (uint32_t)(datetmpreg & RTC_DR_RESERVED_MASK);

    uint32_t tmpreg = (uint32_t)(((uint32_t)RTC_ByteToBcd2(sTime.Hours) << 16) | \
        ((uint32_t)RTC_ByteToBcd2(sTime.Minutes) << 8) | \
        ((uint32_t)RTC_ByteToBcd2(sTime.Seconds)) | \
        (((uint32_t)sTime.TimeFormat) << 16));

    hrtc->Instance->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);

    hrtc->Instance->SSR = RTC_SYNCH_PREDIV;

    /* Exit init mode */
    hrtc->Instance->ISR &= (uint32_t)~RTC_ISR_INIT;

    HAL_RTC_WaitForSynchro(hrtc);
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
    HAL_PWR_DisableBkUpAccess();
}

/*---------------------------------------------------------------------------*/


void soc_rtc_schedule_one_shot(uint8_t channel, rtimer_clock_t t)
{
    PRINTF("soc-rtc: schedule one-shot @ %u for %d\n", t, channel);
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
    RTC_TimeTypeDef sTime;
    rtimer_clock_t time;

    RTC_HandleTypeDef *hrtc = &RtcHandle;

    /* Ensure content of shadow registers are synchronized. */
    while ((hrtc->Instance->ISR & RTC_ISR_RSF) == (uint32_t)RESET);

    sTime.SubSeconds = RTC_SYNCH_PREDIV - (uint32_t)((hrtc->Instance->SSR) & RTC_SSR_SS);
    uint32_t datetmpreg = (uint32_t)(hrtc->Instance->DR & RTC_DR_RESERVED_MASK);
    uint32_t tmpreg = (uint32_t)(hrtc->Instance->TR & RTC_TR_RESERVED_MASK);

    /* Not using date - but still need to read the DR register. */
    (void)datetmpreg;

    /* Fill the structure fields with the read parameters */
    sTime.Hours = (uint8_t)((tmpreg & (RTC_TR_HT | RTC_TR_HU)) >> 16);
    sTime.Minutes = (uint8_t)((tmpreg & (RTC_TR_MNT | RTC_TR_MNU)) >> 8);
    sTime.Seconds = (uint8_t)(tmpreg & (RTC_TR_ST | RTC_TR_SU));
    sTime.TimeFormat = (uint8_t)((tmpreg & (RTC_TR_PM)) >> 16);

    /* Convert the time structure parameters to Binary format */
    sTime.Hours = (uint8_t)RTC_Bcd2ToByte(sTime.Hours);
    sTime.Minutes = (uint8_t)RTC_Bcd2ToByte(sTime.Minutes);
    sTime.Seconds = (uint8_t)RTC_Bcd2ToByte(sTime.Seconds);

    /* This is a global absolute time down to resolution of
     * RTIMER_SECONDS.
     */
    time =
        (sTime.SubSeconds) + /* already in RTIMER_SECOND units */
        (sTime.Seconds +
         sTime.Minutes * 60 +
         sTime.Hours * 60 * 60) * RTIMER_SECOND;

#if DEBUG

    PRINTF("GET TIME: %08x %lu:%d:%d:%d %lu\n",
        (uint32_t)(hrtc->Instance->ISR),
        (uint32_t)sTime.Hours,
        (uint32_t)sTime.Minutes,
        (uint32_t)sTime.Seconds,
        (uint32_t)sTime.SubSeconds,
        (uint32_t)time);
#endif

    return time;
}



