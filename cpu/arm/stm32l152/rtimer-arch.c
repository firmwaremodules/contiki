/*
 * Copyright (c) 2012, STMicroelectronics.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
/*---------------------------------------------------------------------------*/
/**
 *
 * Implementation of the arch-specific rtimer functions for the STM32L152
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "platform-conf.h"

#include "sys/rtimer.h"

#include "dev/soc-rtc.h"
#include "st-lib.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
* \brief We don't need to do anything special here. The RTC is initialised
* elsewhere
*/
void
rtimer_arch_init(void)
{
    return;
}
/*---------------------------------------------------------------------------*/
/**
* \brief Returns the current real-time clock time
* \return The current rtimer time in ticks
*
* The value is read from the RTC counter and converted to a number of
* rtimer ticks
*
*/
rtimer_clock_t
rtimer_arch_now(void)
{
  return soc_rtc_get_current_time();
}
/*---------------------------------------------------------------------------*/
/**
* \brief Schedules an rtimer task to be triggered at time t
* \param t The time when the task will need executed.
*
* \e t is an absolute time, in other words the task will be executed AT
* time \e t, not IN \e t rtimer ticks.
*
* This function schedules a one-shot event with the RTC.
*
* This functions converts \e to a value suitable for the RTC.
*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
    /* Convert the rtimer tick value to a value suitable for the RTC */
    soc_rtc_schedule_one_shot(SOC_RTC_CHANNEL_RTIMER, t);
}
/*---------------------------------------------------------------------------*/
