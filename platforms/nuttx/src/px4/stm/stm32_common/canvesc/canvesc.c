/****************************************************************************
 *
 * Copyright (C) 2019, 2021 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
<<<<<<< HEAD
#include <drivers/drv_pwm_output.h>
=======
#include <drivers/drv_canvesc.h>
>>>>>>> 224d9ef273... in progress
//#include <px4_arch/canvesc.h>
//#include <px4_arch/io_timer.h>
//#include <drivers/drv_pwm_output.h>

int up_canvesc_init(uint32_t channel_mask, unsigned canvesc_pwm_freq)
{
	/* Init channels */
	int ret_val = OK;

	return ret_val;
}

void up_canvesc_trigger(void)
{
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
static void canvesc_motor_data_set(uint32_t motor_number, uint16_t throttle, bool telemetry)
{
}


void up_canvesc_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
	canvesc_motor_data_set(motor_number, throttle + DShot_cmd_MIN_throttle, telemetry);
}

void up_canvesc_motor_command(unsigned channel, uint16_t command, bool telemetry)
{
	canvesc_motor_data_set(channel, command, telemetry);
}


int up_canvesc_arm(bool armed)
{
	return 1;//Traverse todo:
}