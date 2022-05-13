/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/**
 * @file CANVESC servo output interface.
 *
 * Servo values can be set with the CANVESC_SERVO_SET ioctl, by writing a
 * pwm_output_values structure to the device
 * Writing a value of 0 to a channel suppresses any output for that
 * channel.
 */

#pragma once

#include <px4_platform_common/defines.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <board_config.h>

#include "drv_orb_dev.h"

__BEGIN_DECLS

/**
 * Path for the default CANVESC output device.
 *
 * Note that on systems with more than one CANVESC output path (e.g.
 * PX4FMU with PX4IO connected) there may be other devices that
 * respond to this protocol.
 */
#define CANVESC_OUTPUT_BASE_DEVICE_PATH "/dev/canvesc"
#define CANVESC_OUTPUT0_DEVICE_PATH	"/dev/canvesc_output0"
#define CANVESC_OUTPUT1_DEVICE_PATH	"/dev/canvesc_output1"

#define CANVESC_OUTPUT_MAX_CHANNELS 16

struct canvesc_output_values {
	uint32_t channel_count;
	uint16_t values[CANVESC_OUTPUT_MAX_CHANNELS];
};

/* Use defaults unless the board override the defaults by providing
 * PX4_CANVESC_ALTERNATE_RANGES and a replacement set of
 * constants
 */
#if !defined(PX4_CANVESC_ALTERNATE_RANGES)

/**
 * Lowest minimum CANVESC in eRPM
 */
#define CANVESC_LOWEST_MIN 700

/**
 * Default eRPM value for a shutdown motor
 */
#define CANVESC_MOTOR_OFF	0

/**
 * Default minimum eRPM
 */
#define CANVESC_DEFAULT_MIN 2000

/**
 * Highest eRPM allowed as the minimum eRPM
 */
#define CANVESC_HIGHEST_MIN 7000

/**
 * Highest maximum eRPM
 */
#define CANVESC_HIGHEST_MAX 100000

/**
 * Default maximum eRPM
 */
#define CANVESC_DEFAULT_MAX 10000

/**
 * Default trim PWM in us
 */
//#define PWM_DEFAULT_TRIM 0

/**
 * Lowest eRPM allowed as the maximum eRPM
 */
#define CANVESC_LOWEST_MAX 0

#endif // not PX4_CANVESC_ALTERNATE_RANGES

/**
 * Do not output a channel with this value
 */
#define CANVESC_IGNORE_THIS_CHANNEL UINT16_MAX

/**
 * Servo output signal type, value is actual servo output pulse
 * width in microseconds.
 */
typedef uint16_t	servo_position_t;

/*
 * ioctl() definitions
 *
 * Note that ioctls and ORB updates should not be mixed, as the
 * behaviour of the system in this case is not defined.
 */
#define _CANVESC_SERVO_BASE		0x2a00

/** arm all servo outputs handle by this driver */
#define CANVESC_SERVO_ARM		_PX4_IOC(_CANVESC_SERVO_BASE, 0)

/** disarm all servo outputs (stop generating pulses) */
#define CANVESC_SERVO_DISARM	_PX4_IOC(_CANVESC_SERVO_BASE, 1)

/** get default servo update rate */
#define CANVESC_SERVO_GET_DEFAULT_UPDATE_RATE _PX4_IOC(_CANVESC_SERVO_BASE, 2)

/** set alternate servo update rate */
#define CANVESC_SERVO_SET_UPDATE_RATE _PX4_IOC(_CANVESC_SERVO_BASE, 3)

/** get alternate servo update rate */
#define CANVESC_SERVO_GET_UPDATE_RATE _PX4_IOC(_CANVESC_SERVO_BASE, 4)

/** get the number of servos in *(unsigned *)arg */
#define CANVESC_SERVO_GET_COUNT	_PX4_IOC(_CANVESC_SERVO_BASE, 5)

/** selects servo update rates, one bit per servo. 0 = default (50Hz), 1 = alternate */
#define CANVESC_SERVO_SET_SELECT_UPDATE_RATE _PX4_IOC(_CANVESC_SERVO_BASE, 6)

/** check the selected update rates */
#define CANVESC_SERVO_GET_SELECT_UPDATE_RATE _PX4_IOC(_CANVESC_SERVO_BASE, 7)

/** set the 'ARM ok' bit, which activates the safety switch */
#define CANVESC_SERVO_SET_ARM_OK	_PX4_IOC(_CANVESC_SERVO_BASE, 8)

/** clear the 'ARM ok' bit, which deactivates the safety switch */
#define CANVESC_SERVO_CLEAR_ARM_OK	_PX4_IOC(_CANVESC_SERVO_BASE, 9)

/** start DSM bind */
//#define DSM_BIND_START	_PX4_IOC(_CANVESC_SERVO_BASE, 10)

/** set the eRPM value for failsafe */
#define CANVESC_SERVO_SET_FAILSAFE_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 12)

/** get the PWM value for failsafe */
#define CANVESC_SERVO_GET_FAILSAFE_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 13)

/** set the PWM value when disarmed - should be no PWM (zero) by default */
#define CANVESC_SERVO_SET_DISARMED_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 14)

/** get the PWM value when disarmed */
#define CANVESC_SERVO_GET_DISARMED_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 15)

/** set the minimum PWM value the output will send */
#define CANVESC_SERVO_SET_MIN_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 16)

/** get the minimum PWM value the output will send */
#define CANVESC_SERVO_GET_MIN_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 17)

/** set the maximum PWM value the output will send */
#define CANVESC_SERVO_SET_MAX_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 18)

/** get the maximum PWM value the output will send */
#define CANVESC_SERVO_GET_MAX_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 19)

/** get the TRIM value the output will send */
#define CANVESC_SERVO_GET_TRIM_ERPM	_PX4_IOC(_CANVESC_SERVO_BASE, 21)

/** set the lockdown override flag to enable outputs in HIL */
#define CANVESC_SERVO_SET_DISABLE_LOCKDOWN		_PX4_IOC(_CANVESC_SERVO_BASE, 23)

/** get the lockdown override flag to enable outputs in HIL */
#define CANVESC_SERVO_GET_DISABLE_LOCKDOWN		_PX4_IOC(_CANVESC_SERVO_BASE, 24)

/** force safety switch off (to disable use of safety switch) */
#define CANVESC_SERVO_SET_FORCE_SAFETY_OFF		_PX4_IOC(_CANVESC_SERVO_BASE, 25)

/** force failsafe mode (failsafe values are set immediately even if failsafe condition not met) */
#define CANVESC_SERVO_SET_FORCE_FAILSAFE		_PX4_IOC(_CANVESC_SERVO_BASE, 26)

/** make failsafe non-recoverable (termination) if it occurs */
#define CANVESC_SERVO_SET_TERMINATION_FAILSAFE	_PX4_IOC(_CANVESC_SERVO_BASE, 27)

/** force safety switch on (to enable use of safety switch) */
#define CANVESC_SERVO_SET_FORCE_SAFETY_ON		_PX4_IOC(_CANVESC_SERVO_BASE, 28)

/** setup OVERRIDE_IMMEDIATE behaviour on FMU fail */
#define CANVESC_SERVO_SET_OVERRIDE_IMMEDIATE	_PX4_IOC(_CANVESC_SERVO_BASE, 32)

/** set auxillary output mode. These correspond to enum Mode in px4fmu/fmu.cpp */
#define CANVESC_SERVO_MODE_NONE         0
#define CANVESC_SERVO_MODE_1PWM         1
#define CANVESC_SERVO_MODE_4PWM         2
#define CANVESC_SERVO_ENTER_TEST_MODE  18
#define CANVESC_SERVO_EXIT_TEST_MODE   19
#define CANVESC_SERVO_SET_MODE         _PX4_IOC(_CANVESC_SERVO_BASE, 34)

/*
 *
 *
 * WARNING WARNING WARNING! DO NOT EXCEED 47 IN IOC INDICES HERE!
 *
 *
 */

/** set a single servo to a specific value */
#define CANVESC_SERVO_SET(_servo)	_PX4_IOC(_CANVESC_SERVO_BASE, 0x30 + _servo)

/** get a single specific servo value */
#define CANVESC_SERVO_GET(_servo)	_PX4_IOC(_CANVESC_SERVO_BASE, 0x50 + _servo)

/** get the _n'th rate group's channels; *(uint32_t *)arg returns a bitmap of channels
 *  whose update rates must be the same.
 */
#define CANVESC_SERVO_GET_RATEGROUP(_n) _PX4_IOC(_CANVESC_SERVO_BASE, 0x70 + _n)

/** specific rates for configuring the timer for OneShot or PWM */
#define	CANVESC_RATE_VESC			0u
#define	CANVESC_RATE_LOWER_LIMIT		1u
#define	CANVESC_RATE_UPPER_LIMIT		10000u

typedef enum {
	Canvesc_cmd_motor_stop = 0,
	Canvesc_cmd_esc_info,
	Canvesc_cmd_MAX          = 47,     // >47 are throttle values
	Canvesc_cmd_MIN_throttle = 48,
	Canvesc_cmd_MAX_throttle = 2047
} canvesc_command_t;


/*
 * Low-level CANVESC output interface.
 *
 * This is the low-level API to the platform-specific CANVESC driver.
 */

/**
 * Intialise the CANVESC servo outputs using the specified configuration.
 *
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to enable.
 *			This allows some of the channels to remain configured
 *			as GPIOs or as another function.
 * @return		OK on success.
 */
__EXPORT extern int	up_canvesc_servo_init(uint32_t channel_mask);

/**
 * De-initialise the CANVESC servo outputs.
 *
 * @param channel_mask  Bitmask of channels (LSB = channel 0) to enable.
 *      This allows some of the channels to remain configured
 *      as GPIOs or as another function.
 *      A value of 0 is ALL channels
 *
 */
__EXPORT extern void	up_canvesc_servo_deinit(uint32_t channel_mask);

/**
 * Arm or disarm servo outputs.
 *
 * When disarmed, servos output no pulse.
 *
 * @bug This function should, but does not, guarantee that any pulse
 *      currently in progress is cleanly completed.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 *
 * @param channel_mask  Bitmask of channels (LSB = channel 0) to enable.
 *      This allows some of the channels to remain configured
 *      as GPIOs or as another function.
 *      A value of 0 is ALL channels
 *
 */
__EXPORT extern void	up_canvesc_servo_arm(bool armed, uint32_t channel_mask);

/**
 * Set the servo update rate for all rate groups.
 *
 * @param rate		The update rate in Hz to set.
 * @return		OK on success, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_canvesc_servo_set_rate(unsigned rate);

/**
 * Get a bitmap of output channels assigned to a given rate group.
 *
 * @param group		The rate group to query. Rate groups are assigned contiguously
 *			starting from zero.
 * @return		A bitmap of channels assigned to the rate group, or zero if
 *			the group number has no channels.
 */
__EXPORT extern uint32_t up_canvesc_servo_get_rate_group(unsigned group);

/**
 * Set the update rate for a given rate group.
 *
 * @param group		The rate group whose update rate will be changed.
 * @param rate		The update rate in Hz.
 * @return		OK if the group was adjusted, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_canvesc_servo_set_rate_group_update(unsigned group, unsigned rate);

/**
 * Trigger all timer's channels in Oneshot mode to fire
 * the oneshot with updated values.
 * Nothing is done if not in oneshot mode.
 *
 */
__EXPORT extern void up_canvesc_update(void);

/**
 * Set the current output value for a channel.
 *
 * @param channel	The channel to set.
 * @param value		The output pulse width in microseconds.
 */
__EXPORT extern int	up_canvesc_servo_set(unsigned channel, servo_position_t value);

/**
 * Get the current output value for a channel.
 *
 * @param channel	The channel to read.
 * @return		The output pulse width in microseconds, or zero if
 *			outputs are not armed or not configured.
 */
__EXPORT extern servo_position_t up_canvesc_servo_get(unsigned channel);

/**
 * Intialise the CANVESC outputs using the specified configuration.
 *
 * @param channel_mask		Bitmask of channels (LSB = channel 0) to enable.
 *				This allows some of the channels to remain configured
 *				as GPIOs or as another function.
 * @param canvesc_bus_freq	Frequency of DSHOT signal. Usually CANVESC250, CANVESC500, CANVESC1000
 * @return OK on success.
 */
__EXPORT extern int up_canvesc_init(uint32_t channel_mask, unsigned canvesc_bus_freq);

/**
 * Set the current dshot throttle value for a channel (motor).
 *
 * @param channel	The channel to set.
 * @param throttle	The output dshot throttle value in [0 = DSHOT_DISARM_VALUE, 1 = DSHOT_MIN_THROTTLE, 1999 = DSHOT_MAX_THROTTLE].
 * @param telemetry	If true, request telemetry from that motor
 */
__EXPORT extern void up_canvesc_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry);

/**
 * Send DShot command to a channel (motor).
 *
 * @param channel	The channel to set.
 * @param command	dshot_command_t
 * @param telemetry	If true, request telemetry from that motor
 */
__EXPORT extern void up_canvesc_motor_command(unsigned channel, uint16_t command, bool telemetry);

/**
 * Trigger dshot data transfer.
 */
__EXPORT extern void up_canvesc_trigger(void);

/**
 * Arm or disarm dshot outputs (This will enable/disable complete timer for safety purpose.).
 *
 * When disarmed, dshot output no pulse.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 */
__EXPORT extern int up_canvesc_arm(bool armed);

__END_DECLS
