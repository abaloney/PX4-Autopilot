/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
#pragma once

#include <drivers/device/device.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_canvesc_output.h>
#include <drivers/drv_mixer.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>

//#include "canvesc_Telemetry.h"

using namespace time_literals;

#if !defined(BOARD_HAS_PWM) //TODO: Change this to HAS_CAN
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif

/** Canvesc bus frequency */
static constexpr unsigned int CANVESC250   = 250000u;
static constexpr unsigned int CANVESC500   = 500000u;
static constexpr unsigned int CANVESC1000  = 1000000u;

static constexpr int CANVESC_DISARM_VALUE = 0;
static constexpr int CANVESC_MIN_THROTTLE = 1;
static constexpr int CANVESC_MAX_THROTTLE = 1999;

/**
 * Intialise the Canvesc outputs using the specified configuration.
 *
 * @param bus_mask		Bitmask of can buses (LSB = channel 0) to enable.
 *				This allows some of the channels to remain configured
 *				for UAVCAN or as another function.
 * @param canvesc_bus_freq	Frequency of Can bus. Usually DSHOT250, DSHOT500, DSHOT1000
 * @return OK on success.
 */
__EXPORT extern int up_canvesc_init(uint32_t bus_mask, unsigned canvesc_bus_freq);

/**
 * Set the current canvesc throttle value for a channel (motor).
 *
 * @param channel	The channel to set.
 * @param throttle	The output canvesc throttle value in [0 = CANVESC_DISARM_VALUE, 1 = CANVESC_MIN_THROTTLE, 1999 = CANVESC_MAX_THROTTLE].
 * @param telemetry	If true, request telemetry from that VESC
 */
__EXPORT extern void up_canvesc_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry);

/**
 * Send Canvesc command to a channel (VESC).
 *
 * @param channel	The channel to set.
 * @param command	canvesc_command_t
 * @param telemetry	If true, request telemetry from that VESC
 */
__EXPORT extern void up_canvesc_motor_command(unsigned channel, uint16_t command, bool telemetry);

/**
 * Trigger canvesc data transfer.
 */
__EXPORT extern void up_canvesc_trigger(void);

/**
 * Arm or disarm canvesc outputs (This will enable/disable complete timer for safety purpose.).
 *
 * When disarmed, canvesc will output no messages.
 *
 * @param armed		If true, outputs are armed; if false they are armed.
 */
__EXPORT extern int up_canvesc_arm(bool armed);

class Canvesc : public cdev::CDev, public ModuleBase<Canvesc>, public OutputModuleInterface
{
public:
	Canvesc();
	virtual ~Canvesc();

	enum Mode {
		MODE_NONE = 0,
		MODE_1PWM,
		MODE_4PWM,
	};

	/** Mode given via CLI */ // PROBABLY DON"T NEED THIS for canvesc
	enum PortMode {
		PORT_MODE_UNSET = 0,
		PORT_FULL_GPIO,
		PORT_FULL_PWM,
	};

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	Mode get_mode() { return _mode; }

	virtual int init();

	virtual int ioctl(file *filp, int cmd, unsigned long arg);

	/** change the mode of the running module */
	static int module_new_mode(const PortMode new_mode);

	void mixerChanged() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void retrieve_and_print_esc_info_thread_safe(const int motor_index);

	/**
	 * Send a canvesc command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int send_command_thread_safe(const canvesc_command_t command, const int num_repetitions, const int motor_index);

	int set_mode(const Mode new_mode);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	//bool telemetry_enabled() const { return _telemetry != nullptr; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	Canvesc(const Canvesc &) = delete;
	Canvesc operator=(const Canvesc &) = delete;

	// can bus speed bit/s
	enum class CanvescBusRate {
		Disabled  = 0,
		Canvesc250  = 250,
		Canvesc500  = 500,
		Canvesc1000  = 1000,
	};

	struct Command {
		canvesc_command_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

/*
	struct Telemetry {
		CanvescTelemetry handler{};
		uORB::PublicationData<esc_status_s> esc_status_pub{ORB_ID(esc_status)};
		int last_motor_index{-1};
	};
*/
	void capture_callback(const uint32_t channel_index, const hrt_abstime edge_time,
			      const uint32_t edge_state, const uint32_t overflow);

	int capture_ioctl(file *filp, const int cmd, const unsigned long arg);

	void enable_canvesc_outputs(const bool enabled);

	//void init_telemetry(const char *device);

	//void handle_new_telemetry_data(const int motor_index, const CanvescTelemetry::EscData &data);

	int pwm_ioctl(file *filp, const int cmd, const unsigned long arg);

	//int request_esc_info();

	void Run() override;

	void update_params();

	//void update_telemetry_num_motors();

	MixingOutput _mixing_output{DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, true};

	//Telemetry *_telemetry{nullptr};

	//static char _telemetry_device[20];
	//static px4::atomic_bool _request_telemetry_init;

	px4::atomic<Command *> _new_command{nullptr};

	//px4::atomic<CanvescTelemetry::OutputBuffer *> _request_esc_info{nullptr};

	bool _outputs_initialized{false};
	bool _outputs_on{false};
	bool _waiting_for_esc_info{false};

	unsigned _num_outputs{0};
	uint32_t _output_mask{0};

	int _class_instance{-1};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	Command _current_command{};

	Mode _mode{MODE_NONE};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CANVESC_BUS_RATE>)   	_param_canvesc_bus_rate,
		(ParamInt<px4::params::CANVESC_MIN>)    	_param_canvesc_min,
		(ParamInt<px4::params::CANVESC_PP>) 		_param_canvesc_mot_pole_pairs,
		(ParamInt<px4::params::CANVESC_MOT_RATE>) 	_param_canvesc_motor_rate
	)
};
