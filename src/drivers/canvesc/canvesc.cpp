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

#include "canvesc.h"

//char Canvesc::_telemetry_device[] {};
//px4::atomic_bool Canvesc::_request_telemetry_init{false};

Canvesc::Canvesc() :
	CDev("/dev/canvesc"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(CANVESC_DISARM_VALUE);
	_mixing_output.setAllMinValues(CANVESC_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(CANVESC_MAX_THROTTLE);

}

Canvesc::~Canvesc()
{
	// make sure outputs are off
	up_canvesc_arm(false);

	// clean up the alternate device node
	unregister_class_devname(CANVESC_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	//delete _telemetry;
}

int Canvesc::init()
{
	PX4_INFO("inside Canvesc::init - Here 1");
	// do regular cdev init
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	PX4_INFO("inside Canvesc::init - Here 2 %s", CANVESC_OUTPUT_BASE_DEVICE_PATH);

	// try to claim the generic PWM output device node as well - it's OK if we fail at this
	_class_instance = register_class_devname(CANVESC_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		// lets not be too verbose
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	PX4_INFO("inside Canvesc::init - Here 2");

	_mixing_output.setDriverInstance(_class_instance);

	// Getting initial parameter values
	update_params();

	PX4_INFO("inside Canvesc::init - Here 3");

	set_mode(MODE_4PWM); // SP: This might want to be pulled from parameters

	PX4_INFO("inside Canvesc::init - Here 4");

	ScheduleNow();

	PX4_INFO("inside Canvesc::init - Here 5");

	return OK;
}

int Canvesc::set_mode(const Mode mode)
{
	unsigned old_mask = _output_mask;

	/*
	 * Configure for output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {

	//refer to dshot driver for more mode examples

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		PX4_DEBUG("MODE_4PWM");

		// default output rates
		_output_mask = 0xf;
		_outputs_initialized = false;
		_num_outputs = 4;

		break;

	case MODE_NONE:
		PX4_DEBUG("MODE_NONE");
		_output_mask = 0x0;
		_outputs_initialized = false;
		_num_outputs = 0;

		if (old_mask != _output_mask) {
			// disable motor outputs
			enable_canvesc_outputs(false);
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int Canvesc::task_spawn(int argc, char *argv[])
{
	Canvesc *instance = new Canvesc();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void Canvesc::enable_canvesc_outputs(const bool enabled)
{
	if (enabled && !_outputs_initialized && _output_mask != 0) {
		CanvescBusRate busRate = (CanvescBusRate)_param_canvesc_bus_rate.get();

		unsigned int canvesc_bus_frequency = CANVESC1000;

		switch (busRate) {
		case CanvescBusRate::Canvesc250:
			canvesc_bus_frequency = CANVESC250;
			break;

		case CanvescBusRate::Canvesc500:
			canvesc_bus_frequency = CANVESC500;
			break;

		default:
			break;
		}

		int ret = up_canvesc_init(_output_mask, canvesc_bus_frequency);

		if (ret != 0) {
			PX4_ERR("up_canvesc_init failed (%i)", ret);
			return;
		}

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_canvesc_arm(enabled);
		_outputs_on = enabled;
	}
}

/*
void Canvesc::update_telemetry_num_motors()
{
	if (!_telemetry) {
		return;
	}

	int motor_count = 0;

	if (_mixing_output.mixers()) {
		motor_count = _mixing_output.mixers()->get_multirotor_count();
	}

	_telemetry->handler.setNumMotors(motor_count);
}

void Canvesc::init_telemetry(const char *device)
{
	if (!_telemetry) {
		_telemetry = new Telemetry{};

		if (!_telemetry) {
			PX4_ERR("alloc failed");
			return;
		}
	}

	int ret = _telemetry->handler.init(device);

	if (ret != 0) {
		PX4_ERR("telemetry init failed (%i)", ret);
	}

	update_telemetry_num_motors();
}

void Canvesc::handle_new_telemetry_data(const int motor_index, const CanvescTelemetry::EscData &data)
{
	// fill in new motor data
	esc_status_s &esc_status = _telemetry->esc_status_pub.get();

	if (motor_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << motor_index;

		esc_status.esc[motor_index].timestamp       = data.time;
		esc_status.esc[motor_index].esc_rpm         = (static_cast<int>(data.erpm) * 100) / (_param_mot_pole_count.get() / 2);
		esc_status.esc[motor_index].esc_voltage     = static_cast<float>(data.voltage) * 0.01f;
		esc_status.esc[motor_index].esc_current     = static_cast<float>(data.current) * 0.01f;
		esc_status.esc[motor_index].esc_temperature = data.temperature;
		// TODO: accumulate consumption and use for battery estimation
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (motor_index <= _telemetry->last_motor_index) {
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CANVESC;
		esc_status.esc_count = _telemetry->handler.numMotors();
		++esc_status.counter;
		// FIXME: mark all ESC's as online, otherwise commander complains even for a single dropout
		esc_status.esc_online_flags = (1 << esc_status.esc_count) - 1;
		esc_status.esc_armed_flags = (1 << esc_status.esc_count) - 1;

		_telemetry->esc_status_pub.update();

		// reset esc data (in case a motor times out, so we won't send stale data)
		memset(&esc_status.esc, 0, sizeof(_telemetry->esc_status_pub.get().esc));
		esc_status.esc_online_flags = 0;
	}

	_telemetry->last_motor_index = motor_index;
}
*/

int Canvesc::send_command_thread_safe(const canvesc_command_t command, const int num_repetitions, const int motor_index)
{
	Command cmd{};
	cmd.command = command;

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << _mixing_output.reorderedMotorIndex(motor_index);
	}

	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}

	return 0;
}

/*
void Canvesc::retrieve_and_print_esc_info_thread_safe(const int motor_index)
{
	if (_request_esc_info.load() != nullptr) {
		// already in progress (not expected to ever happen)
		return;
	}

	CanvescTelemetry::OutputBuffer output_buffer{};
	output_buffer.motor_index = motor_index;

	// start the request
	_request_esc_info.store(&output_buffer);

	// wait until processed
	int max_time = 1000;

	while (_request_esc_info.load() != nullptr && max_time-- > 0) {
		px4_usleep(1000);
	}

	_request_esc_info.store(nullptr); // just in case we time out...

	if (output_buffer.buf_pos == 0) {
		PX4_ERR("No data received. If telemetry is setup correctly, try again");
		return;
	}

	CanvescTelemetry::decodeAndPrintEscInfoPacket(output_buffer);
}

int Canvesc::request_esc_info()
{
	_telemetry->handler.redirectOutput(*_request_esc_info.load());
	_waiting_for_esc_info = true;

	int motor_index = _mixing_output.reorderedMotorIndex(_request_esc_info.load()->motor_index);

	_current_command.motor_mask = 1 << motor_index;
	_current_command.num_repetitions = 1;
	_current_command.command = Canvesc_cmd_esc_info;

	PX4_DEBUG("Requesting ESC info for motor %i", motor_index);
	return motor_index;
}
*/
void Canvesc::mixerChanged()
{
	//update_telemetry_num_motors();
}

bool Canvesc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	if (stop_motors) {

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				up_canvesc_motor_command(i, _current_command.command, true);

			} else {
				up_canvesc_motor_command(i, Canvesc_cmd_motor_stop, 0);
			}
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;
		}

	} else {
		for (int i = 0; i < (int)num_outputs; i++) {

			uint16_t output = outputs[i];
			if (output == CANVESC_DISARM_VALUE) {
				up_canvesc_motor_command(i, Canvesc_cmd_motor_stop, 0);

			} else {
				up_canvesc_motor_data_set(i, math::min(output, static_cast<uint16_t>(CANVESC_MAX_THROTTLE)), 0);
			}
		}

		// clear commands when motors are running
		_current_command.clear();
	}

	if (stop_motors || num_control_groups_updated > 0) {
		up_canvesc_trigger();
	}

	return true;
}

void Canvesc::Run()
{
	PX4_INFO("Inside Canvesc::Run");

	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// update output status if armed or if mixer is loaded
	bool outputs_on = _mixing_output.armed().armed || _mixing_output.mixers();

	if (_outputs_on != outputs_on) {
		enable_canvesc_outputs(outputs_on);
	}

/*
	if (_telemetry) {
		int telem_update = _telemetry->handler.update();

		// Are we waiting for ESC info?
		if (_waiting_for_esc_info) {
			if (telem_update != -1) {
				_request_esc_info.store(nullptr);
				_waiting_for_esc_info = false;
			}

		} else if (telem_update >= 0) {
			handle_new_telemetry_data(telem_update, _telemetry->handler.latestESCData());
		}
	}
*/
	if (_parameter_update_sub.updated()) {
		update_params();
	}

/*
	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device);
		_request_telemetry_init.store(false);
	}
*/
	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void Canvesc::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>((_param_canvesc_min.get() * static_cast<int>(CANVESC_MAX_THROTTLE))),
							CANVESC_MIN_THROTTLE, CANVESC_MAX_THROTTLE));
}

int Canvesc::ioctl(file *filp, int cmd, unsigned long arg)
{
	PX4_INFO("Inside Canvesc::ioctl");

	int ret;

	// try it as a Capture ioctl next
	ret = capture_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	// if we are in valid PWM mode, try it as a PWM ioctl as well
	switch (_mode) {
	case MODE_1PWM:
	case MODE_4PWM:
		ret = canvesc_ioctl(filp, cmd, arg); //CHANGE THIS TO CAN BUS SEND. Look at UAVCAN driver
		break;

	default:
		PX4_DEBUG("not in a PWM mode");
		break;
	}

	// if nobody wants it, let CDev have it
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int Canvesc::canvesc_ioctl(file *filp, const int cmd, const unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("canvesc ioctl cmd: %d, arg: %ld", cmd, arg);

	return ret;
}

int Canvesc::capture_ioctl(file *filp, const int cmd, const unsigned long arg)
{
	int ret = -EINVAL;

	return ret;
}

int Canvesc::module_new_mode(const PortMode new_mode)
{
	if (!is_running()) {
		return -1;
	}

	return OK;
}

int Canvesc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Canvesc::print_status()
{
	const char *mode_str = nullptr;

	switch (_mode) {

	case MODE_NONE: mode_str = "no outputs"; break;

	case MODE_1PWM: mode_str = "1 motor output"; break;

	case MODE_4PWM: mode_str = "4 motor outputs"; break;

	default:
		break;
	}

	if (mode_str) {
		PX4_INFO("Mode: %s", mode_str);
	}

	PX4_INFO("Armed: %s", _mixing_output.armed().armed ? "yes" : "no");
	PX4_INFO("Mixer loaded: %s", _mixing_output.mixers() ? "yes" : "no");
	//PX4_INFO("bus_rate: %d", _param_canvesc_bus_rate);
	//PX4_INFO("command_rate: %d", _param_canvesc_motor_rate);
	//PX4_INFO("min output: %d", _param_canvesc_min);
	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

/*
	if (_telemetry) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry->handler.printStatus();
	}
*/
	return 0;
}

int Canvesc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the Canvesc  driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use CAN as VESC communication protocol instead of PWM.

It supports:
- VESC protocol
- //telemetry via separate UART and publishing as esc_status message
- //sending Canvesc commands via CLI

### Examples
Permanently reverse motor 1:
$ canvesc reverse -m 1
$ canvesc save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("canvesc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start the module if not running already");

	PRINT_MODULE_USAGE_COMMAND("mode_gpio");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "Select all available pins as PWM");

	//PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	//PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	// Canvesc commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_info", "Request ESC information");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int canvesc_main(int argc, char *argv[])
{
	return Canvesc::main(argc, argv);
}
