/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "FwTrim.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;
using matrix::Vector3f;
using math::interpolate;

FwTrim::FwTrim(ModuleParams *parent) :
	ModuleParams(parent)
{
	updateParams();
	_airspeed = _param_fw_airspd_trim.get();
}

void FwTrim::updateParams()
{
	ModuleParams::updateParams();
	updateParameterizedTrim();
}

void FwTrim::reset()
{
	_auto_trim.reset();
}

void FwTrim::updateAutoTrim(const Vector3f &torque_sp, const float dt)
{
	_auto_trim.update(torque_sp - _parameterized_trim, dt);
}

void FwTrim::setAirspeed(const float airspeed)
{
	_airspeed = airspeed;
	updateParameterizedTrim();
}

void FwTrim::updateParameterizedTrim()
{
	/* bi-linear interpolation over airspeed for actuator trim scheduling */
	Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());

	if (_airspeed < _param_fw_airspd_trim.get()) {
		trim(0) += interpolate(_airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
				       _param_fw_dtrim_r_vmin.get(),
				       0.0f);
		trim(1) += interpolate(_airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
				       _param_fw_dtrim_p_vmin.get(),
				       0.0f);
		trim(2) += interpolate(_airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
				       _param_fw_dtrim_y_vmin.get(),
				       0.0f);

	} else {
		trim(0) += interpolate(_airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
				       _param_fw_dtrim_r_vmax.get());
		trim(1) += interpolate(_airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
				       _param_fw_dtrim_p_vmax.get());
		trim(2) += interpolate(_airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
				       _param_fw_dtrim_y_vmax.get());
	}

	_parameterized_trim = trim;
}

int FwTrim::print_status()
{
	Vector3f trim = getTrim();
	printf("Trim = (%.3f, %.3f, %.3f)\n", (double)trim(0), (double)trim(1), (double)trim(2));

	_auto_trim.print_status();
	return 0;
}
