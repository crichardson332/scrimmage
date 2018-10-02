/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/controller/KeypadManual/KeypadManual.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::KeypadManual,
                KeypadManual_plugin)

namespace scrimmage {
namespace controller {

void KeypadManual::init(std::map<std::string, std::string> &params) {
    pitch_rate_idx_ = vars_.declare("pitch_rate", scrimmage::VariableIO::Direction::Out);
    roll_rate_idx_ = vars_.declare("roll_rate", scrimmage::VariableIO::Direction::Out);
    throttle_idx_ = vars_.declare("throttle", scrimmage::VariableIO::Direction::Out);

    // params
    throttle_increment_ = scrimmage::get("throttle_increment", params, 1);
    roll_limit_deg_ = scrimmage::get("roll_limit_deg", params, 30);
    pitch_limit_deg_ = scrimmage::get("pitch_limit_deg", params, 15);

    // PIDs
    if (!pitch_to_pitch_rate_pid_.init(params["pitch_to_pitch_rate_pid"], false)) {
      printf("KeypadManual controller: Failed to set pitch PID for UAV %d\n",
             parent_->id().id());
    }
    if (!roll_to_roll_rate_pid_.init(params["roll_to_roll_rate_pid"], false)) {
      printf("KeypadManual controller: Failed to set roll PID for UAV %d\n",
             parent_->id().id());
    }

    subscribe<std::string>("GlobalNetwork", "KeypadKeyPress",
      std::bind(&KeypadManual::callback_key_press, this, std::placeholders::_1));
    subscribe<std::string>("GlobalNetwork", "KeypadKeyRelease",
      std::bind(&KeypadManual::callback_key_release, this, std::placeholders::_1));

    // init control
    pitch_rate_idx_ = 0;
    roll_rate_idx_ = 0;
    throttle_ = 0;
}

bool KeypadManual::step(double t, double dt) {

    pitch_to_pitch_rate_pid_.set_setpoint(desired_pitch_);
    roll_to_roll_rate_pid_.set_setpoint(desired_roll_);

    double pitch_rate = pitch_to_pitch_rate_pid_.step(dt, state_->quat().pitch());
    double roll_rate = roll_to_roll_rate_pid_.step(dt, state_->quat().roll());
    vars_.output(pitch_rate_idx_, pitch_rate);
    vars_.output(roll_rate_idx_, roll_rate);
    vars_.output(throttle_idx_, throttle_);

    return true;
}

void KeypadManual::callback_key_press(scrimmage::MessagePtr<std::string> msg) {
    if (msg->data == "KP_1") {
      throttle_ -= throttle_increment_;
    } else if (msg->data == "KP_2") {
      throttle_ += throttle_increment_;
    } else if (msg->data == "KP_4") {
      desired_roll_ = -roll_limit_deg_ * M_PI/180.0;
    } else if (msg->data == "KP_6") {
      desired_roll_ = roll_limit_deg_ * M_PI/180.0;
    } else if (msg->data == "KP_5") {
      desired_pitch_ = -pitch_limit_deg_ * M_PI/180.0;
    } else if (msg->data == "KP_8") {
      desired_pitch_ = pitch_limit_deg_ * M_PI/180.0;
    }
}

void KeypadManual::callback_key_release(scrimmage::MessagePtr<std::string> msg) {
    if (msg->data == "KP_4" || msg->data == "KP_6") {
      desired_roll_ = 0;
    } else if (msg->data == "KP_5" || msg->data == "KP_8") {
      desired_pitch_ = 0;
    }
}

} // namespace controller
} // namespace scrimmage
