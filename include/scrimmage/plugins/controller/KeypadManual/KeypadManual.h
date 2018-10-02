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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_KEYPADMANUAL_KEYPADMANUAL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_KEYPADMANUAL_KEYPADMANUAL_H_

#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/VariableIO.h>
#include <scrimmage/common/PID.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class KeypadManual : public scrimmage::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    uint8_t pitch_rate_idx_ = 0;
    uint8_t roll_rate_idx_ = 0;
    uint8_t throttle_idx_ = 0;

    double desired_roll_;
    double desired_pitch_;
    double throttle_;

    // control
    scrimmage::PID pitch_to_pitch_rate_pid_;
    scrimmage::PID roll_to_roll_rate_pid_;

    // params
    double throttle_increment_;
    double roll_limit_deg_;
    double pitch_limit_deg_;

    // callbacks
    void callback_key_press(scrimmage::MessagePtr<std::string> msg);
    void callback_key_release(scrimmage::MessagePtr<std::string> msg);
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_KEYPADMANUAL_KEYPADMANUAL_H_
