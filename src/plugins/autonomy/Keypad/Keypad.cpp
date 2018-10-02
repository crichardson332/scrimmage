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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/sensor/Sensor.h>

#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

#include <scrimmage/plugins/autonomy/Keypad/Keypad.h>

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::Keypad,
                Keypad_plugin)

namespace scrimmage {
namespace autonomy {

void Keypad::init(std::map<std::string, std::string> &params) {
    laser_speed_ = scrimmage::get("laser_speed", params, 100);

    subscribe<std::string>("GlobalNetwork", "KeypadKeyPress",
      std::bind(&Keypad::callback_key_press, this, std::placeholders::_1));
    subscribe<std::string>("GlobalNetwork", "KeypadKeyRelease",
      std::bind(&Keypad::callback_key_release, this, std::placeholders::_1));

    // viz
    laser_ = std::make_shared<scrimmage_proto::Shape>();
    laser_->set_opacity(1);
    laser_->set_persistent(false);
    laser_->set_ttl(10);
    scrimmage::set(laser_->mutable_color(), 57, 255, 20);
    laser_->mutable_cone()->set_height(2);
    laser_->mutable_cone()->set_base_radius(0.2);
}

bool Keypad::step_autonomy(double t, double dt) {

    update_lasers(dt);

    return true;
}

void Keypad::callback_key_press(scrimmage::MessagePtr<std::string> msg) {
    if (msg->data == "KP_7") {
      Eigen::Vector3d dir = state_->quat().rotate(Eigen::Vector3d::UnitX());
      scrimmage::set(laser_->mutable_cone()->mutable_direction(), -dir);
      scrimmage::set(laser_->mutable_cone()->mutable_apex(), 2 * dir + state_->pos());
      lasers_.push_back(laser_);
    }
}

void Keypad::callback_key_release(scrimmage::MessagePtr<std::string> msg) {
}

void Keypad::update_lasers(double dt) {
  for (auto&& laser : lasers_) {
    Eigen::Vector3d laser_apex, laser_direction, dir;
    laser_apex << laser->mutable_cone()->apex().x(),
                  laser->mutable_cone()->apex().y(),
                  laser->mutable_cone()->apex().z();
    laser_direction << laser->mutable_cone()->direction().x(),
                  laser->mutable_cone()->direction().y(),
                  laser->mutable_cone()->direction().z();
    dir = laser_direction.normalized();
    laser_apex += laser_speed_ * dt * -1 * dir;
    scrimmage::set(laser->mutable_cone()->mutable_apex(), laser_apex);
    draw_shape(laser);
  }
}

} // namespace autonomy
} // namespace scrimmage
