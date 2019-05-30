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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>

#include <iostream>
#include <iomanip>

namespace scrimmage {

State::State() : pose_(), twist_() {}

State::State(Eigen::Vector3d _pos, Eigen::Vector3d _vel,
             Eigen::Vector3d _ang_vel, Quaternion _quat) :
    pose_(_pos, _quat), twist_(_vel, _ang_vel) {}

State::~State() {}

Eigen::Vector3d &State::pos() {return pose_.pos();}
Eigen::Vector3d &State::vel() {return twist_.vel();}
Eigen::Vector3d &State::ang_vel() {return twist_.ang_vel();}
Quaternion &State::quat() {return pose_.quat();}
const Eigen::Vector3d &State::pos() const {return pose_.pos();}
const Eigen::Vector3d &State::vel() const {return twist_.vel();}
const Eigen::Vector3d &State::ang_vel() const {return twist_.ang_vel();}
const Quaternion &State::quat() const {return pose_.quat();}

// for backwards compatibility
const Eigen::Vector3d &State::pos_const() const {return pose_.pos();}
const Eigen::Vector3d &State::vel_const() const {return twist_.vel();}
const Eigen::Vector3d &State::ang_vel_const() const {return twist_.ang_vel();}
const Quaternion &State::quat_const() const {return pose_.quat();}
void State::set_pos(const Eigen::Vector3d &pos) {pose_.pos() = pos;}
void State::set_vel(const Eigen::Vector3d &vel) {twist_.vel() = vel;}
void State::set_ang_vel(const Eigen::Vector3d &ang_vel) {twist_.ang_vel() = ang_vel;}
void State::set_quat(const Quaternion &quat) {pose_.quat() = quat;}

bool State::InFieldOfView(State &other, double fov_width, double fov_height) const {
    Eigen::Vector3d rel_pos = this->rel_pos_local_frame(other.pos());
    double az = atan2(rel_pos(1), rel_pos(0));
    double norm_xy = sqrt(pow(rel_pos(0), 2) + pow(rel_pos(1), 2));
    double el = atan2(rel_pos(2), norm_xy);
    return std::abs(az) < fov_width / 2 && std::abs(el) < fov_height / 2;
}

Eigen::Vector3d State::rel_pos_local_frame(Eigen::Vector3d &other) const {
    return pose_.quat().rotate_reverse(other - pose_.pos());
}

Eigen::Vector3d State::pos_offset(double distance, bool offset_with_velocity) const {
    if (offset_with_velocity) {
        return pose_.pos() + twist_.vel().normalized() * distance;
    } else {
        return pose_.pos() + orient_global_frame() * distance;
    }
}

Eigen::Vector3d State::orient_global_frame() const {
    return pose_.quat().rotate(Eigen::Vector3d::UnitX());
}

double State::rel_az(const Eigen::Vector3d &other) const {
    Eigen::Vector3d diff = other - pose_.pos();
    double az = atan2(diff(1), diff(0));
    return Angles::angle_diff_rad(az, pose_.quat().yaw());
}

Eigen::Matrix4d State::tf_matrix(bool enable_translate) {
    // Construct 4x4 matrix of the form:
    // |rot|rot|rot| sx |
    // |rot|rot|rot| sy |
    // |rot|rot|rot| sz |
    // | x | y | z |  1 |
    // scales are zero (sx, sy, sz)
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0, 0, 3, 3) = pose_.quat().toRotationMatrix();

    if (enable_translate) {
        mat4.block(3, 0, 1, 3) = pose_.pos();
    }
    return mat4;
}

std::ostream& operator<<(std::ostream& os, const State& s) {
    const Quaternion &q = s.quat();

    os << "(" << eigen_str(s.pose_.pos(), s.output_precision)
        << "), (" << eigen_str(s.twist_.vel(), s.output_precision)
        << "), (" << eigen_str(s.twist_.ang_vel(), s.output_precision)
        << "), ("
        << std::setprecision(s.output_precision) << q.roll() << ", "
        << std::setprecision(s.output_precision) << q.pitch() << ", "
        << std::setprecision(s.output_precision) << q.yaw() << ")";
    return os;
}

} // namespace scrimmage
