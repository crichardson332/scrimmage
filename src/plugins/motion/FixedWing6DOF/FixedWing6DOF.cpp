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

#include <scrimmage/plugins/motion/FixedWing6DOF/FixedWing6DOF.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>

#include <iostream>

#include <boost/algorithm/clamp.hpp>

using std::cout;
using std::endl;

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::FixedWing6DOF, FixedWing6DOF_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;
namespace pl = std::placeholders;

enum ControlParams {
    THRUST = 0,
    ELEVATOR,
    AILERON,
    RUDDER,
    CONTROL_NUM_ITEMS
};

FixedWing6DOF::FixedWing6DOF() : min_velocity_(15.0), max_velocity_(40.0),
                                 max_roll_(30.0), max_pitch_(30.0),
                                 write_csv_(false) {
}

std::tuple<int, int, int> FixedWing6DOF::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool FixedWing6DOF::init(std::map<std::string, std::string> &info,
                          std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();
    //quat_world_ = state_->quat();

    min_velocity_ = sc::get("min_velocity", params, 15.0);
    max_velocity_ = sc::get("max_velocity", params, 40.0);
    max_roll_ = sc::Angles::deg2rad(sc::get("max_roll", params, 30.0));
    max_pitch_ = sc::Angles::deg2rad(sc::get("max_pitch", params, 30.0));

    x_[U] = 0;
    x_[V] = 0;
    x_[W] = 0;

    x_[P] = 0;
    x_[Q] = 0;
    x_[R] = 0;

    x_[Uw] = 0;
    x_[Vw] = 0;
    x_[Ww] = 0;

    x_[Xw] = pos(0);
    x_[Yw] = pos(1);
    x_[Zw] = pos(2);

    // Initial Local orientation (no rotation)
    //x_[q0] = 1;
    //x_[q1] = 0;
    //x_[q2] = 0;
    //x_[q3] = 0;
    x_[q0] = state_->quat().w();
    x_[q1] = state_->quat().x();
    x_[q2] = state_->quat().y();
    x_[q3] = state_->quat().z();

    // Parse XML parameters
    g_ = sc::get<double>("gravity_magnitude", params, 9.81);
    mass_ = sc::get<double>("mass", params, 1.2);

    // Parse inertia matrix
    std::vector<std::vector<std::string>> vecs;
    std::string inertia_matrix = sc::get<std::string>("inertia_matrix",
                                                      params, "");
    bool valid_inertia = false;
    if (!sc::get_vec_of_vecs(inertia_matrix, vecs)) {
        cout << "Failed to parse inertia_matrix:" << inertia_matrix << endl;
    } else {
        int row = 0;
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 3) {
                cout << "Invalid vector size in: " << inertia_matrix << endl;
                break;
            }
            I_(row, 0) = std::stod(vec[0]);
            I_(row, 1) = std::stod(vec[1]);
            I_(row, 2) = std::stod(vec[2]);
            row++;
        }
        if (row == 3) {
            valid_inertia = true;
        }
    }
    if (!valid_inertia) {
        cout << "Using identity matrix for inertia." << endl;
        I_ = Eigen::Matrix3d::Identity();
    }
    I_inv_ = I_.inverse();

    write_csv_ = sc::get<bool>("write_csv", params, false);
    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                    "x", "y", "z",
                    "U", "V", "W",
                    "P", "Q", "R",
                    "U_dot", "V_dot", "W_dot",
                    "P_dot", "Q_dot", "R_dot",
                    "roll", "pitch", "yaw",
                    "thrust", "elevator", "aileron", "rudder"});
    }

    return true;
}

bool FixedWing6DOF::step(double time, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();

    // TODO: Saturate inputs
    x_[q0] = state_->quat().w();
    x_[q1] = state_->quat().x();
    x_[q2] = state_->quat().y();
    x_[q3] = state_->quat().z();

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    // TODO: convert global linear velocity and angular velocity into local
    // velocities

    // Cache values to calculate changes:
    Eigen::Vector3d prev_linear_vel(x_[U], x_[V], x_[W]);
    Eigen::Vector3d prev_angular_vel(x_[P], x_[Q], x_[R]);

    // Apply any external forces
    //force_ext_body_ = quat_local_.rotate_reverse(ext_force_);
    //ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable
    force_ext_body_ = state_->quat().rotate_reverse(ext_force_);
    ext_force_ = Eigen::Vector3d::Zero(); // reset ext_force_ member variable

    ode_step(dt);

    // Calculate change in velocity to populate acceleration elements
    Eigen::Vector3d linear_vel(x_[U], x_[V], x_[W]);
    Eigen::Vector3d angular_vel(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d linear_acc = (linear_vel - prev_linear_vel) / dt;
    Eigen::Vector3d angular_acc = (angular_vel - prev_angular_vel) / dt;
    x_[U_dot] = linear_acc(0);
    x_[V_dot] = linear_acc(1);
    x_[W_dot] = linear_acc(2);
    x_[P_dot] = angular_acc(0);
    x_[Q_dot] = angular_acc(1);
    x_[R_dot] = angular_acc(2);

    // // Normalize quaternion
    // quat_local_.w() = x_[q0];
    // quat_local_.x() = x_[q1];
    // quat_local_.y() = x_[q2];
    // quat_local_.z() = x_[q3];
    // quat_local_.normalize();
    //
    // x_[q0] = quat_local_.w();
    // x_[q1] = quat_local_.x();
    // x_[q2] = quat_local_.y();
    // x_[q3] = quat_local_.z();
    state_->quat().set(x_[q0], x_[q1], x_[q2], x_[q3]);
    state_->quat().normalize();

    //sc::Quaternion quat_temp(x_[q0], x_[q1], x_[q2], x_[q3]);
    //quat_temp.normalize();
    //x_[q0] = quat_temp.w();
    //x_[q1] = quat_temp.x();
    //x_[q2] = quat_temp.y();
    //x_[q3] = quat_temp.z();

    // Convert local coordinates to world coordinates
    //state_->quat() = quat_world_ * quat_local_;
    //state_->quat() = quat_temp;
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << x_[Uw], x_[Vw], x_[Ww];

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", time},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"U_dot", x_[U_dot]},
                {"V_dot", x_[V_dot]},
                {"W_dot", x_[W_dot]},
                {"P_dot", x_[P_dot]},
                {"Q_dot", x_[Q_dot]},
                {"R_dot", x_[R_dot]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"thrust", ctrl_u_(0)},
                {"elevator", ctrl_u_(1)},
                {"aileron", ctrl_u_(2)},
                {"rudder", ctrl_u_(3)}});
    }

    return true;
}

void FixedWing6DOF::model(const vector_t &x , vector_t &dxdt , double t) {

    // TODO: Ixx in slug-ft^2, convert to metric
    double b = 27.5; // wing span (ft)
    double S = 260.0; // surface area of wing (ft^2)
    double c = 10.8; // chord length (ft)

    double CDo = 0.03;
    double CDa = 0.3;

    double C_L0 = 0.28;
    double C_La = 3.45;
    double C_LQ = 0.0; // todo, check in L_tic
    double C_Lda = 0.72;
    double C_Lde = 0.36;

    double CMo = 0.0;
    double CMq = -3.6;
    double CMa = -0.38;
    double CMda = -1.1;
    double CMde = -0.5;

    double CYb = -0.98;
    double CYdr = 0.17;

    double CLb = -0.12;
    double CLp = -0.26;
    double CLr = 0.14;
    //double CLda = 0.08;
    double CLdr = -0.105;

    double CNb = 0.25;
    double CNp = 0.022;
    double CNr = -0.35;
    double CNda = 0.06;
    double CNdr = 0.032;

    double rho = 1.2; // todo
    double rudder_limit = 0.2618;
    double aileron_limit = -0.5236;
    double elevator_limit = 0.5236;

    double alpha = atan2(x_[W], x_[U]); // angle of attack
    double beta = atan2(x_[V], x_[U]); // sideslip
    double V_tau = sqrt(pow(x_[U], 2) + pow(x_[V], 2) + pow(x_[W], 2)); // resultant velocity vector
    double Q_dyn_press = 0.5 * rho * pow(V_tau, 2); // dynamic pressure

    double alpha_dot = 0.0; // TODO
    double delta_elevator = 0.0; // todo
    double delta_Ve = 0.0;

    double VtauVe = pow((V_tau + delta_Ve)/V_tau, 2);
    double pVtS = rho * pow(V_tau, 2) * S / 2.0;
    double L_tic = (C_L0 + C_La*alpha + C_LQ*Q_dyn_press * c / (2*V_tau) + C_Lda * alpha_dot * c / (2*V_tau) + C_Lde * delta_elevator * VtauVe) * pVtS;

    double thrust = ctrl_u_(THRUST);
    // double elevator = ctrl_u_(ELEVATOR);
    // double aileron = ctrl_u_(AILERON);
    // double rudder = ctrl_u_(RUDDER);

    // Calculate force from weight in body frame:
    Eigen::Vector3d gravity_vector(0, 0, -mass_*g_);
    Eigen::Vector3d F_weight = state_->quat().rotate_reverse(gravity_vector);

    Eigen::Vector3d F_thrust(thrust, 0, 0);
    Eigen::Vector3d F_total = F_weight + F_thrust;

    dxdt[U] = x[V]*x[R] - x[W]*x[Q] + F_total(0) / mass_;
    dxdt[V] = x[W]*x[P] - x[U]*x[R] + F_total(1) / mass_;
    dxdt[W] = x[U]*x[Q] - x[V]*x[P] + F_total(2) / mass_;

    // double L = aileron + Ixx*P_dot - Ixz*R_dot - Ixz*x[P]*x[Q] + (Izz - Iyy)*x[R]*x[Q];
    // double M = elevator + Iyy*Q_dot + (Ixx-Izz)*x[P]*x[R] + (Ixz*(pow(x[P], 2)-pow(x[R], 2)));
    // double N = rudder + Izz*R_dot - Ixz*P_dot + (Iyy-Ixx)*x[P]*x[Q] + Ixz*x[Q]*x[R];
    //
    // double L_tic = L + Ixz*x[P]*x[Q] - (Izz-Iyy)*x[R]*x[Q];
    // double N_tic = N - (Iyy-Ixx)*x[P]*x[Q] - Ixz*x[R]*x[Q];
    //
    // dxdt[P] = (L_tic*Izz-N_tic*Ixz) / (Ixx*Izz - pow(Ixz, 2));
    // dxdt[Q] = (M - (Ixx - Izz)*x[P]*x[R]) - Ixz*(pow(x[P], 2)-pow(x[R], 2)) / Iyy;
    // dxdt[R] = (N_tic*Ixx+L_tic*Ixz) / (Ixx*Izz - pow(Ixz, 2));

    // Calculate moments from thrust
    Eigen::Vector3d M_thrust(0, 0, 0); // L, M, N

    Eigen::Vector3d pqr(x_[P], x_[Q], x_[R]);
    Eigen::Vector3d pqr_dot = I_inv_ * (M_thrust - pqr.cross(I_*pqr));
    dxdt[P] = pqr_dot(0);
    dxdt[Q] = pqr_dot(1);
    dxdt[R] = pqr_dot(2);

    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Local position / velocity to global
    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.w() = x[q0];
    quat.x() = x[q1];
    quat.y() = x[q2];
    quat.z() = x[q3];

    //quat = quat_world_ * quat;
    quat.normalize();

    // Convert local positions and velocities into global coordinates
    //Eigen::Matrix3d rot = quat.toRotationMatrix();

    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = quat.rotate(vel_local); // rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = vel_world(1);
    dxdt[Zw] = vel_world(2);

    Eigen::Vector3d acc_local(dxdt[U], dxdt[V], dxdt[W]);
    Eigen::Vector3d acc_world = quat.rotate(acc_local); // rot * acc_local;
    dxdt[Uw] = acc_world(0);
    dxdt[Vw] = acc_world(1);
    dxdt[Ww] = acc_world(2);

    // Accelerations get updated based on change in velocities
    dxdt[U_dot] = 0;
    dxdt[V_dot] = 0;
    dxdt[W_dot] = 0;
    dxdt[P_dot] = 0;
    dxdt[Q_dot] = 0;
    dxdt[R_dot] = 0;
}

void FixedWing6DOF::teleport(sc::StatePtr &state) {
    // x_[X] = state->pos()[0];
    // x_[Y] = state->pos()[1];
    // x_[Z] = state->pos()[2];
    // x_[ROLL] = state->quat().roll();
    // x_[PITCH] = state->quat().pitch();
    // x_[YAW] = state->quat().yaw();
    // x_[SPEED] = state->vel()[0];
}
} // namespace motion
} // namespace scrimmage
