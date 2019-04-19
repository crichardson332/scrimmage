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


#include <gtest/gtest.h>
#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <scrimmage/math/Vector.h>
#include <scrimmage/math/State.h>

using scrimmage::Vector3d;

namespace sc = scrimmage;

TEST(test_vector, norm) {
    Vector3d v1 = Vector3d(1, 0, 0);
    Vector3d v2 = Vector3d(3, 4, 0);
    EXPECT_FLOAT_EQ(v1.norm(), 1);
    EXPECT_FLOAT_EQ(v2.norm(), 5);

    auto v2_n = v2.normalized();
    EXPECT_FLOAT_EQ(v2_n[0], 0.6);
    EXPECT_FLOAT_EQ(v2_n[1], 0.8);
    EXPECT_FLOAT_EQ(v2_n[2], 0.0);
}

TEST(test_vector, ostream) {
    Vector3d v1 = Vector3d(3.14159265358979323846, 4.123, 0.00001);
    std::stringstream ss, ss10;
    ss << v1;
    EXPECT_STRCASEEQ(ss.str().c_str(), "(3.14159, 4.123, 1e-05)");

    ss10 << std::setprecision(10) << v1;
    EXPECT_STRCASEEQ(ss10.str().c_str(), "(3.141592654, 4.123, 1e-05)");
}
