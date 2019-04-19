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

#ifndef INCLUDE_SCRIMMAGE_MATH_VECTOR_H_
#define INCLUDE_SCRIMMAGE_MATH_VECTOR_H_

#include <iostream>

namespace scrimmage {

class Vector3d {
 public:
    Vector3d();
    Vector3d(double x, double y, double z);
    friend std::ostream& operator<<(std::ostream& os, const Vector3d& v);

    double norm();
    void normalize();
    Vector3d normalized();

    double operator [](int i) const;
    double & operator [](int i);

 private:
    double x_;
    double y_;
    double z_;
};

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_MATH_VECTOR_H_
