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

#include <scrimmage/math/Vector.h>

#include <cmath>
#include <stdexcept>

namespace scrimmage {

Vector3d::Vector3d()
  : x_(0.0),
    y_(0.0),
    z_(0.0) {}

Vector3d::Vector3d(double x, double y, double z)
  : x_(x),
    y_(y),
    z_(z) {}

double Vector3d::norm() {
  return std::sqrt(std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2));
}

// TODO: should this return void and error out to avoid confusion
// with the normalized() function?
void Vector3d::normalize() {
  double n = this->norm();
  x_ = x_ / n;
  y_ = y_ / n;
  z_ = z_ / n;
}

Vector3d Vector3d::normalized() {
  double n = this->norm();
  scrimmage::Vector3d v(x_ / n, y_ / n, z_ / n);
  return v;
}

double Vector3d::operator [](int i) const {
  if (i == 0) return x_;
  else if (i == 1) return y_;
  else if (i == 2) return z_;
  else throw std::out_of_range("index out of range; Vector3d index must be one of {0, 1, 2}");
}

double & Vector3d::operator [](int i) {
  if (i == 0) return x_;
  else if (i == 1) return y_;
  else if (i == 2) return z_;
  else throw std::out_of_range("index out of range; Vector3d index must be one of {0, 1, 2}");
}

/* std::ostream& Vector3d::operator <<(std::ostream& os, const Vector3d& v) { */
std::ostream& operator<<(std::ostream &os, const Vector3d& v) {
    os << "(" << v[0]
       << ", " << v[1]
       << ", " << v[2]
       << ")";
    return os;
}

} // namespace scrimmage
