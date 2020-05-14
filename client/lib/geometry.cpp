// File: geometry.cpp
// Project: lib
// Created Date: 09/03/2020
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2020 Hapis Lab. All rights reserved.
//

#include <stdio.h>

#include <Eigen/Geometry>
#include <map>

#include "autd3.hpp"
#include "privdef.hpp"

namespace autd {

class Geometry::Device {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Device(Eigen::Vector3f position, Eigen::Vector3f euler_angles) {
    Eigen::Quaternionf quo = Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ());

    Eigen::Affine3f transform_matrix = Eigen::Translation3f(position) * quo;
    _z_direction = quo * Eigen::Vector3f(0, 0, 1);

    Eigen::Matrix<float, 3, NUM_TRANS_IN_UNIT> local_trans_positions;
    int index = 0;
    for (int y = 0; y < NUM_TRANS_Y; y++)
      for (int x = 0; x < NUM_TRANS_X; x++)
        if (!IS_MISSING_TRANSDUCER(x, y)) local_trans_positions.col(index++) = Eigen::Vector3f(x * TRANS_SIZE_MM, y * TRANS_SIZE_MM, 0);

    _global_trans_positions = transform_matrix * local_trans_positions;
  }

  Eigen::Matrix<float, 3, NUM_TRANS_IN_UNIT> _global_trans_positions;
  Eigen::Vector3f _z_direction;
};

std::shared_ptr<Geometry::Device> Geometry::device(int transducer_id) {
  int eid = transducer_id / NUM_TRANS_IN_UNIT;
  return this->_devices[eid];
}

Geometry::Geometry() {}

Geometry::~Geometry() {}

GeometryPtr Geometry::Create() { return GeometryPtr(new Geometry); }

void Geometry::AddDevice(Eigen::Vector3f position, Eigen::Vector3f euler_angles) {
  this->_devices.push_back(std::shared_ptr<Device>(new Device(position, euler_angles)));
}

const int Geometry::numDevices() { return static_cast<int>(this->_devices.size()); }

const int Geometry::numTransducers() { return static_cast<int>(this->numDevices() * NUM_TRANS_IN_UNIT); }

const Eigen::Vector3f Geometry::position(int transducer_id) {
  auto device = this->device(transducer_id);
  const int local_trans_id = transducer_id % NUM_TRANS_IN_UNIT;
  return device->_global_trans_positions.col(local_trans_id);
}

const Eigen::Vector3f Geometry::direction(int transducer_id) {
  auto device = this->device(transducer_id);
  return device->_z_direction;
}

const int autd::Geometry::deviceIdForTransIdx(int transducer_id) { return transducer_id / NUM_TRANS_IN_UNIT; }

}  // namespace autd
