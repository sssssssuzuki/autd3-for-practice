//
//  geometry.cpp
//  autd3
//
//  Created by Seki Inoue on 6/8/16.
//  Copyright © 2016 Hapis Lab. All rights reserved.
//
//

#include <codeanalysis\warnings.h>
#include <stdio.h>
#pragma warning(push)
#pragma warning(disable : ALL_CODE_ANALYSIS_WARNINGS 26812)
#include <Eigen/Geometry>
#pragma warning(pop)

#include <map>

#include "autd3.hpp"
#include "privdef.hpp"

namespace autd {

class Geometry::Device {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Device(int device_id, Eigen::Vector3f position, Eigen::Vector3f euler_angles)
      : device_id(device_id), position(position), euler_angles(euler_angles) {
    Eigen::Quaternionf quo = Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ());

    transform_matrix = Eigen::Translation3f(position) * quo;
    z_direction = quo * Eigen::Vector3f(0, 0, 1);

    int index = 0;
    for (int y = 0; y < NUM_TRANS_Y; y++)
      for (int x = 0; x < NUM_TRANS_X; x++)
        if (!IS_MISSING_TRANSDUCER(x, y)) local_trans_positions.col(index++) = Eigen::Vector3f(x * TRANS_SIZE_MM, y * TRANS_SIZE_MM, 0);

    global_trans_positions = transform_matrix * local_trans_positions;
  }

  int device_id;
  Eigen::Vector3f position;
  Eigen::Matrix<float, 3, NUM_TRANS_IN_UNIT> local_trans_positions;
  Eigen::Matrix<float, 3, NUM_TRANS_IN_UNIT> global_trans_positions;
  Eigen::Vector3f euler_angles;
  Eigen::Vector3f z_direction;
  Eigen::Affine3f transform_matrix;
};

std::shared_ptr<Geometry::Device> Geometry::device(int transducer_id) {
  int eid = transducer_id / NUM_TRANS_IN_UNIT;
  return this->_devices[eid];
}

Geometry::Geometry() {}

Geometry::~Geometry() {}

GeometryPtr Geometry::Create() { return GeometryPtr(new Geometry); }

int Geometry::AddDevice(Eigen::Vector3f position, Eigen::Vector3f euler_angles) {
  int device_id = static_cast<int>(this->_devices.size());
  this->_devices.push_back(std::shared_ptr<Device>(new Device(device_id, position, euler_angles)));
  return device_id;
}

const int Geometry::numDevices() { return static_cast<int>(this->_devices.size()); }

const int Geometry::numTransducers() { return static_cast<int>(this->numDevices() * NUM_TRANS_IN_UNIT); }

const Eigen::Vector3f Geometry::position(int transducer_id) {
  const int local_trans_id = transducer_id % NUM_TRANS_IN_UNIT;
  auto device = this->device(transducer_id);
  return device->global_trans_positions.col(local_trans_id);
}

const Eigen::Vector3f Geometry::direction(int transducer_id) {
  auto device = this->device(transducer_id);
  return device->z_direction;
}

const int Geometry::deviceIdForDeviceIdx(int device_idx) { return this->_devices[device_idx]->device_id; }

const int autd::Geometry::deviceIdForTransIdx(int transducer_id) { return this->device(transducer_id)->device_id; }

}  // namespace autd
