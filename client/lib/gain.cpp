// File: gain.cpp
// Project: lib
// Created Date: 21/03/2018
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2020 Hapis Lab. All rights reserved.
//

#include <cassert>
#include <map>
#include <vector>

#include "autd3.hpp"
#include "privdef.hpp"

namespace autd {

Gain::Gain() {
  this->_built = false;
  this->_geometry = nullptr;
}

bool Gain::built() { return this->_built; }

void Gain::SetGeometry(const GeometryPtr &geometry) { this->_geometry = geometry; }

GeometryPtr Gain::geometry() { return this->_geometry; }

GainPtr NullGain::Create() {
  auto ptr = std::make_shared<NullGain>();
  ptr->_geometry = nullptr;
  return ptr;
}

void NullGain::build() {
  if (this->_built) return;
  assert(this->geometry() != nullptr);

  for (int i = 0; i < this->geometry()->numDevices(); i++) {
    this->_data[i] = std::vector<uint16_t>(NUM_TRANS_IN_UNIT, 0x0000);
  }

  this->_built = true;
}

GainPtr FocalPointGain::Create(Eigen::Vector3f point) {
  auto ptr = std::make_shared<FocalPointGain>();
  ptr->_point = point;
  ptr->_geometry = nullptr;
  return ptr;
}

void FocalPointGain::build() {
  if (this->built()) return;
  assert(this->geometry() != nullptr);

  const int ndevice = this->geometry()->numDevices();
  for (int i = 0; i < ndevice; i++) this->_data[i].resize(NUM_TRANS_IN_UNIT);

  const int ntrans = this->geometry()->numTransducers();
  for (int i = 0; i < ntrans; i++) {
    Eigen::Vector3f trp = this->geometry()->position(i);
    float dist = (trp - this->_point).norm();
    float fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH;
    uint8_t phase = static_cast<uint8_t>(round(255.0f * (1.0f - fphase)));
    uint8_t amp = 0xff;
    int dev_idx = this->geometry()->deviceIdForTransIdx(i);
    this->_data[dev_idx][i % NUM_TRANS_IN_UNIT] = ((uint16_t)amp << 8) + phase;
  }

  this->_built = true;
}
}  // namespace autd
