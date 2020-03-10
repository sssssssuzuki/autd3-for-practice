//
//  gain.cpp
//  autd3
//
//  Created by Seki Inoue on 6/1/16.
//  Copyright © 2016 Hapis Lab. All rights reserved.
//
//

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "autd3.hpp"
#include "privdef.hpp"

autd::GainPtr autd::Gain::Create() { return GainPtr(new Gain()); }

autd::Gain::Gain() {
  this->_built = false;
  this->_geometry = GeometryPtr(nullptr);
}

void autd::Gain::build() {
  if (this->built()) return;
  assert(this->geometry() != nullptr);

  for (int i = 0; i < this->geometry()->numDevices(); i++) {
    this->_data[this->geometry()->deviceIdForDeviceIdx(i)] = std::vector<uint16_t>(NUM_TRANS_IN_UNIT, 0x0000);
  }
}

bool autd::Gain::built() { return this->_built; }

void autd::Gain::SetGeometry(const autd::GeometryPtr &geometry) { this->_geometry = geometry; }

autd::GeometryPtr autd::Gain::geometry() { return this->_geometry; }

autd::GainPtr autd::FocalPointGain::Create(Eigen::Vector3f point) {
  std::shared_ptr<FocalPointGain> ptr = std::shared_ptr<FocalPointGain>(new FocalPointGain());
  ptr->_point = point;
  ptr->_geometry = GeometryPtr(nullptr);
  return ptr;
}

void autd::FocalPointGain::build() {
  if (this->built()) return;
  assert(this->geometry() != nullptr);

  this->_data.clear();
  const int ndevice = this->geometry()->numDevices();
  for (int i = 0; i < ndevice; i++) {
    this->_data[this->geometry()->deviceIdForDeviceIdx(i)].resize(NUM_TRANS_IN_UNIT);
  }
  const int ntrans = this->geometry()->numTransducers();
  for (int i = 0; i < ntrans; i++) {
    Eigen::Vector3f trp = this->geometry()->position(i);
    float dist = (trp - this->_point).norm();
    float fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH;
    uint8_t amp = 0xff;
    uint8_t phase = static_cast<uint8_t>(round(255.0f * (1.0f - fphase)));
    this->_data[this->geometry()->deviceIdForTransIdx(i)][i % NUM_TRANS_IN_UNIT] = ((uint16_t)amp << 8) + phase;
  }

  this->_built = true;
}
