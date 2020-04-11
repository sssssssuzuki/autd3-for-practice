//
//  modulation.cpp
//  autd3
//
//  Created by Seki Inoue on 6/11/16.
//  Copyright © 2016 Hapis Lab. All rights reserved.
//
//

#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <cmath>
#include <fstream>
#include <limits>

#include "autd3.hpp"
#include "privdef.hpp"

autd::Modulation::Modulation() {
  this->sent = 0;
  this->loop = true;
}

const float autd::Modulation::samplingFrequency() { return MOD_SAMPLING_FREQ; }

autd::ModulationPtr autd::Modulation::Create() {
  Modulation *mod = new Modulation();
  return ModulationPtr(mod);
}

autd::ModulationPtr autd::Modulation::Create(uint8_t amp) {
  Modulation *mod = new Modulation();
  mod->buffer.resize(1, amp);
  return ModulationPtr(mod);
}

autd::ModulationPtr autd::SineModulation::Create(float freq, float amp, float offset) {
  ModulationPtr mod = ModulationPtr(new SineModulation());

  freq = std::min<float>(mod->samplingFrequency() / 2, std::max<float>(1.0, freq));

  int T = static_cast<int>(1.0 / freq * mod->samplingFrequency());
  mod->buffer.resize(T, 0);
  for (int i = 0; i < T; i++) {
    float tamp = 255.0f * offset + 127.5f * amp * static_cast<float>(cos(2.0 * M_PI * i / T));
    mod->buffer[i] = static_cast<uint8_t>(floor(fmin(fmaxf(tamp, 0.0f), 255.0f)));
    if (mod->buffer[i] == 0) mod->buffer[i] = 1;
  }
  mod->loop = true;
  return mod;
}
