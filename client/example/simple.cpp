// File: simple.cpp
// Project: simple
// Created Date: 24/08/2019
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2019-2020 Hapis Lab. All rights reserved.
//

#include <iostream>

#include "autd3.hpp"
#include "soem_link.hpp"

using namespace std;

string GetAdapterName() {
  int size;
  auto adapters = autd::SOEMLink::EnumerateAdapters(&size);
  for (auto i = 0; i < size; i++) {
    auto adapter = adapters[i];
    cout << "[" << i << "]: " << adapter.first << ", " << adapter.second << endl;
  }

  int index;
  cout << "Choose number: ";
  cin >> index;
  cin.ignore();

  return adapters[index].second;
}

int main() {
  autd::Controller autd;

  autd.geometry()->AddDevice(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));

  auto ifname = GetAdapterName();
  auto link = autd::SOEMLink::Create(ifname, 1);
  autd.OpenWith(link);
  if (!autd.isOpen()) return ENXIO;

  auto g = autd::FocalPointGain::Create(Eigen::Vector3f(90, 70, 150));
  autd.AppendGainSync(g);
  autd.AppendModulationSync(autd::SineModulation::Create(150));  // 150Hz AM

  cout << "press any key to finish..." << endl;
  getchar();

  cout << "disconnecting..." << endl;
  autd.Close();

  return 0;
}
