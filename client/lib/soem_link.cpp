// File: soem_link.cpp
// Project: lib
// Created Date: 24/08/2019
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2019-2020 Hapis Lab. All rights reserved.
//

#include "soem_link.hpp"

#include <memory>

#include "autdsoem.hpp"
#include "ec_config.hpp"
#include "privdef.hpp"

namespace autd {
LinkPtr SOEMLink::Open(std::string ifname, int device_num) {
  auto link = std::make_shared<SOEMLink>();

  link->_cnt = autdsoem::ISOEMController::Create();

  autdsoem::ECConfig config{0, 0, 0, 0, 0};
  config.ec_sm3_cyctime_ns = EC_SM3_CYCLE_TIME_NANO_SEC;
  config.ec_sync0_cyctime_ns = EC_SYNC0_CYCLE_TIME_NANO_SEC;
  config.header_size = HEADER_SIZE;
  config.body_size = NUM_TRANS_IN_UNIT * 2;
  config.input_frame_size = EC_INPUT_FRAME_SIZE;

  link->_cnt->Open(ifname.c_str(), device_num, config);
  link->_is_open = link->_cnt->is_open();

  return link;
}

void SOEMLink::Close() {
  if (_is_open) {
    _cnt->Close();
    _is_open = false;
  }
}

void SOEMLink::Send(size_t size, std::unique_ptr<uint8_t[]> buf) {
  if (_is_open) {
    _cnt->Send(size, std::move(buf));
  }
}

bool SOEMLink::isOpen() { return _is_open; }

EtherCATAdapters SOEMLink::EnumerateAdapters(int *const size) {
  auto adapters = autdsoem::EtherCATAdapterInfo::EnumerateAdapters();
  *size = static_cast<int>(adapters.size());
  EtherCATAdapters res;
  for (auto adapter : autdsoem::EtherCATAdapterInfo::EnumerateAdapters()) {
    EtherCATAdapter p;
    p.first = adapter.desc;
    p.second = adapter.name;
    res.push_back(p);
  }
  return res;
}

}  // namespace autd
