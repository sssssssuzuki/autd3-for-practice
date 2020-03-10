// File: soem_link.cpp
// Project: lib
// Created Date: 24/08/2019
// Author: Shun Suzuki
// -----
// Last Modified: 28/02/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2019-2020 Hapis Lab. All rights reserved.
//

#include "soem_link.hpp"

#include <algorithm>
#include <bitset>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "autdsoem.hpp"
#include "ec_config.hpp"
#include "privdef.hpp"

namespace autd {
void internal::SOEMLink::Open(std::string ifname) {
  _cnt = autdsoem::ISOEMController::Create();

  auto ifname_and_devNum = autd::split(ifname, ':');
  _dev_num = stoi(ifname_and_devNum[1]);
  _ifname = ifname_and_devNum[0];

  this->_config.ec_sm3_cyctime_ns = EC_SM3_CYCLE_TIME_NANO_SEC;
  this->_config.ec_sync0_cyctime_ns = EC_SYNC0_CYCLE_TIME_NANO_SEC;
  this->_config.header_size = HEADER_SIZE;
  this->_config.body_size = NUM_TRANS_IN_UNIT * 2;
  this->_config.input_frame_size = EC_INPUT_FRAME_SIZE;

  _cnt->Open(_ifname.c_str(), _dev_num, this->_config);
  _is_open = _cnt->is_open();
}

void internal::SOEMLink::Close() {
  if (_is_open) {
    _cnt->Close();
    _is_open = false;
  }
}

void internal::SOEMLink::Send(size_t size, std::unique_ptr<uint8_t[]> buf) {
  if (_is_open) {
    _cnt->Send(size, std::move(buf));
  }
}

bool internal::SOEMLink::isOpen() { return _is_open; }

}  // namespace autd
