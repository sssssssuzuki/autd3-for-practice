// File: soem_link.hpp
// Project: lib
// Created Date: 24/08/2019
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2019-2020 Hapis Lab. All rights reserved.
//

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autd3.hpp"
#include "autdsoem.hpp"

namespace autd {

using EtherCATAdapter = std::pair<std::string, std::string>;
using EtherCATAdapters = std::vector<EtherCATAdapter>;

class SOEMLink : public Link {
 public:
  SOEMLink();
  ~SOEMLink();
  static LinkPtr Create(std::string ifname, int device_num);

  void Open() final;
  void Close() final;
  void Send(size_t size, std::unique_ptr<uint8_t[]> buf) final;
  bool isOpen() final;
  static EtherCATAdapters SOEMLink::EnumerateAdapters(int *const size);

 private:
  std::string _ifname;
  int _device_num;
  std::unique_ptr<autdsoem::ISOEMController> _cnt;
  bool _is_open = false;
};
}  // namespace autd
