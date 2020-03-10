//
//  ethercat_link.hpp
//  autd3
//
//  Created by Seki Inoue on 6/1/16.
//  Copyright © 2016 Hapis Lab. All rights reserved.
//
//

#pragma once

#include <AdsLib.h>
#include <stdio.h>

#include <string>

#include "link.hpp"

#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#include <winnt.h>
#else
typedef void* HMODULE;
#endif

namespace autd {
namespace internal {
class EthercatLink : public Link {
 public:
  virtual void Open(std::string location);
  virtual void Open(std::string ams_net_id, std::string ipv4addr);
  virtual void Close();
  virtual void Send(size_t size, std::unique_ptr<uint8_t[]> buf);
  bool isOpen();

 protected:
  long _port = 0;
  AmsNetId _netId;
};

class LocalEthercatLink : public EthercatLink {
 public:
  void Open(std::string location = "");
  void Close();
  void Send(size_t size, std::unique_ptr<uint8_t[]> buf);

 private:
  HMODULE lib = nullptr;
};
}  // namespace internal
}  // namespace autd
