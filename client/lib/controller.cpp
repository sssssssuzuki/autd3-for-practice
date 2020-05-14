// File: controller.cpp
// Project: lib
// Created Date: 21/03/2018
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2020 Hapis Lab. All rights reserved.
//

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "autd3.hpp"
#include "privdef.hpp"

namespace autd {

Controller::Controller() { this->_geometry = GeometryPtr(new Geometry()); }

Controller::~Controller() { this->Close(); }

void Controller::SetLink(LinkPtr link) {
  this->Close();

  this->_link = link;
  if (this->_link->isOpen())
    this->InitPipeline();
  else
    this->Close();
}

bool Controller::isOpen() { return this->_link.get() && this->_link->isOpen(); }

void Controller::Close() {
  if (this->isOpen()) {
    this->AppendGainSync(NullGain::Create());
    this->_link->Close();
    this->FlushBuffer();
    this->_build_cond.notify_all();
    if (std::this_thread::get_id() != this->_build_thr.get_id() && this->_build_thr.joinable()) this->_build_thr.join();
    this->_send_cond.notify_all();
    if (std::this_thread::get_id() != this->_send_thr.get_id() && this->_send_thr.joinable()) this->_send_thr.join();
    this->_link = nullptr;
  }
}

GeometryPtr Controller::geometry() { return this->_geometry; }

void Controller::SetSilentMode(bool silent) { this->_silent_mode = silent; }

bool Controller::silentMode() { return this->_silent_mode; }

void Controller::AppendGain(const GainPtr gain) {
  {
    std::unique_lock<std::mutex> lk(_build_mtx);
    _build_q.push(gain);
  }
  _build_cond.notify_all();
}

void Controller::AppendGainSync(const GainPtr gain) {
  try {
    gain->SetGeometry(this->_geometry);
    if (!gain->built()) gain->build();
    size_t body_size = 0;
    std::unique_ptr<uint8_t[]> body = this->MakeBody(gain, ModulationPtr(nullptr), &body_size);
    if (this->isOpen()) this->_link->Send(body_size, std::move(body));
  } catch (int errnum) {
    this->_link->Close();
    std::cerr << "Link closed." << errnum << std::endl;
  }
}

void Controller::AppendModulation(const ModulationPtr mod) {
  {
    std::unique_lock<std::mutex> lk(_send_mtx);
    _send_mod_q.push(mod);
  }
  _send_cond.notify_all();
}

void Controller::AppendModulationSync(const ModulationPtr mod) {
  try {
    if (this->isOpen()) {
      while (mod->buffer.size() > mod->sent) {
        size_t body_size = 0;
        std::unique_ptr<uint8_t[]> body = this->MakeBody(GainPtr(nullptr), mod, &body_size);
        this->_link->Send(body_size, std::move(body));
      }
      mod->sent = 0;
    }
  } catch (int errnum) {
    this->Close();
    std::cerr << "Link closed." << errnum << std::endl;
  }
}

void Controller::InitPipeline() {
  srand(0);
  this->_build_thr = std::thread([&] {
    while (this->isOpen()) {
      GainPtr gain;
      {
        std::unique_lock<std::mutex> lk(_build_mtx);
        _build_cond.wait(lk, [&] { return _build_q.size() || !this->isOpen(); });
        if (_build_q.size()) {
          gain = _build_q.front();
          _build_q.pop();
        }
      }

      if (gain != nullptr && !gain->built()) gain->build();

      {
        std::unique_lock<std::mutex> lk(_send_mtx);
        _send_gain_q.push(gain);
        _send_cond.notify_all();
      }
    }
  });

  this->_send_thr = std::thread([&] {
    try {
      while (this->isOpen()) {
        GainPtr gain;
        ModulationPtr mod;

        {
          std::unique_lock<std::mutex> lk(_send_mtx);
          _send_cond.wait(lk, [&] { return _send_gain_q.size() || _send_mod_q.size() || !this->isOpen(); });
          if (_send_gain_q.size()) gain = _send_gain_q.front();
          if (_send_mod_q.size()) mod = _send_mod_q.front();
        }

        size_t body_size = 0;
        std::unique_ptr<uint8_t[]> body = MakeBody(gain, mod, &body_size);
        if (this->_link->isOpen()) this->_link->Send(body_size, std::move(body));

        std::unique_lock<std::mutex> lk(_send_mtx);
        if (gain.get() != nullptr) _send_gain_q.pop();
        if (mod.get() != nullptr && mod->buffer.size() <= mod->sent) {
          mod->sent = 0;
          _send_mod_q.pop();
        }
      }
    } catch (int errnum) {
      this->Close();
      std::cerr << "Link closed." << errnum << std::endl;
    }
  });
}

void Controller::FlushBuffer() {
  std::unique_lock<std::mutex> lk0(_send_mtx);
  std::unique_lock<std::mutex> lk1(_build_mtx);
  std::queue<GainPtr>().swap(_build_q);
  std::queue<GainPtr>().swap(_send_gain_q);
  std::queue<ModulationPtr>().swap(_send_mod_q);
}

std::unique_ptr<uint8_t[]> Controller::MakeBody(GainPtr gain, ModulationPtr mod, size_t *size) {
  auto num_devices = (gain != nullptr) ? gain->geometry()->numDevices() : 0;

  *size = sizeof(RxGlobalHeader) + sizeof(uint16_t) * NUM_TRANS_IN_UNIT * num_devices;
  auto body = std::make_unique<uint8_t[]>(*size);

  auto *header = reinterpret_cast<RxGlobalHeader *>(&body[0]);
  header->msg_id = static_cast<uint8_t>(rand() % 256);  // NOLINT
  header->control_flags = 0;
  header->mod_size = 0;

  if (this->_silent_mode) header->control_flags |= SILENT;

  if (mod != nullptr) {
    const int remainning_size = static_cast<int>(mod->buffer.size() - mod->sent);
    const uint8_t mod_size = std::clamp(remainning_size, 0, MOD_FRAME_SIZE);
    header->mod_size = mod_size;
    auto sent = static_cast<size_t>(mod->sent);
    if (sent == 0) header->control_flags |= LOOP_BEGIN;
    if (sent + mod_size >= mod->buffer.size()) header->control_flags |= LOOP_END;

    std::memcpy(header->mod, &mod->buffer[mod->sent], mod_size);
    mod->sent += mod_size;
  }

  auto *cursor = &body[0] + sizeof(RxGlobalHeader) / sizeof(body[0]);
  if (gain != nullptr) {
    for (int i = 0; i < gain->geometry()->numDevices(); i++) {
      auto byteSize = NUM_TRANS_IN_UNIT * sizeof(uint16_t);
      std::memcpy(cursor, &gain->_data[i].at(0), byteSize);
      cursor += byteSize / sizeof(body[0]);
    }
  }
  return body;
}
};  // namespace autd
