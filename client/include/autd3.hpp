// File: autd3.hpp
// Project: include
// Created Date: 21/03/2018
// Author: Shun Suzuki
// -----
// Last Modified: 14/05/2020
// Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
// -----
// Copyright (c) 2020 Hapis Lab. All rights reserved.
//

#pragma once

#include <Eigen/Core>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autd {
class Link;
class Controller;
class Gain;
class Geometry;
class Modulation;

using GainPtr = std::shared_ptr<Gain>;
using LinkPtr = std::shared_ptr<Link>;
using GeometryPtr = std::shared_ptr<Geometry>;
using ModulationPtr = std::shared_ptr<Modulation>;
using ControllerPtr = std::shared_ptr<Controller>;

class Geometry {
  friend class Controller;

 public:
  Geometry();
  ~Geometry();
  static GeometryPtr Create();
  void AddDevice(Eigen::Vector3f position, Eigen::Vector3f euler_angles);
  const int numDevices();
  const int numTransducers();
  const Eigen::Vector3f position(int transducer_idx);
  const Eigen::Vector3f direction(int transducer_id);
  const int deviceIdForTransIdx(int device_index);

 private:
  class Device;
  std::vector<std::shared_ptr<Device>> _devices;
  std::shared_ptr<Device> device(int transducer_id);
};

class Link {
 public:
  virtual void Open() = 0;
  virtual void Close() = 0;
  virtual void Send(size_t size, std::unique_ptr<uint8_t[]> buf) = 0;
  virtual bool isOpen() = 0;
};

class Gain {
  friend class Controller;
  friend class Geometry;

 protected:
  Gain();
  bool _built;
  GeometryPtr _geometry;
  std::map<int, std::vector<uint16_t>> _data;

 public:
  virtual void build() = 0;
  void SetGeometry(const GeometryPtr &geometry);
  GeometryPtr geometry();
  bool built();
};

class NullGain : public Gain {
 public:
  static GainPtr Create();
  void build();
};

class FocalPointGain : public Gain {
 public:
  static GainPtr Create(Eigen::Vector3f point);
  void build();

 private:
  Eigen::Vector3f _point;
};

class Modulation {
  friend class Controller;

 public:
  static ModulationPtr Create();
  static ModulationPtr Create(uint8_t amp);
  const float samplingFrequency();
  std::vector<uint8_t> buffer;

 protected:
  Modulation();

 private:
  int sent;
};

class SineModulation : public Modulation {
 public:
  static ModulationPtr Create(float freq, float amp = 1.0f, float offset = 0.5f);
};

class Controller {
 public:
  Controller();
  ~Controller();
  void OpenWith(LinkPtr link);
  bool isOpen();
  void Close();
  GeometryPtr geometry();

  void SetSilentMode(bool silent);
  bool silentMode();

  void AppendGain(const GainPtr gain);
  void AppendGainSync(const GainPtr gain);
  void AppendModulation(const ModulationPtr modulation);
  void AppendModulationSync(const ModulationPtr modulation);

 private:
  GeometryPtr _geometry;
  LinkPtr _link;
  std::queue<GainPtr> _build_q;
  std::queue<GainPtr> _send_gain_q;
  std::queue<ModulationPtr> _send_mod_q;

  std::thread _build_thr;
  std::thread _send_thr;
  std::condition_variable _build_cond;
  std::condition_variable _send_cond;
  std::mutex _build_mtx;
  std::mutex _send_mtx;

  bool _silent_mode = true;

  void InitPipeline();
  void FlushBuffer();
  std::unique_ptr<uint8_t[]> MakeBody(GainPtr gain, ModulationPtr mod, size_t *size);
};
}  // namespace autd
