/*
 *  autd3.hpp
 *  autd3
 *
 *  Created by Seki Inoue on 5/13/16.
 *  Copyright © 2016 Hapis Lab. All rights reserved.
 *
 */

#pragma once

#include <codeanalysis\warnings.h>
#pragma warning(push)
#pragma warning(disable : ALL_CODE_ANALYSIS_WARNINGS 26812)
#include <Eigen/Core>
#pragma warning(pop)

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
namespace internal {
class Link;
}
class Controller;
class Gain;
class Geometry;
class Modulation;

using GainPtr = std::shared_ptr<Gain>;
using GeometryPtr = std::shared_ptr<Geometry>;
using ModulationPtr = std::shared_ptr<Modulation>;

using EtherCATAdapter = std::pair<std::string, std::string>;
using EtherCATAdapters = std::vector<EtherCATAdapter>;
using ControllerPtr = std::shared_ptr<Controller>;

enum class LinkType { SOEM };

class Geometry {
  friend class Controller;

 public:
  Geometry();
  ~Geometry();
  static GeometryPtr Create();
  int AddDevice(Eigen::Vector3f position, Eigen::Vector3f euler_angles);
  const int numDevices();
  const int numTransducers();
  const Eigen::Vector3f position(int transducer_idx);
  const Eigen::Vector3f direction(int transducer_id);
  const int deviceIdForTransIdx(int device_index);
  const int deviceIdForDeviceIdx(int device_index);

 private:
  class Device;
  std::vector<std::shared_ptr<Device>> _devices;
  std::shared_ptr<Device> device(int transducer_id);
};

class Gain {
  friend class Controller;
  friend class Geometry;
  friend class internal::Link;

 protected:
  Gain();
  std::mutex _mtx;
  bool _built;
  GeometryPtr _geometry;
  std::map<int, std::vector<uint16_t>> _data;

 public:
  static GainPtr Create();
  virtual void build();
  void SetGeometry(const GeometryPtr &geometry);
  GeometryPtr geometry();
  bool built();
};

typedef Gain NullGain;

class FocalPointGain : public Gain {
 public:
  static GainPtr Create(Eigen::Vector3f point);
  void build();

 private:
  Eigen::Vector3f _point;
};

class Modulation {
  friend class Controller;
  friend class internal::Link;

 public:
  static ModulationPtr Create();
  static ModulationPtr Create(uint8_t amp);
  const float samplingFrequency();
  bool loop;
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
  void Open(LinkType type, std::string location = "");
  bool isOpen();
  void Close();
  GeometryPtr geometry();

  void SetSilentMode(bool silent);
  bool silentMode();

  void AppendGain(const GainPtr gain);
  void AppendGainSync(const GainPtr gain);
  void AppendModulation(const ModulationPtr modulation);
  void AppendModulationSync(const ModulationPtr modulation);

  static EtherCATAdapters EnumerateAdapters(int *const size);

 private:
  GeometryPtr _geometry;
  std::shared_ptr<internal::Link> _link;
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
