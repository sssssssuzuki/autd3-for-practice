/*
 *  autd3.hpp
 *  autd3
 *
 *  Created by Seki Inoue on 5/13/16.
 *  Copyright © 2016 Hapis Lab. All rights reserved.
 *
 */

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#if WIN32
#include <codeanalysis\warnings.h>
#pragma warning(push)
#pragma warning(disable : ALL_CODE_ANALYSIS_WARNINGS)
#endif
#include <Eigen/Core>
#if WIN32
#pragma warning(pop)
#endif

/*! @namespace autd
    @brief A base namespace of this autd library
 */
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

/*! @enum LinkType
    @brief A connection type between autd board and PC.
 */
enum class LinkType { ETHERCAT, ETHERNET, USB, SERIAL, SOEM };

class Geometry {
  friend class Controller;

 public:
  Geometry();
  ~Geometry();
  static GeometryPtr Create();
  /*!
      @brief Add new device with position and rotation. Note that the transform
     is done with order: Rotate -> Translate
      @param position Position of transducer #0, which is the one at the lower
     right corner.
      @param euler_angles ZYZ convention Euler angle of the device.
      @return an id of added device, which is used to delete or do other device
     specific controls.
   */
  int AddDevice(Eigen::Vector3f position, Eigen::Vector3f euler_angles);
  /*!
      @brief Remove device from the geometry.
   */
  void DelDevice(int device_id);
  const int numDevices();
  const int numTransducers();
  const Eigen::Vector3f position(int transducer_idx);
  /*!
      @brief Normalized direction of a transducer specified by id
   */
  const Eigen::Vector3f &direction(int transducer_id);
  const int deviceIdForTransIdx(int transducer_idx);
  const int deviceIdForDeviceIdx(int device_index);

 private:
  int8_t _freq_shift;
  class impl;
  std::unique_ptr<impl> _pimpl;
};

/*!
    @brief A gain of phased array which describes amps and phases of each
   transducers
 */
class Gain {
  friend class Controller;
  friend class Geometry;
  friend class internal::Link;

 protected:
  Gain();
  std::mutex _mtx;
  bool _built;
  GeometryPtr _geometry;
  std::map<int, std::vector<uint16_t> > _data;

 public:
  static GainPtr Create();
  /*!
   @brief Calculate a gain and put it into `_data`.
   Unless called explicitly by user, this method will be called internally on
   not-main thread. Be careful to manage critical sections if you extend this
   class.
   */
  virtual void build();
  void SetGeometry(const GeometryPtr &geometry);
  GeometryPtr geometry();
  bool built();
};

/*!
    @brief A gain which represents no ultrasound.
 */
typedef Gain NullGain;

/*!
    @brief A gain with single focal point
 */
class FocalPointGain : public Gain {
 public:
  static GainPtr Create(Eigen::Vector3f point);
  void build();

 private:
  Eigen::Vector3f _point;
};

/*!
 @brief A sequence of amplitude gains which describes temporal modulation
 */
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

/*!
    @brief A controller class.

 */
class Controller {
 public:
  Controller();
  ~Controller();
  /*!
   @brief Open device by link type and location.
      The scheme of location is as follows:
      ETHERCAT - <ams net id> or <ipv4 addr>:<ams net id> (ex.
   192.168.1.2:192.168.1.3.1.1 ). The ipv4 addr will be extracted from leading 4
   octets of ams net id if not specified. ETHERNET - ipv4 addr USB      -
   ignored SERIAL   - file discriptor
   */
  void Open(LinkType type, std::string location = "");
  bool isOpen();
  void Close();
  /*!
   @brief Return the number of gains and modulations remaining in the buffer
   */
  size_t remainingInBuffer();
  GeometryPtr geometry();
  void SetGeometry(const GeometryPtr &geometry);

  void SetSilentMode(bool silent);
  bool silentMode();
  /*!
   @brief [procedure style] append base gain
   */
  void AppendGain(const GainPtr &gain);
  void AppendGainSync(const GainPtr &gain);
  void AppendModulation(const ModulationPtr &modulation);
  void AppendModulationSync(const ModulationPtr &modulation);
  void Flush();

  static EtherCATAdapters EnumerateAdapters(int *const size);

 private:
  class impl;
  std::unique_ptr<impl> _pimpl;
};
}  // namespace autd
