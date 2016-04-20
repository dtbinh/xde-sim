#ifndef __XDE_HARDWARE_SPACEMOUSE_H__
#define __XDE_HARDWARE_SPACEMOUSE_H__

#pragma once

#include "hidapi/hidapi.h"

#include <Eigen/Lgsm>

#include <boost/property_tree/ptree.hpp>

#include <string>
#include <vector>

namespace xde
{
  namespace hardware
  {
    struct SpaceMouseDesc
    {
      std::string name;
      std::string path;
      unsigned int id;
      boost::property_tree::ptree properties;
    };

    class SpaceMouse
    {
    public:
      static std::vector<SpaceMouseDesc> scan();

    public:
      SpaceMouse(const SpaceMouseDesc& smd);
      ~SpaceMouse();

      void setMaxLinearVelocity(double maxLinearVelocity);
      void setMaxAngularVelocity(double maxAngularVelocity);

      double getMaxLinearVelocity() const;
      double getMaxAngularVelocity() const;

      void setRotationOn();
      void setRotationOff();

      void setTranslationOn();
      void setTranslationOff();

      void setDominatingModeOn();
      void setDominatingModeOff();

      void setAxisConventionToZUp();
      void setAxisConventionToZInTheEye();
      void setAxisConventionToRaw();

      void setObsFrame(const Eigen::Displacementd& d);
      const Eigen::Displacementd& getObsFrame() const;

      void update();

      const std::vector<int>& getRawOutput() const;
      const std::vector<bool>& getButtons() const;
      const Eigen::Twistd& getVelocity() const;

    private:
      void readRawData();

    private:
      hid_device* handle_;
      unsigned int id_;
      int resolution_;
      unsigned int nbButtons_;

      double maxLinearVelocity_;
      double maxAngularVelocity_;

      bool rotationOn_;
      bool translationOn_;
      bool dominatingModeOn_;

      int axisConvention_;

      std::vector<bool> buttons_;
      std::vector<int> raw_;
      Eigen::Twistd velocity_;
    };
  }
}

#endif // __XDE_HARDWARE_SPACEMOUSE_H__
