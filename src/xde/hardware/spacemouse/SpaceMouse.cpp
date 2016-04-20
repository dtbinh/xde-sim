#include "SpaceMouse.h"

#include "base/Types.h"

#include "sys/sysString.h"

#include <boost/lexical_cast.hpp>

namespace
{

  struct SupportedDevices 
  {
    const unsigned short vid;
    const unsigned short hid;
    const unsigned int buttons;
    SupportedDevices(unsigned short v, unsigned short h, unsigned int b) : vid(v), hid(h), buttons(b) {;}
  };

  const unsigned int supported_devices_size = 9;

  // see vprn 3Dconnexion file
  const SupportedDevices supported_devices[supported_devices_size] =
  { 
    SupportedDevices(0x46d, 0xc623, 8), // 3Dconnexion Space Traveler
    SupportedDevices(0x46d, 0xc626, 2), // 3Dconnexion Space Navigator
    SupportedDevices(0x46d, 0xc627, 15), // 3Dconnexion Space Explorer
    SupportedDevices(0x46d, 0xc621, 12), // 3Dconnexion SpaceBall 5000
    SupportedDevices(0x46d, 0xc603, 11), // 3Dconnexion Space Mouse
    SupportedDevices(0x46d, 0xc62b, 11), // 3Dconnexion Space Mouse PRO
    SupportedDevices(0x46d, 0xc625, 15), // 3Dconnexion SpacePilot (XXX maybe more buttons)
    SupportedDevices(0x46d, 0xc629, 15), // 3Dconnexion SpacePilot PRO (XXX maybe more buttons)
    SupportedDevices(0x256f, 0xc62f, 11) // 3Dconnexion Space Mouse WIRELESS (XXX check if correct number of buttons)
  };

  enum AxisConvention
  {
    AXIS_CONVENTION_Z_UP = 0,
    AXIS_CONVENTION_Z_IN_THE_EYE = 1,
    AXIS_CONVENTION_RAW = 2,
  };

  int maxIndex(const Eigen::Twistd& vel)
  {
    int maxIndex = 0;
    double maxAbsValue = std::abs(vel[maxIndex]);

    for(int i = 1; i < 6; ++i)
    {
      if(std::abs(vel[i]) > maxAbsValue)
      {
        maxIndex = i;
        maxAbsValue = std::abs(vel[maxIndex]);
      }
    }

    return maxIndex;
  }
}

namespace xde
{
  namespace hardware
  {
    std::vector<SpaceMouseDesc> SpaceMouse::scan()
    {
      std::vector<SpaceMouseDesc> spaceMice;

      for(unsigned int i = 0; i < supported_devices_size; ++i)
      {
        struct hid_device_info *devs, *cur_dev;

        devs = hid_enumerate(supported_devices[i].vid, supported_devices[i].hid);
        unsigned int j = 0;
        cur_dev = devs;

        while(cur_dev)
        {
          std::string manufacturer = cur_dev->manufacturer_string ? xde::sys::string::wstrtostr(cur_dev->manufacturer_string) : "";
          std::string product = cur_dev->product_string ? xde::sys::string::wstrtostr(cur_dev->product_string) : "";

          SpaceMouseDesc device;

          device.id = j++;
          device.path = std::string(cur_dev->path);
          device.name = manufacturer + " " + product + " " + boost::lexical_cast<std::string>(device.id);

          device.properties.put("VID", cur_dev->vendor_id);
          device.properties.put("PID", cur_dev->product_id);
          device.properties.put("path", device.path);
          device.properties.put("id", device.id);
          device.properties.put("Manuifacturer", manufacturer); 
          device.properties.put("Product", product); 
          device.properties.put("Release", cur_dev->release_number);
          device.properties.put("Interface", cur_dev->interface_number); 
          device.properties.put("buttons", supported_devices[i].buttons);

          spaceMice.push_back(device);

          cur_dev = cur_dev->next;
        }

        hid_free_enumeration(devs);
      }

      return spaceMice;
    }

    SpaceMouse::SpaceMouse(const SpaceMouseDesc& smd)
      : handle_(hid_open_path(smd.path.c_str()))
      , id_(smd.id)
      , resolution_(1400)
      , nbButtons_(smd.properties.get<unsigned int>("buttons"))

      , maxLinearVelocity_(.1)
      , maxAngularVelocity_(.2 * M_PI)

      , rotationOn_(true)
      , translationOn_(true)
      , dominatingModeOn_(false)

      , axisConvention_(AXIS_CONVENTION_Z_UP)

      , raw_(6, 0)
      , buttons_(smd.properties.get<unsigned int>("buttons"), false)
      , velocity_(Eigen::Twistd::Zero())
    {
      hid_set_nonblocking(handle_, 1); // non blocking mode to prevent update from being blocked (and other methods)
    }

    SpaceMouse::~SpaceMouse()
    {
      hid_close(handle_);
    }

    void SpaceMouse::setMaxLinearVelocity(double maxLinearVelocity)
    {
      maxLinearVelocity_ = maxLinearVelocity;
    }

    void SpaceMouse::setMaxAngularVelocity(double maxAngularVelocity)
    {
      maxAngularVelocity_ = maxAngularVelocity;
    }

    double SpaceMouse::getMaxLinearVelocity() const
    {
      return maxLinearVelocity_;
    }

    double SpaceMouse::getMaxAngularVelocity() const
    {
      return maxAngularVelocity_;
    }

    void SpaceMouse::setRotationOn()
    {
      rotationOn_ = true;
    }

    void SpaceMouse::setRotationOff()
    {
      rotationOn_ = false;
    }

    void SpaceMouse::setTranslationOn()
    {
      translationOn_ = true;
    }

    void SpaceMouse::setTranslationOff()
    {
      translationOn_ = false;
    }

    void SpaceMouse::setDominatingModeOn()
    {
      dominatingModeOn_ = true;
    }

    void SpaceMouse::setDominatingModeOff()
    {
      dominatingModeOn_ = false;
    }

    void SpaceMouse::setAxisConventionToZUp()
    {
      axisConvention_ = AXIS_CONVENTION_Z_UP;
    }

    void SpaceMouse::setAxisConventionToZInTheEye()
    {
      axisConvention_ = AXIS_CONVENTION_Z_IN_THE_EYE;
    }

    void SpaceMouse::setAxisConventionToRaw()
    {
      axisConvention_ = AXIS_CONVENTION_RAW;
    }

    void SpaceMouse::update()
    {
      readRawData();

      velocity_ = Eigen::Twistd(
        static_cast<double>(raw_[3]) / resolution_,
        static_cast<double>(raw_[4]) / resolution_,
        static_cast<double>(raw_[5]) / resolution_,
        static_cast<double>(raw_[0]) / resolution_,
        static_cast<double>(raw_[1]) / resolution_,
        static_cast<double>(raw_[2]) / resolution_
        );

      if(!rotationOn_)
        velocity_.getAngularVelocity () *= 0;

      if(!translationOn_)
        velocity_.getLinearVelocity () *= 0;

      if(dominatingModeOn_)
      {
        int dominatingAxisIndex = maxIndex(velocity_);
        for(int i = 0; i < 6; ++i)
          if(i != dominatingAxisIndex)
            velocity_[i] = 0.;
      }

      velocity_.getLinearVelocity() *= maxLinearVelocity_ / .3;
      velocity_.getAngularVelocity() *= maxAngularVelocity_ / .3;

      const double sqrt2over2 = std::sqrt(2.) * .5;
      
      if(axisConvention_ == AXIS_CONVENTION_Z_UP)
        velocity_ = velocity_.changeFrame(Eigen::Rotation3d(0., 1., 0., 0.));
      else if(axisConvention_ == AXIS_CONVENTION_Z_IN_THE_EYE)
        velocity_ = velocity_.changeFrame(Eigen::Rotation3d(sqrt2over2, sqrt2over2, 0., 0.));
    }

    const std::vector<int>& SpaceMouse::getRawOutput() const
    {
      return raw_;
    }

    const std::vector<bool>& SpaceMouse::getButtons() const
    {
      return buttons_;
    }

    const Eigen::Twistd& SpaceMouse::getVelocity() const
    {
      return velocity_;
    }

    void SpaceMouse::readRawData()
    {
      unsigned char buf[256];

      // Do the process twice. One time for the rotation, another for the translation.
      for(unsigned i = 0; i < 2; i++) 
      {
        int size = hid_read(handle_, buf, sizeof(buf));
        if (size == 0) 
          return;

        // see Note_1 at the end of this file
        // there are 3 chunks of data: 1 for translation, 2 for rotation, 3 for buttons.
        for(size_t i = 0; i < size / 7; i++) 
        {
          unsigned char* report = buf + (i * 7);
          unsigned char type = report[0];
          int16_t* data = reinterpret_cast<int16_t *>(report + 1);

          switch(type)
          {
          case 1: // translation
            raw_[0] = data[0];
            raw_[1] = data[1];
            raw_[2] = data[2];
            break;

          case 2: // rotation
            raw_[3] = data[0];
            raw_[4] = data[1];
            raw_[5] = data[2];
            break;

          case 3: // buttonint btn; see Note_2 at the end of this file
            uint16_t data = *reinterpret_cast<uint16_t*>(report + 1);
            for (uint16_t btn = 0; btn < nbButtons_; btn++)
              buttons_[btn] = ((data & (1 << btn)) != 0);
            break;
          }
        }
      }
    }
  }
}

// --- Note_1
// (doc from vprn)
// Full reports for all of the pro devices are 7 bytes long (the first
// byte is the report type, because this device has multiple ones the
// HIDAPI library leaves it in the report).

// --- Note_2
// (doc from vprn)
// Button reports are encoded as bits in the first 2 bytes
// after the type.  There can be more than one byte if there
// are more than 8 buttons such as on SpaceExplorer or SpaceBall5000.
// If 8 or less, we don't look at 2nd byte. No known devices with >15 buttons.
// SpaceExplorer buttons are (for example):
// Name           Number
// 1              0
// 2              1
// T              2
// L              3
// R              4
// F              5
// ESC            6
// ALT            7
// SHIFT          8
// CTRL           9
// FIT            10
// PANEL          11
// +              12
// -              13
// 2D             14