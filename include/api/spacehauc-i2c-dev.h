/*!
 * @file
 */

// Copyright 2016 UMass Lowell Command and Data Handling Team

#ifndef INCLUDE_SPACEHAUC_I2C_DEV_H_
#define INCLUDE_SPACEHAUC_I2C_DEV_H_

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string>
#include <vector>

using std::vector;
using std::string;

namespace spacehauc-i2c {

/*!
 * This is a structure of integer variables x y, and z that can be easily passed
 * around by methods that measure in 3 dimensions.
 */
 /*!
 * This is a structure of floating point variables x y, and z that can be easily passed
 * around by methods that measure in 3 dimensions.
 */

template <typename T>
class Triplet {
  T x;
  T y;
  T z;
public:
  Triplet() { x = y = z = 0;}
  Triplet(T X, T Y, T Z) {x = X, y = Y, z = Z;}
  ~Triplet() {}
  T getX() const {return x;}
  T getY() const {return y;}
  T getZ() const {return z;}
  void setX(T X) {x = X;}
  void setY(T Y) {y = Y;}
  void setZ(T Z) {z = Z;}
}
/*!
 * Enumerated data type for scaling the magnetometer. One of these is entered
 * into the magnetometer's constructor to set the scale.
 */
enum MagScale { MAG_SCALE_2GS, MAG_SCALE_4GS, MAG_SCALE_8GS, MAG_SCALE_12GS };

// class that contains all system I2C functions
class I2C {
 protected:
  virtual int openDevice(string I2C_device_name);
  virtual int I2C_ctl(int file, i2c_rdwr_ioctl_data *packets);
  /*! This is an integer that represents the opened I2C file (device) */
  static int deviceFile;
 public:
  virtual ~I2C();
  static bool initBus(int bus);
};

/*!
 * This is a class for any i2c device to inherit from. All i2c devices need
 * these methods.
 */
class I2C_Device : public I2C {
  // change inheritance for mocking
 protected:
  /*! A vector of each address that the device may contain. */
  vector<uint8_t> mAddress;
  /*! A vector of each of the ID registers that the device may contain. */
  vector<uint8_t> mID_Regsiters;
  int readBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
  int writeBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
 public:
  I2C_Device();
  virtual ~I2C_Device();
  bool initDevice();
  virtual bool init() = 0;
  //virtual double read() = 0;
};

/*!
 * This is a class for a temperature sensor, specifically the 9DoF board's
 * sensor.
 */
class NineDoF_temp : public I2C_Device {
 private:
  /*! A vector of each of the data registers the temperature sensor uses. */
  vector<uint8_t> mDataRegisters;
  /*! A vector of each of the control registers the temperature sensor uses. */
  vector<uint8_t> mControlRegisters;
  /*! This is a variable to hold the measured temperature value. */
  uint8_t mTemperature;
 public:
  explicit NineDoF_temp(int file, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister, uint8_t dataRegister);
  ~NineDoF_temp();
  bool init();
  uint8_t read();
};

/*!
 * This is a class for a NineDoF_Magnetometer, specifically the 9DoF board's sensor.
 */
class NineDoF_Magnetometer : public I2C_Device {
 private:
  /*! An array that holds each possible scaling for the magnetometer. */
  const float mMagScaleValue[5] = { 2 / 32768.0, 4 / 32768.0, 6 / 32768.0,
    8 / 32768.0, 12 / 32768.0 };
  /*! A variable of the enumerated type MagScale; is set to a scale's name.*/
  MagScale mScale;
  /*! A vector that holds the data register(s) for the magnetometer. */
  vector<uint8_t> mDataRegisters;
  /*! A vector that holds the control registers for the magnetometer. */
  vector<uint8_t> mControlRegisters;
 public:
  explicit NineDoF_Magnetometer(int file, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister1, uint8_t controlRegister2, uint8_t dataRegister,
    MagScale scale);
  ~NineDoF_Magnetometer();
  bool init();
  Triplet<float> read();
};

/*!
 * This is a class for a luminosity sensor.
 */
class TSL2561 : public I2C_Device {
 private:
  const uint8_t ID_register = 0x0A;
  const uint8_t controlRegister1 = 0x00;
  const uint8_t controlRegister2 = 0x01;
  const uint8_t dataRegister = 0x0C;
    /*! This is a variable to hold the measured luminosity value. */
  double mLuminosity;
 public:
  TSL2561(int file, uint8_t address);
  ~TSL2561();
  bool init();
  double read();
};

class PWMcontroller : public I2C_Device {
 private:
  /*! A vector that holds the control registers for the sensor. */
  vector<uint8_t> mControlRegisters;
 public:
  PWMcontroller(int file, uint8_t address, uint8_t ID_register,
  uint8_t controlRegister1, uint8_t controlRegister2);
  ~PWMcontroller();
  bool initRGB_PWMcontroller();
  bool setFreq(float freq);
  bool channelWrite(uint8_t channel, uint16_t on, uint16_t off);
  bool setChlPercent(uint8_t channel, uint8_t percent);
  bool setChlDuty(uint8_t channel, float duty);
};

/*!
 * This is a class for a temperature sensor, specifically the MCP9808
 * sensor.
 */
class MCP9808 : public I2C_Device {
 private:
  const uint8_t ID_REGISTER = 0x07;
  const uint8_t DATA_REGISTER = 0x05;
  const uint8_t CONTROL_REGISTER = 0x01;
  const uint8_t RESOLUTION_REGISTER = 0x08;
  /*! This is a variable to hold the measured temperature value. */
  double mTemperatureCelsius;
 public:
  explicit MCP9808(uint8_t address);
  ~MCP9808();
  bool init();
  double read();

  double getLastTemperatureCelsius() const {
    return mTemperatureCelsius;
  }
  double getLastTemperatureFahrenheit() const {
    // convert celsisus to fahrenheit then return
    return ((mTemperatureCelsius * 1.8) + 32);
  }
};

} // spacehauc-i2c

#endif  // INCLUDE_SPACEHAUC_I2C_DEV_H_
