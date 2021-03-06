/*!
 * @file
 */

// Copyright 2016 - 2017 UMass Lowell Command and Data Handling Team

#include "../include/spacehauc-i2c-dev.h"
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

using std::string;
using std::cout;
using std::endl;

using namespace spacehauc_i2c;

string spacehauc_i2c::hexString(uint8_t decimal) {
  std::stringstream stream;
  stream << "0x" <<  std::setfill ('0') << std::setw(sizeof(uint8_t)*2)
  << std::hex << static_cast<unsigned int>(decimal);
  return stream.str();
}

static int deviceFile;
static string filepath = "/dev/i2c-";
//string I2C::filepath = "/dev/i2c-";

I2C::~I2C() {
  if (deviceFile != 0) {
    close(deviceFile);
  }
}

int I2C::openDevice() {
  return open(filepath.c_str(), O_RDWR);
}

int I2C::I2C_ctl(i2c_rdwr_ioctl_data *packets) {
  cout << deviceFile << endl;
  return ioctl(3, I2C_RDWR, packets);
}


/*!
 * initBus attempts to enable the i2c bus number that is passed to it. This bus
 * can now be written to and read from.
 *
 * @param bus The i2c bus number to be enabled. (Almost always 1)
 * @return success/failure
 */
bool I2C::initBus(int busNumber) {
  I2C bus;
  filepath += std::to_string(busNumber);
  deviceFile = open(filepath.c_str(), O_RDWR);
  if (deviceFile != 0) {
    return true;
  }
  return false;
}

/*!
 * Constructor for I2C Device. Currently has no functionality, it may in
 * a future release.
 *
 */
I2C_Device::I2C_Device() {}

/*!
 * Destructor for I2C Device. Currently has no functionality, it may in
 * a future release.
 *
 */
I2C_Device::~I2C_Device() {}

/*!
 * readBytes first edits the i2c_rdwr_ioctl_data and i2c_msg structs to include
 * parameter data. It then calls the ioctl() function to read data from data
 * register. This data is stored in the buffer
 *
 * @param reg The data register to read from.
 * @param *buffer Pointer to a buffer to store read data.
 * @param count The number of elements in the buffer.
 *
 * @return success/failure
 */
int I2C_Device::readBytes(uint8_t reg, uint8_t *buffer, uint8_t count) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  /* write the register we want to read from */
  messages[0].addr  = mAddress;
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  /* read */
  messages[1].addr  = mAddress;
  messages[1].flags = I2C_M_RD;
  messages[1].len   = count;
  messages[1].buf   = buffer;

  packets.msgs      = messages;
  packets.nmsgs     = 2;

  return I2C_ctl(&packets);
}

/*!
 * writeBytes writes input data to an i2c register. It first edits the
 * i2c_rdwr_ioctl_data and i2c_msg structs to include parameter data. Then it
 * calls the ioctl() function to write the data.
 *
 * @param reg The register to write to.
 * @param *buffer A pointer to an array of input data.
 * @param count The number of elements in the input.
 *
 * @return success/failure
 */
int I2C_Device::writeBytes(uint8_t reg, uint8_t *buffer, uint8_t count) {
  vector<uint8_t> input;
  input.push_back(reg);
  for (int i = 0; i < count; ++i) {
    input.push_back(buffer[i]);
  }
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];
  cout << hexString(mAddress) << endl;
  messages[0].addr  = mAddress;
  messages[0].flags = 0;
  messages[0].len   = count + 1;
  messages[0].buf   = &input[0];

  packets.msgs      = messages;
  packets.nmsgs     = 1;

  return I2C_ctl(&packets);
}


// Adafruit MCP9808 I2C Temperature Sensor
// More info https://www.adafruit.com/product/1782

MCP9808::MCP9808(uint8_t address) {
  mAddress = address;
  deviceName = "MCP9808_" + hexString(address);
}

MCP9808::~MCP9808() {}

bool MCP9808::init() {
  uint8_t ctlData[] = {0x00, 0x00}; // set up control register for continuous conversion
  if (writeBytes(CONTROL_REGISTER, ctlData, 2) <= 0) {
    return false;
  }
  uint8_t resolutionData[] = {0x03};
  if (writeBytes(RESOLUTION_REGISTER, resolutionData, 1) <= 0) {
    return false;
  }
  return true;
}

double MCP9808::read() {
  uint8_t data[2] = {0};
  if (readBytes(DATA_REGISTER, data, 2) <= 0) {
    return -100000; // not a temperature, use for error checking
  };
  int temp = (data[0] & 0x1F) * 256 + data[1]; // clear first 3 bits and combines
  if (temp > 4095) {
    temp -= 8192;
  }
  mTemperatureCelsius = temp * 0.0625;
  return mTemperatureCelsius;
}

/*!
 * Constructor for a LuminositySensor object.
 *
 * @param file This is the i2c bus file that the sensor is connected to.
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info.
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info.
 * @param controlRegister1 This is the first register that controls the
 *        sensor. See sensor datasheet for info.
 * @param controlRegister2 This is the second register that controls the
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured magnetic data is
 *        stored in. See sensor datasheet for info.
 */
TSL2561::TSL2561(uint8_t address) {
  mAddress = address;
  deviceName = "TSL2561_" + hexString(address);
}

/*!
 * Destructor for a TSL2561 object.
 */
TSL2561::~TSL2561() {}

/*!
 * initLuminositySensor() initializes the Luminosity Sensor by writing 0x03 to
 * the first control register (turning it on) and writing 0x02 to the second
 * one, enabling the sensor to read data.
 *
 * @return success/failure
 */
bool TSL2561::init() {
  // Select control register(0x00 | 0x80)
  // Power ON mode(0x03)
  uint8_t data = 0x03;
  if (writeBytes(0x00 | 0x80, &data, 1) <= 0) {
    return false;
  }
  // Select timing register(0x01 | 0x80)
  // Nominal integration time = 402ms(0x02)
  data = 0x02;
  if (writeBytes(0x01 | 0x80, &data, 1) <= 0) {
    return false;
  }
  return true;
}

/*!
 * readLuminositySensor() reads data from the Luminosity sensor
 *
 * @return The luminosity in lux if data was read, or -1 if data couldn't be.
 */
double TSL2561::read() {
  // Read 4 bytes of data from register(0x0C | 0x80)
  // ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
  uint8_t input = 0;
  if (writeBytes(dataRegister | 0x80, &input, 1) <= 0) {
    return -1;  // error
  }
  uint8_t data[2] = {0};
  if (readBytes(dataRegister, data, 2) <= 0) {
    return -1;  // error
  }
  // Convert the data
  return ((data[1]) * 256 + data[0]);
}

//
//
// /*!
//  * Constructor for a Temperature Sensor. It initializes member variables based
//  * on input parameters
//  *
//  * @param file This is the i2c bus file that the sensor is connected to.
//  * @param address This is the address of the sensor on the bus. See sensor
//  *        datasheet for info.
//  * @param ID_register This is the register that identifies the device. See
//  *        sensor datasheet for info.
//  * @param controlRegister This is the register that controls the temperature
//  *        sensor. See sensor datasheet for info.
//  * @param dataRegister This is the register that measured temperature data is
//  *        stored in. See sensor datasheet for info.
//  */
//  NineDoF_temp::NineDoF_temp(int file, uint8_t address,
//   uint8_t ID_register, uint8_t controlRegister, uint8_t dataRegister) {
//   mFile = file;
//   mAddress.push_back(address);
//   mID_Regsiters.push_back(ID_register);
//   mControlRegisters.push_back(controlRegister);
//   mDataRegisters.push_back(dataRegister);
// }
//
// /*!
//  * Destructor for a Temperature Sensor.
//  */
// NineDoF_temp::~NineDoF_temp() {}
//
// /*!
//  * initTempSensor initializes the temperature sensor by writing to the control
//  * register.
//  *
//  * @return  success/failure
//  */
// bool NineDoF_temp::init() {
//   uint8_t data = 0x98;
//   return (writeBytes(mControlRegisters[0], &data, 1) > 0);
// }
//
// /*!
//  * readTemp reads data from the data register using readBytes()
//  *
//  * @return mTemperature the temperature
//  */
// uint8_t NineDoF_temp::read() {
//   readBytes(mDataRegisters[0], &mTemperature, 2);
//   return mTemperature;
// }
//
// /*!
//  * Constructor for a magnetometer. It initializes the member variables based on
//  * input parameters.
//  *
//  * @param file This is the i2c bus file that the sensor is connected to.
//  * @param address This is the address of the sensor on the bus. See sensor
//  *        datasheet for info.
//  * @param ID_register This is the register that identifies the device. See
//  *        sensor datasheet for info.
//  * @param controlRegister1 This is the first register that controls the
//  *        sensor. See sensor datasheet for info.
//  * @param controlRegister2 This is the second register that controls the
//  *        sensor. See sensor datasheet for info.
//  * @param dataRegister This is the register that measured magnetic data is
//  *        stored in. See sensor datasheet for info.
//  */
// NineDoF_Magnetometer::NineDoF_Magnetometer(int file, uint8_t address, uint8_t ID_register,
//     uint8_t controlRegister1, uint8_t controlRegister2, uint8_t dataRegister,
//     MagScale scale) {
//   mFile = file;
//   mAddress.push_back(address);
//   mID_Regsiters.push_back(ID_register);
//   mControlRegisters.push_back(controlRegister1);
//   mControlRegisters.push_back(controlRegister2);
//   mDataRegisters.push_back(dataRegister);
//   mScale = scale;
// }
//
// /*!
//  * Destructor for a Magnetometer.
//  */
// NineDoF_Magnetometer::~NineDoF_Magnetometer() {}
//
// /*!
//  * initMagnetometer() initializes the Magnetometer by enabling the registers
//  *
//  * @return success/failure
//  */
// bool NineDoF_Magnetometer::init() {
//   uint8_t scale = (uint8_t) mScale << 5;
//   // all other bits 0
//   uint8_t data = 0x00;
//   // continuous conversion mode
//   return ((writeBytes(mControlRegisters[1], &data, 1) > 0) && (writeBytes(mControlRegisters[1], &scale, 1) > 0));
// }
//
// /*!
//  * readMagnetometer() reads data from the magnetometer's data register and
//  * converts it into floats.
//  *
//  * @return data The struct of x, y, and z components of the magnetic field.
//  */
// fTriplet NineDoF_Magnetometer::read() {
//   fTriplet data = {0};
//   uint8_t buffer[6] = {0};
//   readBytes(mDataRegisters[0], buffer, 6);
//   data.x = ((buffer[1] << 8) | buffer[0]);
//   data.y = ((buffer[2] << 8) | buffer[2]);
//   data.z = ((buffer[3] << 8) | buffer[4]);
//   data.x = (data.x)/*- m_bias.x) * m_scale.x */* mMagScaleValue[mScale];
//   data.y = (data.y)/*- m_bias.y) * m_scale.y */* mMagScaleValue[mScale];
//   /* invert z axis so it's positive down like other sensors */
//   data.z = -(data.z)/*- m_bias.z) * m_scale.z */* mMagScaleValue[mScale];
//   return data;
// }
//
// /*!
// * Constructor for a PWMcontroller object.
// *
// * @param file This is the i2c bus file that the sensor is connected to.
// * @param address This is the address of the PWMboard on the bus. See sensor
// *        datasheet for info.
// * @param ID_register This is the register that identifies the device. See
// *        sensor datasheet for info.
// * @param controlRegister1 This is the first register that controls the
// *        sensor. See sensor datasheet for info.
// * @param controlRegister2 This is the second register that controls the
// *        sensor. See sensor datasheet for info.
// */
// PWMcontroller::PWMcontroller(int file, uint8_t address, uint8_t ID_register,
//   uint8_t controlRegister1, uint8_t controlRegister2) {
//     mFile = file;
//     mAddress.push_back(address);
//     mID_Regsiters.push_back(ID_register);
//     mControlRegisters.push_back(controlRegister1);
//     mControlRegisters.push_back(controlRegister2);
//   }
//
//   PWMcontroller::~PWMcontroller() {}
//
// /*!
//  * initRGB_PWMcontroller() sets up the PWMcontroller to output to an RGB
//  *
//  * @return Returns true/false depending on success/failure.
//  */
// bool PWMcontroller::initRGB_PWMcontroller() {
//   // invert direction of current flow
//   // uint8_t mode2RegVal;
//   // cout << readBytes(0x01, &mode2RegVal, 1) << "   rb 328" << endl;
//   // mode2RegVal |= 0x10;
//   // cout << writeBytes(0x01, &mode2RegVal, 1) << "   wb 330" << endl;
//   return setFreq(400);
// }
//
// /*!
//  * setFreq() sets up the PWMcontroller to output to an RGB
//  *
//  * @param freq the desired frequency (in Hz) that the PWMcontroller will be
//  *   set to
//  *
//  * @return Returns true/false depending on success/failure.
//  */
// bool PWMcontroller::setFreq(float freq) {
//   uint8_t prescaler = static_cast<uint8_t>(((25000000)/(4096*freq))-1);
//   uint8_t modeReg;
//   // Set the SLEEP bit, which stops the oscillator on the part.
//   if (readBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//   modeReg |= 0x10;  // changes 5th bit to 1 to enable sleep
//   if (writeBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//   // This register can only be written when the oscillator is stopped.
//   if (writeBytes(0xfe, &prescaler, 1) <= 0) {
//     return false;
//   }
//
//   // Clear the sleep bit.
//   if (readBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//   modeReg &= ~(0x10);
//   if (writeBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//
//   usleep(500);  // According to the datasheet, we must wait 500us before
//                 //  we touch the RESTART bit after touching the SLEEP bit.
//                 //  *Maybe* we can count on that much time elapsing in the
//                 //  I2C transaction, but let's be on the safe side.
//
//   // Set the RESTART bit which, counterintuitively, clears the actual RESTART
//   //  bit in the register.
//   if (readBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//   modeReg |= 0x80;
//   if (writeBytes(0x00, &modeReg, 1) <= 0) {
//     return false;
//   }
//   return true;
// }
//
// /*!
//  * channelWrite() This function actually does the hardware interaction, setting
//  * the up/down duty cycle of each PWM channel.
//  *
//  * @param channel the channel whose value will be set.
//  * @param on The two bits written to the on time registers
//  * @param off The two bits written to the off time registers
//  *
//  * @return Returns true/false depending on success/failure.
//  */
// bool PWMcontroller::channelWrite(uint8_t channel, uint16_t on, uint16_t off) {
//   uint8_t offHigh = off >> 8;
//   uint8_t onHigh = on >> 8;
//   uint8_t onLow = on;
//   uint8_t offLow = off;
//   uint8_t onL = 0x06 + (channel*4);
//   uint8_t onH = onL + 1;
//   uint8_t offL = onL + 2;
//   uint8_t offH = onL + 3;
//   if (writeBytes(onL, &onLow, 1) <= 0) {
//     return false;
//   } else if (writeBytes(onH, &onHigh, 1) <= 0) {
//     return false;
//   } else if (writeBytes(offL, &offLow, 1) <= 0) {
//     return false;
//   } else if (writeBytes(offH, &offHigh, 1) <= 0) {
//     return false;
//   }
//   return true;
// }
//
// /*!
//  * setChlPercent() sets the percent duty cycle of a PWM channel
//  *
//  * @param channel the channel being modified
//  * @param percent the percent duty cycle
//  *
//  * @return Returns true/false depending on success/failure.
//  */
// bool PWMcontroller::setChlPercent(uint8_t channel, uint8_t percent) {
//   float weighted;
//   if (percent != 100) {
//     percent = 100 - percent;
//     weighted = 1-(log10(percent)/2);
//   } else {
//     weighted = 1;
//   }
//   if (!setChlDuty(channel, weighted*100)) {
//     return false;
//   }
//   return true;
// }
//
// /*!
//  * setChlDuty() sets the duty cycle to the given percentage.
//  *
//  * @param channel The channel to set the duty of.
//  * @param duty The percentage duty cycle to set.
//  *
//  * @return Returns true/false depending on success/failure.
//  */
// bool PWMcontroller::setChlDuty(uint8_t channel, float duty) {
//   uint16_t onTime = 0;
//   uint16_t offTime = uint16_t(duty*4096*.01)-1;
//   return (channelWrite(channel, onTime, offTime));
// }
