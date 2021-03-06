// Copyright 2016 UMass Lowell Command and Data Handling Team

#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include "spacehauc-i2c-dev.h"

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;
using namespace spacehauc_i2c;


bool testTemperatureSensor();
bool testLuminositySensor();
//bool testMagnetometer();
//bool testRGB();
//bool setRGBColor(PWMcontroller *rgb, int red, int green, int blue);

int main(int argc, char* argv[]) {
  cout << "SPACEHAUC I2C Library Driver" << endl;
  cout << "Initializing Bus..." << endl;
  if (I2C::initBus(1) == false) {
    cerr << "Error: I2C bus failed to open." << endl;
  }
  cout << "Testing Temperature Sensor..." << endl;
  if (testTemperatureSensor()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  cout << "Testing Luminosity Sensor..." << endl;
  if (testLuminositySensor()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  /*
  cout << "Testing Magnetometer..." << endl;
  if (testMagnetometer()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  cout << "Testing RGB functionality of PWM Board..." << endl;
  if (testRGB()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  return 0;
}



// int main(int argc, char* argv[]) {
//   cout << "SPACEHAUC I2C Library Driver" << endl;
//   cout << "Initializing Bus" << endl;
//   int file;
//   int bus = 1;  // the first i2c bus
//   if (initBus(bus, &file) == false) {
//     cerr << "Error: I2C bus failed to open." << endl;
//   }
//   cout << "Testing Temperature Sensor..." << endl;
//   if (testTemperatureSensor(file)) {
//     cout << "Success" << endl;
//   } else {
//     cout << "Failure" << endl;
//   }
//   cout << "Testing Magnetometer..." << endl;
//   if (testMagnetometer(file)) {
//     cout << "Success" << endl;
//   } else {
//     cout << "Failure" << endl;
//   }
//   cout << "Testing RGB functionality of PWM Board..." << endl;
//   if (testRGB(file)) {
//     cout << "Success" << endl;
//   } else {
//     cout << "Failure" << endl;
//   }
//   cout << "Testing Luminosity Sensor..." << endl;
//   if (testLuminositySensor(file)) {
//     cout << "Success" << endl;
//   } else {
//     cout << "Failure" << endl;
//   }
//   return 0;
// }
*/
}
bool testTemperatureSensor() {
  /*
  uint8_t address = 0x1d;
  uint8_t ID_register = 0x0F;
  uint8_t ctlRegister = 0x24;
  uint8_t dataRegister = 0x05;
  */
  MCP9808 tempSensor(0x18);
  cout << "Initializing " << tempSensor.getDeviceName() << " Temperature Sensor..." << endl;
  if (tempSensor.init() == false) {
    cerr << "Error: Temperature Sensor " << tempSensor.getDeviceName() << " failed to initalize." << endl;
    return false;
  }
  cout << "Initialized Temperature Sensor " << tempSensor.getDeviceName() << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << tempSensor.getDeviceName() << ": Temperature = " << tempSensor.read() << endl;
    usleep(500000);
  }
  return true;
}

bool testLuminositySensor() {
  TSL2561 light(0x29);
  cout << "Initializing  " << light.getDeviceName() << " Luminosity Sensor..." << endl;
  if (light.init() == false) {
    cerr << "Error: Luminosity Sensor " << light.getDeviceName() << " failed to initialize." << endl;
    cout << "Attempting read: " << light.read() << endl;
    return false;
  }
  cout << "Initialized Luminosity Sensor " << light.getDeviceName() << endl;
  cout << "Reading light intensity data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << light.getDeviceName() << ": Luminosity: " << light.read() << endl;
    sleep(1);
  }
  return true;
}
//
// bool testMagnetometer() {
//   uint8_t address = 0x1d;
//   uint8_t ID_register = 0x0F;
//   uint8_t ctlRegister1 = 0x25;
//   uint8_t ctlRegister2 = 0x26;
//   uint8_t dataRegister = 0x08;
//   NineDoF_Magnetometer magnetometer(file, address, ID_register, ctlRegister1,
//     ctlRegister2, dataRegister, MAG_SCALE_2GS);
//   if (magnetometer.init() == false) {
//     cerr << "Error: Magnetometer failed to initalize." << endl;
//     return false;
//   }
//   cout << "Initialized Magnetometer" << endl;
//   cout << "Reading Magnetic Field Data..." << endl;
//   for (int i = 0; i < 5; ++i) {
//     fTriplet field = magnetometer.read();
//     cout << "x: " << field.x << " y: " << field.y << " z: " << field.z << endl;
//     usleep(500000);
//   }
//   return true;
// }
//
//
// bool testRGB() {
//   uint8_t address = 0x40;
//   uint8_t ID_register = 1;  // We do not know what the actual register is, but
//   // it doesn't matter for now because currently id registers are not used.
//   uint8_t ctlRegister1 = 0x00;
//   uint8_t ctlRegister2 = 0x01;
//   PWMcontroller rgb(file, address, ID_register, ctlRegister1, ctlRegister2);
//   if (!rgb.initRGB_PWMcontroller()) {
//     cerr << "Error: RGB PWM controller failed to initialize" << endl;
//     return false;
//   }
//   cout << "red!" << endl;
//   if (!setRGBColor(&rgb, 100, 0, 0)) {
//     cerr << "Error: Red color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "orange!" << endl;
//   if (!setRGBColor(&rgb, 100, 25, 0)) {
//     cerr << "Error: Orange color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "yellow!" << endl;
//   if (!setRGBColor(&rgb, 100, 50, 0)) {
//     cerr << "Error: Yellow color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "green!" << endl;
//   if (!setRGBColor(&rgb, 0, 100, 0)) {
//     cerr << "Error: Green color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "cyan!" << endl;
//   if (!setRGBColor(&rgb, 0, 100, 100)) {
//     cerr << "Error: Cyan color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "blue!" << endl;
//   if (!setRGBColor(&rgb, 0, 0, 100)) {
//     cerr << "Error: Blue color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "purple!" << endl;
//   if (!setRGBColor(&rgb, 50, 0, 100)) {
//     cerr << "Error: Purple color failed to set" << endl;
//     return false;
//   }
//   sleep(2);
//   cout << "off!" << endl;
//   if (!setRGBColor(&rgb, 0, 0, 0)) {
//     cerr << "Error: LED failed to turn off" << endl;
//     return false;
//   }
//   return true;
// }
//
// bool setRGBColor(PWMcontroller *rgb, int red, int green, int blue) {
//   const int RED = 2;
//   const int GREEN = 3;
//   const int BLUE = 4;
//   if (rgb->setChlPercent(RED, red)) {
//     if (rgb->setChlPercent(BLUE, blue)) {
//       if (rgb->setChlPercent(GREEN, green)) {
//         return true;
//       }
//     }
//   }
//   return false;
// }
