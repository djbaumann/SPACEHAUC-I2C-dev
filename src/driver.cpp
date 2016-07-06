// Copyright 2016 UMass Lowell Command and Data Handling Team

#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include "../include/spacehauc-i2c-dev.h"

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;

bool testTemperatureSensor(int file);
bool testMagnetometer(int file);
bool testLuminositySensor(int file);

int main(int argc, char* argv[]) {
  cout << "SPACEHAUC I2C Library Driver" << endl;
  cout << "Initializing Bus" << endl;
  int file;
  int bus = 1;  // the first i2c bus
  if (initBus(bus, &file) == false) {
    cerr << "Error: I2C bus failed to open." << endl;
  }
  cout << "Testing Temperature Sensor..." << endl;
  if (testTemperatureSensor(file)) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  cout << "Testing Magnetometer..." << endl;
  if (testMagnetometer(file)) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  cout << "Testing Luminosity Sensor..." << endl;
  if (testLuminositySensor(file)) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  return 0;
}

bool testTemperatureSensor(int file) {
  uint8_t address = 0x1d;
  uint8_t ID_register = 0x0F;
  uint8_t ctlRegister = 0x24;
  uint8_t dataRegister = 0x05;
  TemperatureSensor tempSensor(file, address, ID_register, ctlRegister,
    dataRegister);
  if (tempSensor.initTempSensor() == false) {
    cerr << "Error: Temperature Sensor failed to initalize." << endl;
    return false;
  }
  cout << "Initialized Temperature Sensor" << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << "Temperature = " << static_cast<int>(tempSensor.readTemp()) << endl;
    usleep(500000);
  }
  return true;
}

bool testMagnetometer(int file) {
  uint8_t address = 0x1d;
  uint8_t ID_register = 0x0F;
  uint8_t ctlRegister1 = 0x25;
  uint8_t ctlRegister2 = 0x26;
  uint8_t dataRegister = 0x08;
  Magnetometer magnetometer(file, address, ID_register, ctlRegister1,
    ctlRegister2, dataRegister, MAG_SCALE_2GS);
  if (magnetometer.initMagnetometer() == false) {
    cerr << "Error: Magnetometer failed to initalize." << endl;
    return false;
  }
  cout << "Initialized Magnetometer" << endl;
  cout << "Reading Magnetic Field Data..." << endl;
  for (int i = 0; i < 5; ++i) {
    fTriplet field = magnetometer.readMagnetometer();
    cout << "x: " << field.x << " y: " << field.y << " z: " << field.z << endl;
    usleep(500000);
  }
  return true;
}

bool testLuminositySensor(int file) {
  uint8_t address = 0x39;
  uint8_t ID_register = 0x0A;
  uint8_t ctlRegister1 = 0x00;
  uint8_t ctlRegister2 = 0x01;
  uint8_t dataRegister = 0x0C;
  LuminositySensor light(file, address, ID_register, ctlRegister1, ctlRegister2,
    dataRegister);
  if (light.initLuminositySensor() == false) {
    cerr << "Error: Luminosity Sensor failed to initialize." << endl;
    return false;
  }
  cout << "Initialized Luminosity Sensor" << endl;
  cout << "Reading light intensity data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << "Luminosity: " << light.readLuminositySensor() << endl;
    sleep(1);
  }
  return true;
}