/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Blob Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal 
 * in the Software without restriction, including without limitation the rights 
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
 * 
 * \file       test_linux.cpp
 * \brief      test for comms library
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/comms.h"
#include "blob/serial.h"
#include <iostream>
#include <unistd.h>

class TestReceiver : public blob::Comms::DataReceiver
{
  public:
    virtual bool onTimeData (const uint32_t& ts) {
      std::cout << "[comms] - time data (ms): " << ts << std::endl;
      return true;
    }

    virtual bool onStateData (const uint16_t& state) {
      std::cout << "[comms] - state data: " << std::hex << state 
                                            << std::dec << std::endl;
      return true;
    }

    virtual bool onImuData (const blob::Vector3d<float>& gyro, 
                            const blob::Vector3d<float>& acc) {
      std::cout << "[comms] - imu data: " << gyro.x << ", "
                << gyro.y << ", " << gyro.z << " (rad/s) " << std::endl
                << "                    " << acc.x << ", " << acc.y << ", " 
                << acc.z << " (m/s2)" << std::endl;
      return true;
    }

    virtual bool onMagData (const blob::Vector3d<float>& mag) {
      std::cout << "[comms] - mag data (T): " << mag.x << ", " << mag.y << ", " 
                << mag.z << std::endl;
      return true;
    }

    virtual bool onAhrsData (const blob::Vector3d<float>& euler) {
      std::cout << "[comms] - ahrs data (rad): " << euler.x << ", " << euler.y 
                << ", " << euler.z << std::endl;
      return true;
    }

    virtual bool onBaroData (const float& pressure, const float& temperature, 
                             const float& altitude, const float& roc) {
      std::cout << "[comms] - baro data: press(Pa) = " << pressure 
                << ", temp(C) = " << temperature << ", alt(m) = " << altitude 
                << ", roc(m/s) = " << roc << std::endl;
      return true;
    }
};

int main(int argc, char* argv[])
{
  blob::Vector3d<float> e, a, v, p;
  const char* port = "/dev/ttyACM0";
  uint32_t time = 0;

  if(argc > 1)
    port = argv[1];

  TestReceiver receiver;

  blob::SerialPort serial(port);
  blob::Comms comms;
    
  std::cout << "starting comms test at " << port << " ..." << std::endl;  
  serial.init();
  
  if (!serial.isReady())
  {
    std::cout << "comms init error... aborting" << std::endl;
    return -1;
  }

  // use serial as comms channel
  comms.connectTo(&serial);

  // register receiver as listener
  comms.subscribe(&receiver);

  std::cout << "sending Setup message ..." << std::endl;
  comms.cmdSetup(6, blob::Comms::Time,
                    blob::Comms::State,
                    blob::Comms::Imu,
                    blob::Comms::Mag,
                    blob::Comms::Ahrs,
                    blob::Comms::Baro);
  while (true) 
  {
    comms.receive();
    usleep(10000);
  }

  return 0;
}
