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
 * \file       test_arduino.cpp
 * \brief      test for comms library
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/comms.h"
#include "blob/serial.h"

class TestReceiver : public blob::Comms::CommandReceiver
{
  public:
    virtual bool onSetupCmd (const uint32_t& mask) { _mask=mask; return true; }

    bool configured (uint32_t id) { return _mask & (uint32_t)1<<id;}

  private:
    uint32_t _mask;
};

blob::Vector3d<float> g, a, e, m;
float ts, press, temp, alt, roc;

blob::SerialPort serial;
blob::Comms  comms;
TestReceiver receiver;

long int period = 20;
long int last = 0;
long int dt = 0;

bool send = false;

void setup()
{
  serial.init();
  comms.connectTo(&serial);
  comms.subscribe(&receiver);
}

void loop()
{
  dt = millis()-last;
  if (dt >= period)
  { 
    last = millis();
/*    while(Serial.available())
    {
      byte b = Serial.read();
      Serial.write(&b,1);
    }
*/
    comms.receive();

    if (receiver.configured(blob::Comms::Time)) 
      comms.prepareTime(last);

    if (receiver.configured(blob::Comms::State))
      comms.prepareState(0xABCD);
  
    if (receiver.configured(blob::Comms::Imu))
    {
      const blob::Vector3d<float> inc (dt*0.001f,dt*0.001f,dt*0.001f);
      g+=inc; a+=inc;
      comms.prepareImu(g,a);
    }

    if (receiver.configured(blob::Comms::Mag))
    {
      const blob::Vector3d<float> inc (dt*0.01f,dt*0.01f,dt*0.01f);
      m+=inc;
      comms.prepareMag(m);
    }

    if (receiver.configured(blob::Comms::Ahrs))
    {
      const blob::Vector3d<float> inc (dt*0.02f,dt*0.02f,dt*0.02f);
      e+=inc;
      comms.prepareAhrs(e);
    }

    comms.send();
  }
}
