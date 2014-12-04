/********* blob robotics 2014 *********
 *  title: test_arduino.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include "Arduino.h"
#include "blob/comms.h"


blob::Vector3d<float> e, a, v, p;
blob::Comms comms;
long int period = 20;
long int last = 0;
long int dt = 0;

bool send = false;

void setup()
{  
  comms.init();
}

void loop()
{
  dt = millis()-last;
  if (dt >= period)
  { 
    last = millis();
/*    if(Serial.available() >= 10)
    {
      delay(1000);
      byte buff[10];
      Serial.readBytes((char *)buff,10);
      Serial.write(buff,10);
    }*/
    if (comms.receive() == true)
    {
      switch (comms.getMsgType())
      {
        case blob::Comms::Command:
          if (comms.getCmd (blob::Comms::Start))
          {
            send = true;
          }
          break;
        default: break;
      }
    }

    if (send)
    { 
      const blob::Vector3d<float> inc (dt*0.001f,dt*0.001f,dt*0.001f);
      e+=inc; a+=inc; v+=inc; p+=inc;
      comms.send(e,a,v,p);
    } 
  }
}
