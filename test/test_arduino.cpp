/********* blob robotics 2014 *********
 *  title: test_arduino.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include "Arduino.h"
#include "bComms.h"

blob::Vector3d<float> e, a, v, p;
blob::Comms comms;

bool send = false;

void setup()
{  
  comms.init();
}

void loop()
{
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
    const blob::Vector3d<float> inc (1.f,1.f,1.f);
    e+=inc; a+=inc; v+=inc; p+=inc;
    comms.send(e,a,v,p);
  }  
  delay(10);
}
