/********* blob robotics 2014 *********
 *  title: test_linux.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include <iostream>
#include "blob/comms.h"

int main(int argc, char* argv[])
{
  blob::Vector3d<float> e, a, v, p;
  std::string port = "/dev/ttyACM0";
  uint32_t time = 0;

  if(argc > 1)
    port = argv[1];

  blob::Comms comms(port);
  
  std::cout << "starting comms test at " << port << " ..." << std::endl;  
  comms.init();
  
  if (!comms.isReady())
  {
    std::cout << "comms init error... aborting" << std::endl;
    return -1;
  }

  std::cout << "sending Start message ..." << std::endl;
  comms.cmd(blob::Comms::Start);

  while (true) 
  {
    if(comms.receive() == true)
    {
      switch(comms.getMsgType())
      {
        case blob::Comms::Data:
          if(comms.getData(e, a, v, p))
          {
            std::cout << "Euler (" <<  e.x << ", " << e.y << ", " << e.z <<
                         ") Acc (" <<  a.x << ", " << a.y << ", " << a.z <<
                         ") Vel (" <<  v.x << ", " << v.y << ", " << v.z <<
                         ") Pos (" <<  p.x << ", " << p.y << ", " << p.z <<
                         ") time " << comms.getTimestamp() << std::endl;
            time = comms.getTimestamp();
          }
          break;

        case blob::Comms::Command:
          if (comms.getCmd (blob::Comms::Start))
          {
            std::cout << "Start!" <<std::endl;
            /*e = blob::Vector3d<float> (1.f,2.f,3.f);
            a = blob::Vector3d<float> (4.f,5.f,6.f);
            v = blob::Vector3d<float> (7.f,8.f,9.f);
            p = blob::Vector3d<float> (10.f,11.f,12.f);
            comms.send(e,a,v,p);
*/
          }
          break;

        default: break;
      }
    }
/*
    if(serial.available())
      std::cout << std::hex << serial.read() << std::dec << std::endl;
*/
    unistd::usleep(10000);

  }

  return 0;
}
