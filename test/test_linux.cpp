/********* blob robotics 2014 *********
 *  title: test_linux.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include <iostream>
#include "bComms.h"

int main(int argc, char* argv[])
{
  std::string port = "/dev/ttyACM0";
  if(argc > 1)
    port = argv[1];
  
  std::cout << "starting comms test at " << port << " ..." << std::endl;
  
  blob::Comms comms(port);

  blob::Vector3d<float> e, a, v, p;

  std::cout << "sending Start message ..." << std::endl;
  comms.cmd(blob::Comms::Start);

  while (true) 
  {
    if(comms.receive() == true)
    {
      std::cout << "got " << comms.getMsgType() << " message" << std::endl;
  
      switch(comms.getMsgType())
      {
        case blob::Comms::Data:
          if(comms.getData(e, a, v, p))
          {
            std::cout << "Euler (" <<  e.x << ", " << e.y << ", " << e.z <<
                         ") Acc (" <<  a.x << ", " << a.y << ", " << a.z <<
                         ") Vel (" <<  v.x << ", " << v.y << ", " << v.z <<
                         ") Pos (" <<  p.x << ", " << p.y << ", " << p.z <<
                         std::endl;
          }
          break;

        default: break;
      }
    }
    unistd::usleep(10000);
  }
  return 0;
}
