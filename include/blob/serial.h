/********* blob robotics 2014 *********
 *  title: serial.h
 *  brief: interface for Serial port
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_SERIAL_H
#define B_SERIAL_H

#if defined(__linux__)

#include <cstdio>
#include <string>
#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// Functions of unistd.h to be called with namespace: unistd::function()
namespace unistd {
  #include <unistd.h>
}

namespace blob {

class Serial
{
  public:
    enum Config { SERIAL_5N1, SERIAL_6N1, SERIAL_7N1, SERIAL_8N1,
                  SERIAL_5N2, SERIAL_6N2, SERIAL_7N2, SERIAL_8N2,
                  SERIAL_5E1, SERIAL_6E1, SERIAL_7E1, SERIAL_8E1,
                  SERIAL_5E2, SERIAL_6E2, SERIAL_7E2, SERIAL_8E2,
                  SERIAL_5O1, SERIAL_6O1, SERIAL_7O1, SERIAL_8O1,
                  SERIAL_5O2, SERIAL_6O2, SERIAL_7O2, SERIAL_8O2 };

    Serial (std::string port = "/dev/ttyUSB0");

    int  begin      (unsigned int baudrate = 115200, Config cfg = SERIAL_8N1);
    int  available  ();
    int  end        ();
    void flush      ();
    int  peek       ();
    int  read       ();
    int  readBytes  (char *buffer, uint16_t length);
    int  write      (uint8_t *buffer, uint16_t length);
    int  setTimeout (long ms);
    bool isReady    ();

  private:
    unsigned int   _baudrate;
    std::string    _port;
    Config         _config;
    struct termios _options;
    int            _fd;
    int            _status;
    uint8_t        _timeout; // 0.1 secs up to 25.5 secs
};

}

#endif /* defined(__linux__) */
#endif /* B_SERIAL_H */
