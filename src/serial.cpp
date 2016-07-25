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
 * \file       serial.cpp
 * \brief      driver for serial port communication
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/serial.h"

#if defined(__linux__)
 #include <cstdio>
 #include <cstring>
 #include <stdint.h>
 #include <errno.h>
 #include <fcntl.h>
 #include <sys/ioctl.h>
 // Functions of unistd.h to be called with namespace: unistd::function()
 namespace unistd {
  #include <unistd.h>
 }
#if defined(__DEBUG__)
  #include <iostream>
#endif

#endif // defined(__linux__)
static const int bits   = 0;
static const int parity = 1;
static const int stop   = 2;

blob::SerialPort::SerialPort (const char* port, const uint32_t &baudrate, 
                              const char* config)
{
   _baudrate = baudrate;
   _port     = port;
   _config   = config;
   _status   = -1;
   _fd       = -1;
} // SerialPort::SerialPort

bool blob::SerialPort::init ()
{
#if defined(__linux__)
  // configuration options :
  // O_RDWR - read and write access
  // O_CTTY - prevent other input (like keyboard) from affecting what we read
  // O_NDELAY - Don't care if the other side is connected (some devices don't explicitly connect)
  _fd = open(_port, O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK);

  if(_fd < 0) {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error opening port: " << _port << std::endl;
#endif
    return false;
  }

  fcntl (_fd, F_SETFL, O_RDWR) ;

  // get the current settings
  tcgetattr(_fd, &_options);
  cfmakeraw(&_options);

  int baudEnum = 0;

  // set the read and write speed
  switch (_baudrate)
  {
    case     50: baudEnum =     B50; break;
    case     75: baudEnum =     B75; break;
    case    110: baudEnum =    B110; break;
    case    134: baudEnum =    B134; break;
    case    150: baudEnum =    B150; break;
    case    200: baudEnum =    B200; break;
    case    300: baudEnum =    B300; break;
    case    600: baudEnum =    B600; break;
    case   1200: baudEnum =   B1200; break;
    case   1800: baudEnum =   B1800; break;
    case   2400: baudEnum =   B2400; break;
    case   9600: baudEnum =   B9600; break;
    case  19200: baudEnum =  B19200; break;
    case  38400: baudEnum =  B38400; break;
    case  57600: baudEnum =  B57600; break;
    case 115200: baudEnum = B115200; break;
    case 230400: baudEnum = B230400; break;
    default: return false;
  }

  cfsetispeed(&_options, baudEnum);
  cfsetospeed(&_options, baudEnum);
   
  if (strlen(_config) != 3)  // make sure configuration has 3 characters
    return false;

  if (_config[parity]=='N')
  {                               
    _options.c_cflag &= ~PARENB; // disable the parity bit with ~PARENB
  }
  else if (_config[parity]=='E')
  {
    _options.c_cflag |= PARENB; // enable parity with PARENB
    _options.c_cflag &= ~PARODD; // even parity
  }
  else if (_config[parity]=='O')
  {
    _options.c_cflag |= PARENB; // enable parity with PARENB
    _options.c_cflag |= PARODD; // PARAODD enables odd parity
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error in parity bits: " << _config << std::endl;
#endif
    return false;
  }

  if(_config[stop]=='2')
  {
    _options.c_cflag |= CSTOPB; // CSTOPB means 2 stop bits
  }
  else if(_config[stop]=='1')
  {
    _options.c_cflag &= ~CSTOPB; // only one stop bit
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error in stop bits: " << _config << std::endl;
#endif
    return false;
  }
 
  // CSIZE is a mask for all data size bits
  _options.c_cflag &= ~CSIZE; // clear out current data size setting
  if(_config[bits]=='5') 
  {
    _options.c_cflag |= CS5; // CS5 means 5-bits per work
  }
  else if(_config[bits]=='6') 
  {
    _options.c_cflag |= CS6; // CS5 means 5-bits per work
  }  
  else if(_config[bits]=='7') 
  {
    _options.c_cflag |= CS7; // CS5 means 5-bits per work
  }
  else if(_config[bits]=='8') 
  {
    _options.c_cflag |= CS8; // CS5 means 5-bits per work
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error in number of bits: " << _config << std::endl;
#endif
    return false;
  }

  /* timeouts for non-blocking read */
  _options.c_cc[VMIN] = 0; // VMIN: min amount of characters to read
  _options.c_cc[VTIME] = _timeout; // time to wait for VMIN characters in tenth of sec.

  // CLOCAL: don't allow control of the port to be changed
  // CREAD: enable the receiver
  _options.c_cflag |= (CLOCAL | CREAD);
  _options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  _options.c_oflag &= ~OPOST;

  // apply the settings to the serial port:
  // - TCSNOW: apply changes now
  // - TCSADRAIN: wait until everything has been transmitted
  // - TCSAFLUSH - flush buffers and apply changes

  if(tcsetattr(_fd, TCSANOW, &_options) < 0) {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error setting port configuration" << std::endl;
#endif
    return false;
  }

  ioctl (_fd, TIOCMGET, &_status);
  _status |= TIOCM_DTR;
  _status |= TIOCM_RTS;
  ioctl (_fd, TIOCMSET, &_status);

  unistd::usleep (10000);

#elif defined(__AVR_ATmega32U4__)

  if (strcmp(_config,"8N1")==0)
    Serial.begin(_baudrate);
  else 
    return false;

  // wait for Leonardo enumeration, others continue immediately
  while (!Serial);
    
  _fd = 1;

#endif // defined(__linux__)

  return true;
} // SerialPort::init

int blob::SerialPort::available ()
{
  if(!isReady())
    return -1;

#if defined(__linux__)
  int retval;
  int error;
  int bytes_avail = 0;
  
  if((error=ioctl(_fd, FIONREAD, &bytes_avail)) < 0)
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::available] Error (" << error << " in ioctl request for fd " << _fd << std::endl;
#endif
    retval = -1;
  }
  else
  {
    retval = bytes_avail;
  }
  return bytes_avail;

#elif defined(__AVR_ATmega32U4__)
  return Serial.available();
#endif // defined(__linux__)

} // SerialPort::available

bool blob::SerialPort::end ()
{
  if(!isReady())
    return false;

// close serial port
#if defined(__linux__)
  return unistd::close(_fd) == 0? true:false;
#elif defined(__AVR_ATmega32U4__)
  Serial.end();
  return true; 
#endif // defined(__linux__)

} // SerialPort::end

void blob::SerialPort::flush ()
{
  if(!isReady())
    return;

#if defined(__linux__)
  char c;
  while(available() && unistd::read(_fd,&c,1) > 0);
#elif defined(__AVR_ATmega32U4__)
  Serial.flush(); 
#endif // defined(__linux__)

} // SerialPort::flush

bool blob::SerialPort::peek (byte& b)
{
  if(!isReady())
    return false;

#if defined(__linux__)
  // We obtain a pointer to FILE structure from the file descriptor sd
  FILE * fp = NULL;
  char c = 0;
  if ((fp = fdopen(_fd,"r+")) != NULL)
  {
    c = getc(fp);
    ungetc(c, fp);
#if defined(__DEBUG__)
  std::cout << "p0x" << std::hex << (int)c << std::dec << std::endl;
#endif // defined(__DEBUG__)
    b = c;    
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::peek] Error in opening port as a file" << std::endl;
#endif
    return false;
   }

#elif defined(__AVR_ATmega32U4__)
  int res = Serial.peek();
  if(res != -1)
    b = *(byte*)&res;
  else
    return false; 
#endif // defined(__linux__)

  return true;
} // SerialPort::peek

bool blob::SerialPort::read (byte& b)
{
  if(!isReady())
    return false;

#if defined(__linux__)
  uint8_t res;

  if(unistd::read(_fd, &res, 1) < 0)
    return false;

 #if defined(__DEBUG__)
  std::cout << "r0x" << std::hex << (int)res << std::dec << std::endl;
 #endif // defined(__DEBUG__)
  b = *(byte*)&res;

#elif defined(__AVR_ATmega32U4__)
  int res = Serial.read();
  if(res != -1)
    b = *(byte*)&res;
  else
    return false;
#endif // defined(__linux__)

  return true;
} // SerialPort::read

int blob::SerialPort::read (const uint16_t& length, byte *buffer)
{
  if(!isReady())
    return -1;

#if defined(__linux__)
 #if defined(__DEBUG__)
  int retval = unistd::read(_fd, buffer, length);
  for (int i = 0; i < length; i++)
    std::cout << "r0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
  return retval;
 #else
  return unistd::read(_fd, (char*)buffer, length);
 #endif // defined(__DEBUG__)

#elif defined(__AVR_ATmega32U4__)
  return Serial.readBytes((char*)buffer, length); 
#endif // defined(__linux__)

} // SerialPort::readBytes

int blob::SerialPort::write (const uint16_t& length, const byte *buffer)
{
  if(!isReady())
    return -1;
#if defined(__linux__)
 #if defined(__DEBUG__)
  for (int i = 0; i < length; i++)
    std::cout << "w0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
 #endif // defined(__DEBUG__)
  return unistd::write(_fd, (uint8_t*)buffer, length);

#elif defined(__AVR_ATmega32U4__)
  return Serial.write(buffer, length); 
#endif // defined(__linux__)

} // SerialPort::write

bool blob::SerialPort::setTimeout(const uint32_t& ms)
{
  if(!isReady())
    return false;

#if defined(__linux__)
  uint8_t timeout = 1;

  // 0.1 secs up to 25.5 secs
  if (ms > 0 && ms < 100)
    timeout = 1;
  else if (ms > 25500)
    timeout = 255;
  else
    timeout = (uint8_t)(ms/100);

  // get the current settings
  tcgetattr(_fd, &_options);
  _options.c_cc[VTIME] = _timeout;

   if(tcsetattr(_fd, TCSANOW, &_options) < 0)
   {
#if defined(__DEBUG__)
    std::cerr << "[blob::SerialPort::begin] Error setting port configuration" << std::endl;
#endif
     return false;
   }
   else
   {
     _timeout = (int)timeout*100;
   }

#elif defined(__AVR_ATmega32U4__)
  Serial.setTimeout(ms);
  _timeout = ms;
#endif // defined(__linux__)

  return true;

} // SerialPort::setTimeout

bool blob::SerialPort::isReady () 
{ 
#if defined(__DEBUG__)
  if(_fd < 0)
    std::cerr << "[blob::SerialPort::isReady] Error fd=" << _fd << std::endl;
#endif
  return (_fd >= 0); 
}

