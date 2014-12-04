/********* blob robotics 2014 *********
 *  title: serial.cpp
 *  brief: driver for generic I2C device
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/
#include "blob/serial.h"

#if defined(__linux__)

#if defined(__DEBUG__)
  #include <iostream>
#endif

blob::Serial::Serial (std::string port)
{
   _baudrate = 0;
   _port     = port;
   _config   = SERIAL_8N1;
   _status   = -1;
   _fd       = -1;
} // Serial::Serial

int blob::Serial::begin (uint32_t baudrate, Config cfg)
{
  int16_t retval = 0;
  // configuration options :
  // O_RDWR - read and write access
  // O_CTTY - prevent other input (like keyboard) from affecting what we read
  // O_NDELAY - Don't care if the other side is connected (some devices don't explicitly connect)
  _fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK);

  if(_fd < 0) {
#if defined(__DEBUG__)
    std::cerr << "[blob::Serial::begin] Error opening port: " << _port << std::endl;
#endif
    return -1;
  }

  fcntl (_fd, F_SETFL, O_RDWR) ;

  // get the current settings
  tcgetattr(_fd, &_options);
  cfmakeraw(&_options);

  int baudEnum = 0;

  // set the read and write speed
  switch (baudrate)
  {
    case     50: baudEnum =     B50; _baudrate = baudrate; break;
    case     75: baudEnum =     B75; _baudrate = baudrate; break;
    case    110: baudEnum =    B110; _baudrate = baudrate; break;
    case    134: baudEnum =    B134; _baudrate = baudrate; break;
    case    150: baudEnum =    B150; _baudrate = baudrate; break;
    case    200: baudEnum =    B200; _baudrate = baudrate; break;
    case    300: baudEnum =    B300; _baudrate = baudrate; break;
    case    600: baudEnum =    B600; _baudrate = baudrate; break;
    case   1200: baudEnum =   B1200; _baudrate = baudrate; break;
    case   1800: baudEnum =   B1800; _baudrate = baudrate; break;
    case   2400: baudEnum =   B2400; _baudrate = baudrate; break;
    case   9600: baudEnum =   B9600; _baudrate = baudrate; break;
    case  19200: baudEnum =  B19200; _baudrate = baudrate; break;
    case  38400: baudEnum =  B38400; _baudrate = baudrate; break;
    case  57600: baudEnum =  B57600; _baudrate = baudrate; break;
    case 115200: baudEnum = B115200; _baudrate = baudrate; break;
    case 230400: baudEnum = B230400; _baudrate = baudrate; break;
    default: _baudrate = 0; return -1; break;
  }

  cfsetispeed(&_options, baudEnum);
  cfsetospeed(&_options, baudEnum);

  if (cfg >= SERIAL_5N1 && cfg <= SERIAL_8N2)
  {
    // PARENB is enable parity bit
    _options.c_cflag &= ~PARENB; // disable the parity bit

    if(cfg >= SERIAL_5N2)
      _options.c_cflag |= CSTOPB; // CSTOPB means 2 stop bits
    else
      _options.c_cflag &= ~CSTOPB; // only one stop bit
  }
  else if (cfg >= SERIAL_5E1 && cfg <= SERIAL_8E2)
  {
    _options.c_cflag |= PARENB; // enable parity with PARENB

    _options.c_cflag &= ~PARODD; // even parity

    if(cfg >= SERIAL_5E2)
      _options.c_cflag |= CSTOPB; // CSTOPB means 2 stop bits
    else
      _options.c_cflag &= ~CSTOPB; // only one stop bit
  }
  else if (cfg >= SERIAL_5O1 && cfg <= SERIAL_8O2)
  {
    _options.c_cflag |= PARENB; // enable parity with PARENB

    _options.c_cflag |= PARODD; // PARAODD enables odd parity

    if(cfg >= SERIAL_5O2)
      _options.c_cflag |= CSTOPB; // CSTOPB means 2 stop bits
    else
      _options.c_cflag &= ~CSTOPB; // only one stop bit
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::Serial::begin] Error in parity/stop bits: " << cfg << std::endl;
#endif
    return -1;
  }

  // CSIZE is a mask for all data size bits
  _options.c_cflag &= ~CSIZE; // clear out current data size setting
  switch(cfg) {
    case SERIAL_5N1:
    case SERIAL_5N2:
    case SERIAL_5E1:
    case SERIAL_5E2:
    case SERIAL_5O1:
    case SERIAL_5O2:
      _options.c_cflag |= CS5; // CS5 means 5-bits per work
      break;
    case SERIAL_6N1:
    case SERIAL_6N2:
    case SERIAL_6E1:
    case SERIAL_6E2:
    case SERIAL_6O1:
    case SERIAL_6O2:
      _options.c_cflag |= CS6; // CS5 means 5-bits per work
      break;
    case SERIAL_7N1:
    case SERIAL_7N2:
    case SERIAL_7E1:
    case SERIAL_7E2:
    case SERIAL_7O1:
    case SERIAL_7O2:
      _options.c_cflag |= CS7; // CS5 means 5-bits per work
      break;
    case SERIAL_8N1:
    case SERIAL_8N2:
    case SERIAL_8E1:
    case SERIAL_8E2:
    case SERIAL_8O1:
    case SERIAL_8O2:
      _options.c_cflag |= CS8; // CS5 means 5-bits per work
      break;
    default:
#if defined(__DEBUG__)
      std::cerr << "[blob::Serial::begin] Error in number of bits: " << cfg << std::endl;
#endif
      return -1;
    break;
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
    std::cerr << "[blob::Serial::begin] Error setting port configuration" << std::endl;
#endif
    return -1;
  }

  ioctl (_fd, TIOCMGET, &_status);
  _status |= TIOCM_DTR;
  _status |= TIOCM_RTS;
  ioctl (_fd, TIOCMSET, &_status);

  unistd::usleep (10000);

  return _fd;
} // Serial::begin

int blob::Serial::available ()
{
  if(!isReady())
    return -1;

  int retval;
  int error;
  int bytes_avail = 0;
  
  if((error=ioctl(_fd, FIONREAD, &bytes_avail)) < 0)
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::Serial::available] Error (" << error << " in ioctl request for fd " << _fd << std::endl;
#endif
    retval = -1;
  }
  else
  {
    retval = bytes_avail;
  }
  return bytes_avail;

} // Serial::available

int blob::Serial::end ()
{
  if(!isReady())
    return -1;

  // close the serial port
  return unistd::close(_fd);

} // Serial::end

void blob::Serial::flush ()
{
  if(!isReady())
    return;

  char c;
  while(available() && unistd::read(_fd,&c,1) > 0);
} // Serial::flush

int blob::Serial::peek ()
{
  if(!isReady())
    return -1;

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
    return c;
  }
  else
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::Serial::peek] Error in opening port as a file" << std::endl;
#endif
    return -1;
   }
} // Serial::peek

int blob::Serial::read ()
{
  if(!isReady())
    return -1;

  uint8_t byte;
  int16_t retval = -1;

  if(unistd::read(_fd, &byte, 1) >= 0)
    retval = (int16_t)byte;

#if defined(__DEBUG__)
  std::cout << "r0x" << std::hex << (int)byte << std::dec << std::endl;
#endif // defined(__DEBUG__)
  
  return retval;

} // Serial::read

int blob::Serial::readBytes (char *buffer, uint16_t length)
{
  if(!isReady())
    return -1;
#if defined(__DEBUG__)
  int retval = unistd::read(_fd, buffer, length);
  for (uint16_t i = 0; i < length; i++)
    std::cout << "r0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
  return retval;
#else
  return unistd::read(_fd, buffer, length);
#endif // defined(__DEBUG__)

} // Serial::readBytes

int blob::Serial::write (uint8_t *buffer, uint16_t length)
{
  if(!isReady())
    return -1;
#if defined(__DEBUG__)
  for (uint16_t i = 0; i < length; i++)
    std::cout << "w0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
#endif // defined(__DEBUG__)
  return unistd::write(_fd, buffer, length);
} // Serial::write

int blob::Serial::setTimeout(long ms)
{
  if(!isReady())
    return -1;

  if (ms > 0 && ms < 100)
    _timeout = 1;
  else if (ms > 25500)
    _timeout = 255;
  else
    _timeout = (uint8_t)(ms/100);

  // get the current settings
  tcgetattr(_fd, &_options);
   _options.c_cc[VTIME] = _timeout;

   if(tcsetattr(_fd, TCSANOW, &_options) < 0)
   {
#if defined(__DEBUG__)
    std::cerr << "[blob::Serial::begin] Error setting port configuration" << std::endl;
#endif
     return -1;
   }
   else
   {
     return _timeout;
   }
} // Serial::setTimeout

bool blob::Serial::isReady () 
{ 
#if defined(__DEBUG__)
  if(_fd < 0)
    std::cerr << "[blob::Serial::isReady] Error fd=" << _fd << std::endl;
#endif
  return (_fd >= 0); 
}

#endif // defined(__linux__)
