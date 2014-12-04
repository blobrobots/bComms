/********* blob robotics 2014 *********
 *  title: comms.cpp
 *  brief: communication manager
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/comms.h"

#if defined(__linux__)
  #include <stdio.h>
  #include <string.h>
  #include <iostream>
  #include <sys/time.h>
#endif

static const byte _sync[2]  = {0xFF, 0xFF};
static const uint8_t _crc_length    = 2; // bytes
static const uint8_t _header_length = 6; // bytes
static const uint8_t _state_length  = _header_length+50+_crc_length; // bytes
static const uint8_t _vel_length    = _header_length+18+_crc_length; // bytes
static const uint8_t _pos_length    = _header_length+20+_crc_length; // bytes
static const uint8_t _action_length = _header_length+1+_crc_length; // bytes
static const uint8_t _empty_length  = _header_length+0+_crc_length; // bytes

#if defined(__linux__)
  blob::Comms::Comms (std::string port) : Serial(port.c_str()) {}
#endif

void blob::Comms::init (int32_t baudrate) // FIXME treat retval and errors
{
  flushRx();
  flushTx();

  // initialize serial communication at 115200 bits per second:
  Serial.begin(baudrate);

#if defined(__AVR_ATmega32U4__)
  // wait for Leonardo enumeration, others continue immediately
  while (!Serial);
#endif
  _receiving = false;

} // Comms::init

bool blob::Comms::isReady () {
#if defined(__AVR_ATmega32U4__)
  return Serial;
#elif defined (__linux__)
  return Serial.isReady();
#else
  return false;
#endif

} // Comms::isReady

int16_t blob::Comms::calcCrc(byte *frame, size_t length)
{
  int16_t crc = 0;

  for(uint16_t i = 0; i < length; i++) {
    crc ^= (int16_t)frame[i];
  }
#if defined(__DEBUG__) && defined(__linux) 
  std::cout << " crc = " << crc << " from length = " << length << std::endl;
#endif
  return crc;
}

uint32_t blob::Comms::timestamp ()
{
#if defined(__AVR_ATmega32U4__)
  return millis();
#elif defined (__linux__)
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000*(uint32_t)(tv.tv_sec + 0.000001*tv.tv_usec);
#endif 
}

template <typename T> bool blob::Comms::push (T const& v)
{
  uint8_t inc = sizeof(T);
  bool retval = false;

  if((_itx + inc) < sizeof(_buffer))
  { 
    *(reinterpret_cast<T *>(&_buffer[_itx])) = v;
    _itx += inc;
    retval = true;
  }
  return retval;
}

template <typename T> T blob::Comms::pop ()
{
  uint8_t inc = sizeof(T);
  T retval = 0;

  if((_irx + inc) < sizeof(_received))
  { 
    retval = *(reinterpret_cast<T *>(&_received[_irx]));
    _irx += inc;
  }
  return retval;
}

bool blob::Comms::flushTx()
{
  _itx = 0;
  memset(_buffer,0,sizeof(_buffer));
  return true;
} 

bool blob::Comms::flushRx()
{
  _irx = 0;
  memset(_received,0,sizeof(_received));
  return true;
} 

bool blob::Comms::send (MsgSubType st)
{
  bool retval = false;

  if(st == Health)
  {
    flushTx();
    push<uint8_t>(Data);
    push<uint8_t>(st);
    push<float>(timestamp());
    push<int16_t>(calcCrc(_buffer, _itx));

    Serial.write((byte *) _sync, sizeof(_sync));
    Serial.write(_buffer, _itx);

    retval = true;
  }
  return retval;
} // Comms::send

bool blob::Comms::send (blob::Vector3d<float> e, blob::Vector3d<float> a, 
                        blob::Vector3d<float> v, blob::Vector3d<float> p)
{
  bool retval = true;

  flushTx();
  push<uint8_t>(Data);
  push<uint8_t>(State);
  push<float>(timestamp());
  push<uint8_t>(0); // flags
  push<uint8_t>(0); // mode
  push<float>(e.x); // roll
  push<float>(e.y); // pitch
  push<float>(e.z); // yaw
  push<float>(a.x); 
  push<float>(a.y);
  push<float>(a.z);
  push<float>(v.x);
  push<float>(v.y);
  push<float>(v.z);
  push<float>(p.x);
  push<float>(p.y);
  push<float>(p.z);
  push<int16_t>(calcCrc(_buffer, _itx));

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write(_buffer, _itx);

  return retval;
} // Comms::send

bool blob::Comms::cmd (MsgSubType st)
{
  bool retval = false;

  if(st == Start)
  {
    flushTx();
    push<uint8_t>(Command);
    push<uint8_t>(st);
    push<float>(timestamp());
    push<int16_t>(calcCrc(_buffer, _itx));

    Serial.write((byte *) _sync, sizeof(_sync));
    Serial.write(_buffer, _itx);

#if defined(__DEBUG__) && defined(__linux) 
//  std::cout << " crc = 0x" << std::hex << (int)(&_crc)[1] << (int)(&_crc)[0] << std::dec << std::endl;
#endif
  
    retval = true;
  }
  return retval;
} // Comms::cmd

bool blob::Comms::cmd (OnOff action)
{
  bool retval = false;

  flushTx();
  push<uint8_t>(Command);
  push<uint8_t>(Motors);
  push<float>(timestamp());
  push<uint8_t>(action);
  push<int16_t>(calcCrc(_buffer, _itx));

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write(_buffer, _itx);

  retval = true;

  return retval;
} // Comms::cmd


bool blob::Comms::cmd (InOut action)
{
  bool retval = false;

  flushTx();
  push<uint8_t>(Command);
  push<uint8_t>(Dock);
  push<float>(timestamp());
  push<uint8_t>(action);
  push<int16_t>(calcCrc(_buffer, _itx));

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write(_buffer, _itx);

  retval = true;

  return retval;
} // Comms::cmd

// FIXME check if values are within range of int 16 (aprox +- 32.2)
bool blob::Comms::cmd (blob::Vector3d<float> vel, float vyaw)
{
  bool retval = false;

  flushTx();
  push<uint8_t>(Command);
  push<uint8_t>(Vel);
  push<float>(timestamp());
  push<uint8_t>(0); // cmd priority
  push<float>(vel.x);
  push<float>(vel.y);
  push<float>(vel.z);
  push<float>(vyaw);
  push<int16_t>(calcCrc(_buffer, _itx));

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write(_buffer, _itx);

  retval = true;

  return retval;
} // Comms::cmd

// FIXME check if values are within range of int 16 (aprox +- 32.2)
bool blob::Comms::cmd (blob::Vector3d<float> pos, float heading, float speed)
{
  bool retval = false;

  flushTx();
  push<uint8_t>(Command);
  push<uint8_t>(GoTo);
  push<float>(timestamp());
  push<float>(pos.x);
  push<float>(pos.y);
  push<float>(pos.z);
  push<float>(heading);
  push<float>(speed);
  push<int16_t>(calcCrc(_buffer, _itx));

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write(_buffer, _itx);

  retval = true;

  return retval;
} // Comms::cmd

bool blob::Comms::sync ()
{
  bool retval = false;

  static int bytesRead = 0;

  while (Serial.available() > 0 && bytesRead < 8 && retval == false)
  {
    byte c = Serial.read();

    if (bytesRead == 1 && c == _sync[1])
    {
      bytesRead++;
    }
    else if (bytesRead == 2 && isMsgTypeValid(c))
    {
      _received[0] = c;
      bytesRead++;
    }
    else if (bytesRead == 3 && isMsgSubTypeValid(c))
    {
      _received[1] = c;
      bytesRead++;
      
    }
    else if (bytesRead > 3) // timestamp
    {
      _received[bytesRead-2] = c;
      bytesRead++;
      if(bytesRead == 8)
      { 
        retval = true;
        bytesRead = 0;
#if defined(__DEBUG__) && defined(__linux) 
        std::cout << " got message header: 0x" << std::hex << (int)_sync[0] << (int)_sync[1] << std::dec 
                  << " type: " << (int)_received[0] 
                  << " subtype: " << (int)_received[1]
                  << " timestamp: " << *(unsigned int *)&_received[2] 
                  << std::dec << std::endl;
#endif
      }
    }
    else if (c == _sync[0])
    {
      bytesRead = 1;
    }
    else
    {
      bytesRead = 0;
    }
  }
  return retval;
} // Comms::getHeader

bool blob::Comms::receive ()
{
  bool retval = false;

  if (_receiving == false)
  { // receive header
    _receiving = sync();
  }

  if (_receiving == true)
  { // retrieve body
    switch (_received[0]) // type
    {
      case Data:
        switch (_received[1]) // subtype
        {
          case Health:
            retval = retrieve(_empty_length-_header_length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Health " << _empty_length-_header_length << std::endl;
#endif
            break;
          case State:
            retval = retrieve(_state_length-_header_length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::State " << _state_length-_header_length << std::endl;
#endif
            break;
          default: break;
        }
        break;

      case Command:
        switch(_received[1]) // subtype
        {
          case Start:
            retval = retrieve(_empty_length-_header_length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Start " << _empty_length-_header_length << std::endl;
#endif
            break;
          case Motors:
            retval = retrieve(_action_length-_header_length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Motors " << _action_length-_header_length << std::endl;
#endif
            break;
          default: break;
        }
        break;

      case Request:
        break;
      case Reply:
        break;

      default: break;
    }
  }
  return retval;
} // Comms::receive

bool blob::Comms::retrieve (size_t length)
{
  bool retval = false;

  if (Serial.available() >= length)
  {
    // retrieve message including crc
    Serial.readBytes((char *)&_received[_header_length], length);
    int16_t receivedCrc = *reinterpret_cast<int16_t *>(&_received[_header_length+length-_crc_length]);
    int16_t frameCrc = calcCrc(_received, _header_length+length-_crc_length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " received crc: 0x" << std::hex << (int)(_received[_header_length+length-_crc_length+1]) 
                      << (int)(_received[_header_length+length-_crc_length]) << std::dec 
                      << " = " << receivedCrc 
                      << " shouldbe crc: " << frameCrc << std::endl;
#endif
    if (receivedCrc != frameCrc)
    {
      _type = blob::Comms::NotValid;
    }
    else
    {
      _irx = 0;
      _type      = static_cast<MsgType>(pop<uint8_t>());
      _subtype   = static_cast<MsgSubType>(pop<uint8_t>());
      _timestamp = pop<uint32_t>();

      retval = true;
    }
    _receiving = false;
  }
  return retval;
} // Comms::retrieve

bool blob::Comms::isMsgTypeValid (int8_t type)
{
  switch (type) {
    case Data:
    case Command:
    case Request:
      return true;
    break;
    default:
      return true;
  }
}

bool blob::Comms::isMsgSubTypeValid (int8_t subtype)
{
  switch (subtype) {
    case Health:
    case State:
    case Start:
    case Motors:
    case Dock:
      return true;
    break;
    default:
      return true;
  }
}

bool blob::Comms::getData (Vector3d<float> &euler, Vector3d<float> &acc, Vector3d<float> &vel, Vector3d<float> &pos)
{
  bool retval = false;

  if(_receiving == false && _type == Data && _subtype == State)
  {
    _irx = _header_length;
    uint8_t flags = pop<uint8_t>();
    uint8_t mode  = pop<uint8_t>();
    euler.x = pop<float>();
    euler.y = pop<float>();
    euler.z = pop<float>();
    acc.x  = pop<float>();
    acc.y  = pop<float>();
    acc.z  = pop<float>();
    vel.x  = pop<float>();
    vel.y  = pop<float>();
    vel.z  = pop<float>();
    pos.x  = pop<float>();
    pos.y  = pop<float>();
    pos.z  = pop<float>();
    retval = true;
  }
  return retval;
} // Comms::getData

bool blob::Comms::getCmd (MsgSubType subtype)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == subtype)
  {
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (OnOff &action)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == Motors)
  {
    _irx = _header_length;
    action = static_cast<OnOff>(pop<uint8_t>());
    retval = true;
  }
  return retval;
} // Comms::getCmd


bool blob::Comms::getCmd (InOut &action)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == Dock)
  {
    _irx = _header_length;
    action = static_cast<InOut>(pop<uint8_t>());
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &vel, float &vyaw)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == Vel)
  {
    _irx = _header_length;
    uint8_t prio = pop<uint8_t>();
    vel.x = pop<float>();
    vel.y = pop<float>();
    vel.z = pop<float>();
    vyaw  = pop<float>();
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &pos, float &heading, float &speed)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == GoTo)
  {
    _irx = _header_length;
    pos.x = pop<float>();
    pos.y = pop<float>();
    pos.z = pop<float>();
    heading = pop<float>();
    speed   = pop<float>();
    retval = true;
  }
  return retval;
} // Comms::getCmd
