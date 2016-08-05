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
 * \file       comms.cpp
 * \brief      communication manager
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/comms.h"

#if defined(__linux__)
  #include <stdio.h>
  #include <string.h>
  #include <iostream>
  #include <sys/time.h>
#endif

static const uint8_t _hdr_size    = 2; // bytes
static const uint8_t _crc_size    = 2; // bytes
static const uint8_t _cmd_size[4] = {4, 1, 1, 16}; // bytes

blob::Comms::Comms()
{
  _dataMask = 0;
  memset(_received,0,BCOMMS_MAX_LENGTH);
  memset(_buffer,0,BCOMMS_MAX_LENGTH);
  _irx = _itx = 0;
  _channel = NULL; 
  _dataListener = NULL;
  _cmdListener = NULL;
}

bool blob::Comms::connectTo (Channel* channel)
{
  if(channel == NULL)
    return false;

  if(_channel)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::connect() error: already connected to channel with handler " 
              << _channel << std::endl;
#endif
    return false;
  }

  if(!channel->isReady())
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::connect() error: channel not ready to connect" << std::endl;
#endif
    return false;
  }

  flushRx();
  flushTx();
  _receiving = false;
  _channel = channel;

  return true;
} // Comms::connect

bool blob::Comms::disconnectFrom (Channel* channel)
{
  if(channel == NULL || _channel == NULL)
    return false;

  if(channel != _channel)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::disconnect() error: channel handler " << channel << 
                 " is not connected (connected handler is " << _channel << 
                 ")" << std::endl;
#endif
    return false;
  }

  _channel = NULL;

  return true;
} // Comms::connect

bool blob::Comms::isReady () {
  if(_channel == NULL)
    return false;

  return _channel->isReady();

} // Comms::isReady

bool blob::Comms::checksum()
{
  byte c0, c1;
  calcCrc(_rxLength, _rxBuffer, c0, c1 );
  
  return (c0==_rxBuffer[_rxLength-2] && c1==_rxBuffer[_rxLength-1]);

} // Comms::checksum

bool blob::Comms::calcCrc(const size_t& length, const byte *frame, 
                                                byte& c0, byte& c1 )
{
  if(length == 0 || frame ==NULL)
    return false;

  c0 = c1 = 0;

  for(int i=0; i<length;i++)
  {
    c0 += frame[i];
    c1 += c0;
  }

#if defined(__DEBUG__) && defined(__linux) 
  std::cout << "Comms::calcCrc() crc (0x[c1][c0]) = 0x" << std::hex 
            << (int)c1 << (int)c0 << std::dec << " from length = " << length << std::endl;
#endif

  return true;
}

template <typename T> bool blob::Comms::push (T const& v, const uint8_t& length,
                                                  byte* buffer, uint8_t& index)
{
  uint8_t inc = sizeof(T);
  bool retval = false;

  if((index + inc) < length)
  { 
    *(reinterpret_cast<T *>(&buffer[index])) = v;
    index += inc;
    retval = true;
  }
  return retval;
}

template <typename T> bool blob::Comms::push (T const& v)
{
  return push<T>(v, sizeof(_txBuffer), _txBuffer, _itx);
}

template <typename T> T blob::Comms::pop ()
{
  uint8_t inc = sizeof(T);
  T retval = 0;

  if((_irx + inc) < sizeof(_rxBuffer))
  { 
    retval = *(reinterpret_cast<T *>(&_received[_irx]));
    _irx += inc;
  }
  return retval;
}

bool blob::Comms::flushTx()
{
  _itx = 0;
  _dataMask=0;
  memset(_buffer,0,sizeof(_buffer));
  return true;
} 

bool blob::Comms::flushRx()
{
  _irx = 0;
  _dataMask=0;
  memset(_received,0,sizeof(_received));
  return true;
} 

bool blob::Comms::subscribe (DataReceiver* listener)
{
  if(listener == NULL)
    return false;

  if(_dataListener)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::register() error: already connected to listener with handler " 
              << _dataListener << std::endl;
#endif
    return false;
  }

  _dataListener = listener;

  return true;
}

bool blob::Comms::unsubscribe (DataReceiver* listener)
{
  if(listener == NULL || _dataListener == NULL)
    return false;

  if(listener != _dataListener)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::register() error: listener handler " << listener << 
                 " is not connected (connected handler is " << _dataListener << 
                 ")" << std::endl;
#endif
    return false;
  }

  _dataListener = NULL;

  return true;
}

bool blob::Comms::subscribe (CommandReceiver* listener)
{
  if(listener == NULL)
    return false;

  if(_cmdListener)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::register() error: already connected to listener with handler " 
              << _cmdListener << std::endl;
#endif
    return false;
  }

  _cmdListener = listener;

  return true;
}

bool blob::Comms::unsubscribe (CommandReceiver* listener)
{
  if(listener == NULL || _cmdListener == NULL)
    return false;

  if(listener != _cmdListener)
  {
#if defined(__DEBUG__) && defined(__linux) 
    std::cerr << "Comms::register() error: listener handler " << listener << 
                 " is not connected (connected handler is " << _cmdListener << 
                 ")" << std::endl;
#endif
    return false;
  }

  _cmdListener = NULL;

  return true;
}

bool blob::Comms::prepareTime (const uint32_t& ts)
{
  if(!((uint32_t)State)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Time)==false ||
      push<uint32_t> (ts)==false )
    return false;
  
  return true;
} // Comms::prepareTime

bool blob::Comms::prepareState (const uint16_t& state)
{
  if(!((uint32_t)State)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t> (State)==false ||
      push<uint16_t>(state)==false )
    return false;
  
  return true;
} // Comms::prepareState

bool blob::Comms::prepareMotors (const uint8_t& num, const uint16_t* motors)
{
  if(!((uint32_t)Motors)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Motors)==false ||
      push<uint8_t>(num)   ==false )
    return false;
  
  for(int i=0; i<num; i++)
  {
    if(push<uint16_t>(motors[i])==false)
      return false;
  }

  return true;
} // Comms::prepareMotors

bool blob::Comms::prepareImu (const Vector3d<float>& gyro, 
                  	          const Vector3d<float>& acc)
{
  if(!((uint32_t)Imu)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Imu) ==false ||
      push<float>(gyro.x)==false ||
      push<float>(gyro.y)==false ||
      push<float>(gyro.z)==false ||
      push<float>(acc.x) ==false ||
      push<float>(acc.y) ==false ||
      push<float>(acc.z) ==false )
    return false;

  return true;
} // Comms::prepareImu

bool blob::Comms::prepareMag (const Vector3d<float>& mag)
{
  if(!((uint32_t)Mag)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Mag)==false ||
      push<float>(mag.x)==false ||
      push<float>(mag.y)==false ||
      push<float>(mag.z)==false )
    return false;

  return true;
} // Comms::prepareMag

bool blob::Comms::prepareAhrs (const Vector3d<float>& euler)
{
  if(!((uint32_t)Ahrs)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Ahrs) ==false ||
      push<float>(euler.x)==false ||
      push<float>(euler.y)==false ||
      push<float>(euler.z)==false )
    return false;

  return true;
} // Comms::prepareAhrs

bool blob::Comms::prepareBaro (const float& pressure, const float& temperature, 
                  		         const float& altitude, const float& roc)
{
  if(!((uint32_t)Imu)&_dataMask)
    return false;
  
  if(!_itx)                           // if buffer is empty
    if (push<uint8_t>(Data)==false || // message type
        push<uint8_t>(0)   ==false )  // temporary size
      return false;
  
  if (push<uint8_t>(Baro)     ==false ||
      push<float>(pressure)   ==false ||
      push<float>(temperature)==false ||
      push<float>(altitude)   ==false ||
      push<float>(roc)        ==false )
    return false;

  return true;
} // Comms::prepareBaro

bool blob::Comms::send ()
{
  if(!_itx)
    return false;

  byte c0=0, c1=0;    // checksum bytes

  // update data size (used instead of msg subtype)
  _buffer[1] = _itx-2; // data size

  // add checksum bytes
  if(calcCrc(_itx, _buffer, c0, c1)==false)
  {
    flushTx();
    return false;
  }

  if (push<byte>(c0) == false || 
      push<byte>(c1) == false )
  {
    flushTx();
    return false;
  }

  _channel->write(sizeof(_sync), _sync);
  _channel->write(_itx, _buffer);

  // reset buffer
  flushTx();

  return true;
} // Comms::send

bool blob::Comms::cmdSetup (const uint8_t& num, ...)
{
  uint8_t i = 0;   // index
  byte b[10];      // cmd tx buffer
  byte c0=0, c1=0; // cmd checksum bytes
  uint32_t mask = 0;

  // prepare message
  va_list arguments;                     // A place to store the list of arguments
  
  va_start ( arguments, num ); // initializes arguments to store args. after num
  for ( i=0; i<num; i++ )      // loop until all args are added to mask
    mask |= (uint32_t)1<<va_arg(arguments, uint32_t); // adds next arg to mask      
  va_end ( arguments );        // cleans up the list

  i=0;
  if (push<uint8_t>(Command, 10, b, i)==false ||
      push<uint8_t>(Setup, 10, b, i)  ==false ||
      push<uint32_t>(mask, 10, b, i)  ==false )
    return false;
  
  if(calcCrc(i, b, c0, c1) == false)
    return false;

  if (push<byte>(c0, 10, b, i) == false || 
      push<byte>(c1, 10, b, i) == false )
    return false;
  
  _channel->write(sizeof(_sync), _sync);
  _channel->write(i, b);
  
  return true;
} // Comms::cmd

bool blob::Comms::cmdOnOff (const uint8_t& action)
{
  if(action > On) return false;

  uint8_t i = 0;   // cmd tx index
  byte b[9];       // cmd tx buffer
  byte c0=0, c1=0; // cmd checksum bytes

  // prepare message
  if (push<uint8_t>(Command, 9, b, i)== false ||
      push<uint8_t>(OnOff, 9, b, i)  == false ||
      push<uint8_t>(action, 9, b, i) == false )
    return false;
  
  if(calcCrc(i, b, c0, c1) == false)
    return false;

  if (push<byte>(c0, 9, b, i) == false || 
      push<byte>(c1, 9, b, i) == false )
    return false;
  
  _channel->write(sizeof(_sync), _sync);
  _channel->write(i, b);
  
  return true;
} // Comms::cmd

bool blob::Comms::cmdDock (const uint8_t& action)
{
  if(action > In) return false;

  uint8_t i = 0;   // cmd tx index
  byte b[9];       // cmd tx buffer
  byte c0=0, c1=0; // cmd checksum bytes

  // prepare message
  if (push<uint8_t>(Command, 9, b, i)== false ||
      push<uint8_t>(Dock, 9, b, i)   == false ||
      push<uint8_t>(action, 9, b, i) == false )
    return false;
  
  if(calcCrc(i, b, c0, c1) == false)
    return false;

  if (push<byte>(c0, 9, b, i) == false || 
      push<byte>(c1, 9, b, i) == false )
    return false;
  
  _channel->write(sizeof(_sync), _sync);
  _channel->write(i, b);
  
  return true;
} // Comms::cmd

bool blob::Comms::cmdMove (const Vector3d<float>& movement,const float& heading)
{
  uint8_t i = 0;   // cmd tx index
  byte b[24];      // cmd tx buffer
  byte c0=0, c1=0; // cmd checksum bytes

  // prepare message
  if (push<uint8_t> (Command, 24, b, i)   == false ||
      push<uint8_t> (Move, 24, b, i)      == false ||
      push<float>   (movement.x, 24, b, i)== false ||
      push<float>   (movement.y, 24, b, i)== false ||
      push<float>   (movement.z, 24, b, i)== false ||
      push<float>   (heading, 24, b, i)   == false )
    return false;
  
  if(calcCrc(i, b, c0, c1) == false)
    return false;

  if (push<byte>(c0, 24, b, i) == false || 
      push<byte>(c1, 24, b, i) == false )
    return false;
  
  _channel->write(sizeof(_sync), _sync);
  _channel->write(i, b);
  
  return true;
} // Comms::cmd

bool blob::Comms::read ()
{
  bool retval = false;
  static bool escaping = false;

  if(!_channel)
    return false;

  while (_channel->available() > 0 && retval == false)
  {
    byte c;
    _channel->read(c);

    if(escaping)
    {
      escaping=false;        
      if(c==DLE || c==ETX || c==SOH)
      {
        if(_receiving)
          _received[_rxLength++] = c; // DLE DLE case
      }
      else
        resetRx();
    }
    else if (c==DLE) // escape character found
    {
      escaping=true;
    }
    else if (!_receiving && c==SOH)
    {
      _receiving=true;
    }
    else if (_receiving && _rxLength<BCOMMS_MAX_LENGTH)
    {
      if(c==ETX)
      {
        _receiving = false;
        retval = true;
      }
      else
        _received[_rxLength++] = c;
    }
    else
    {
      resetRx();
    }
  }
  return retval;

} // Comms::read

void blob::Comms::resetRx ()
{
  _receiving = false;
  _rxLength  = false;
}

bool blob::Comms::receive ()
{
  bool retval = false;

  if (read() && checksum()) // retrieve body
  {
    switch (_received[0]) // type
    {
      case Data:
        parse(_received[1]);
        break;

      case Command:
        parse(_cmd_size[_received[1]]); // subtype length
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

bool blob::Comms::parse (const size_t& length)
{
  bool retval = false;

  if (_channel->available() >= length+_crc_size)
  {
    byte c0, c1, rc0, rc1;

    // retrieve message including crc
    _channel->read(length+_crc_size, &_received[_hdr_size]);

    rc0 = _received[_hdr_size+length];
    rc1 = _received[_hdr_size+length+1];
    
    calcCrc(_hdr_size+length, _received, c0, c1);

#if defined(__DEBUG__) && defined(__linux) 
    std::cout << " received crc: 0x" << std::hex << (int)rc1 << (int)rc0 
              << " shouldbe crc: 0x" << (int)c1 << (int)c0 << std::dec 
              << std::endl;
#endif

    if ((rc0 == c0)&&(rc1 == c1))
    {
      processData ();
      processCmd ();
//    processReq();
//    processResp();

      retval = true;
    }

    _receiving = false;
  }
  return retval;
} // Comms::retrieve

bool blob::Comms::processData ()
{
  if(_received[0] != Data || !_dataListener)
    return false;

  _irx = 0;
     
  MsgType type  = static_cast<MsgType>(pop<uint8_t>());
  size_t length = pop<uint8_t>();
  
  while (_irx < length)
  {
    uint8_t subtype = pop<uint8_t>();
    switch (subtype) {
      case Time:
      {
        uint32_t time = pop<uint32_t>();
        _dataListener->onTimeData(time);
        break;
      }
      case State:
      {
        uint16_t state = pop<uint16_t>();
        _dataListener->onStateData(state);
        break;
      }
      case Motors:
      {
        uint16_t motors[8];
        uint8_t n = pop<uint8_t>();
        for(int i=0; i<n; i++)
          motors[i] = pop<uint16_t>();
        _dataListener->onMotorData(n, motors);
        break;
      }
      case Imu:
      {
        Vector3d<float> g, a;
        g.x = pop<float>();
        g.y = pop<float>();
        g.z = pop<float>();
        a.x = pop<float>();
        a.y = pop<float>();
        a.z = pop<float>();
        _dataListener->onImuData(g,a);
        break;
      }
      case Mag:
      {
        Vector3d<float> m;
        m.x = pop<float>();
        m.y = pop<float>();
        m.z = pop<float>();
        _dataListener->onMagData(m);
        break;
      }
      case Ahrs:
      {
        Vector3d<float> e;
        e.x = pop<float>();
        e.y = pop<float>();
        e.z = pop<float>();
        _dataListener->onAhrsData(e);
        break;
      }
      case Baro:
      {
        float pres = pop<float>();
        float temp = pop<float>();
        float alt  = pop<float>();
        float roc = pop<float>();
        _dataListener->onBaroData(pres,temp,alt,roc);
        break;
      }
      default: 
        return false;
        break;
    }
  }
  return true;
}

bool blob::Comms::processCmd ()
{
  if(_received[0] != Command || !_cmdListener)
    return false;

  _irx = 0;
     
  MsgType type  = static_cast<MsgType>(pop<uint8_t>());
  uint8_t subtype = pop<uint8_t>();

  switch (subtype) {
    case Setup:
    {
      uint32_t setup = pop<uint32_t>();
      _cmdListener->onSetupCmd(setup);
      break;
    }
    case OnOff:
    {
      uint8_t action = pop<uint8_t>();
      _cmdListener->onOnOffCmd(action);
      break;
    }
    case Dock:
    {
      uint8_t action = pop<uint8_t>();
      _cmdListener->onDockCmd(action);
      break;
    }
    case Move:
    {
      Vector3d<float> move;
      move.x = pop<float>();
      move.y = pop<float>();
      move.z = pop<float>();
      float heading = pop<float>();
      _cmdListener->onMoveCmd(move,heading);
      break;
    }
    default: 
      return false;
      break;
  }
  return true;
}


bool blob::Comms::isMsgTypeValid (const int8_t& type)
{
  return (type<=Command);
}

bool blob::Comms::isMsgSubTypeValid (const int8_t& type, const int8_t& subtype)
{
  
  switch (type) {
    case Data:
      return (subtype<=Baro);
      break;
    case Command:
      return (subtype<=Move);
      break;
    default:
      return false;
  }
}
