/********* blob robotics 2014 *********
 *  title: bComms.cpp
 *  brief: communication manager
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "bComms.h"

#if defined(__linux__)
  #include <stdio.h>
  #include <string.h>
  #include <iostream>
#endif

static const byte _sync[2] = {0xFF, 0xFF};

#if defined(__linux__)
  blob::Comms::Comms (std::string port) : Serial(port.c_str()) {}
#endif

void blob::Comms::init (int32_t baudrate) // FIXME treat retval and errors
{
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

bool blob::Comms::send (blob::Comms::MsgSubType st)
{
  emptyMsg_t msg;
  bool retval = false;

  memset(&msg, 0, sizeof(msg));

  if(st == blob::Comms::Health)
  {
    msg.header.type = blob::Comms::Data;
    msg.header.subtype = st;

    msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

    Serial.write((byte *) _sync, sizeof(_sync));
    Serial.write((byte *)&msg, sizeof(msg));

    retval = true;
  }
  return retval;
} // Comms::send

// FIXME check if values are within range of int 16 (aprox +- 32.2)
bool blob::Comms::send(blob::Vector3d<float> e, blob::Vector3d<float> a, blob::Vector3d<float> v, blob::Vector3d<float> p)
{
  bool retval = true;

  stateMsg_t msg;

  memset(&msg, 0, sizeof(msg));

  msg.header.type = blob::Comms::Data;
  msg.header.subtype = blob::Comms::State;

  msg.ex = (int16_t)(1000.f*e.x);
  msg.ey = (int16_t)(1000.f*e.y);
  msg.ez = (int16_t)(1000.f*e.z);
  msg.ax = (int16_t)(1000.f*a.x);
  msg.ay = (int16_t)(1000.f*a.y);
  msg.az = (int16_t)(1000.f*a.z);
  msg.vx = (int16_t)(1000.f*v.x);
  msg.vy = (int16_t)(1000.f*v.y);
  msg.vz = (int16_t)(1000.f*v.z);
  msg.px = (int16_t)(1000.f*p.x);
  msg.py = (int16_t)(1000.f*p.y);
  msg.pz = (int16_t)(1000.f*p.z);

  msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write((byte *)&msg, sizeof(msg));

  return retval;
} // Comms::send

bool blob::Comms::cmd (blob::Comms::MsgSubType st)
{
  emptyMsg_t msg;
  bool retval = false;

  memset(&msg, 0, sizeof(msg));

  if(st == blob::Comms::Start)
  {
    msg.header.type = blob::Comms::Command;
    msg.header.subtype = st;

    msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

    Serial.write((byte *) _sync, sizeof(_sync));
    Serial.write((byte *)&msg, sizeof(msg));

    retval = true;
  }
  return retval;
} // Comms::cmd

bool blob::Comms::cmd (blob::Comms::OnOff action)
{
  actionMsg_t msg;
  bool retval = false;

  memset(&msg, 0, sizeof(msg));

  msg.header.type = blob::Comms::Command;
  msg.header.subtype = blob::Comms::Motors;
  msg.value = action;
  msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write((byte *)&msg, sizeof(msg));

  retval = true;

  return retval;
} // Comms::cmd


bool blob::Comms::cmd (blob::Comms::InOut action)
{
  actionMsg_t msg;
  bool retval = false;

  memset(&msg, 0, sizeof(msg));

  msg.header.type = blob::Comms::Command;
  msg.header.subtype = blob::Comms::Dock;
  msg.value = action;
  msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write((byte *)&msg, sizeof(msg));

  retval = true;

  return retval;
} // Comms::cmd

// FIXME check if values are within range of int 16 (aprox +- 32.2)
bool blob::Comms::cmd (blob::Vector3d<float> vel, float vyaw)
{
  bool retval = false;

  velMsg_t msg;

  memset(&msg, 0, sizeof(msg));

  msg.header.type = blob::Comms::Command;
  msg.header.subtype = blob::Comms::Vel;

  msg.vx = (int16_t)(1000.f*vel.x);
  msg.vy = (int16_t)(1000.f*vel.y);
  msg.vz = (int16_t)(1000.f*vel.z);
  msg.vyaw = (int16_t)(1000.f*vyaw);

  msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write((byte *)&msg, sizeof(msg));

  retval = true;

  return retval;
} // Comms::cmd

// FIXME check if values are within range of int 16 (aprox +- 32.2)
bool blob::Comms::cmd (blob::Vector3d<float> pos, float heading, float speed)
{
  bool retval = false;

  gotoMsg_t msg;

  memset(&msg, 0, sizeof(msg));

  msg.header.type = blob::Comms::Command;
  msg.header.subtype = blob::Comms::Vel;

  msg.x = (int16_t)(1000.f*pos.x);
  msg.y = (int16_t)(1000.f*pos.y);
  msg.z = (int16_t)(1000.f*pos.z);
  msg.heading = (int16_t)(1000.f*heading);
  msg.speed = (int16_t)(1000.f*speed);

  msg.crc = calcCrc((byte *)&msg, sizeof(msg)-2);

  Serial.write((byte *) _sync, sizeof(_sync));
  Serial.write((byte *)&msg, sizeof(msg));

  retval = true;

  return retval;
} // Comms::cmd

bool blob::Comms::sync ()
{
  bool retval = false;

  enum {GotNothing, GotHalfSync, GotSync, GotMsgType};
  static int step = GotNothing;

  byte c = Serial.read();

  if (step == GotHalfSync && c == _sync[1])
  {
    step = GotSync;
  }
  else if (step == GotSync && isMsgTypeValid(c))
  {
    _received[0] = c;
    step = GotMsgType;
  }
  else if (step == GotMsgType && isMsgSubTypeValid(c))
  {
    _received[1] = c;
    step = GotNothing;
    retval = true;
#if defined(__DEBUG__) && defined(__linux) 
    std::cout << " got message header: 0x" << std::hex << (int)_sync[0] << (int)_sync[1] 
              << " type: " << (int)_received[0] 
              << " subtype: " << (int)_received[1] 
              << std::dec << std::endl;
#endif
  }
  else if (c == _sync[0])
  {
    step = GotHalfSync;
  }
  else
  {
    step = GotNothing;
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
      case blob::Comms::Data:
        switch (_received[1]) // subtype
        {
          case blob::Comms::Health:
            retval = retrieve(sizeof(emptyMsg_t)-2);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Health " << sizeof(emptyMsg_t)-2 << std::endl;
#endif
            break;
          case blob::Comms::State:
            retval = retrieve(sizeof(stateMsg_t)-2);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::State " << sizeof(stateMsg_t)-2 << std::endl;
#endif
            break;
          default: break;
        }
        break;

      case blob::Comms::Command:
        switch(_received[1]) // subtype
        {
          case blob::Comms::Start:
            retval = retrieve(sizeof(emptyMsg_t)-2);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Start " << sizeof(emptyMsg_t)-2 << std::endl;
#endif
            break;
          case blob::Comms::Motors:
            retval = retrieve(sizeof(actionMsg_t)-2);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " retrieving blob::Comms::Motors " << sizeof(actionMsg_t)-2 << std::endl;
#endif
            break;
          default: break;
        }
        break;

      case blob::Comms::Request:
        break;
      case blob::Comms::Reply:
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
    Serial.readBytes((char *)&_received[2], length);

    uint16_t receivedCrc = (uint16_t)(_received[length+1]<<8)+_received[length];
    uint16_t frameCrc = calcCrc(_received, length);
#if defined(__DEBUG__) && defined(__linux) 
            std::cout << " received crc: " << receivedCrc 
                      << " shouldbe crc: " << frameCrc << std::endl;
#endif
    if (receivedCrc != frameCrc)
    {
      _type = blob::Comms::NotValid;
    }
    else
    {
      _type = (blob::Comms::MsgType)_received[0];
      _subtype = (blob::Comms::MsgSubType)_received[1];
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
    stateMsg_t *msg = (stateMsg_t *)_received;
    euler.x = 0.001f*msg->ex;
    euler.y = 0.001f*msg->ey;
    euler.z = 0.001f*msg->ez;
    acc.x   = 0.001f*msg->ax;
    acc.y   = 0.001f*msg->ay;
    acc.z   = 0.001f*msg->az;
    vel.x   = 0.001f*msg->vx;
    vel.y   = 0.001f*msg->vy;
    vel.z   = 0.001f*msg->vz;
    pos.x   = 0.001f*msg->px;
    pos.y   = 0.001f*msg->py;
    pos.z   = 0.001f*msg->pz;
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
    actionMsg_t *msg = (actionMsg_t *)_received;
    action = (OnOff)msg->value;
    retval = true;
  }
  return retval;
} // Comms::getCmd


bool blob::Comms::getCmd (InOut &action)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == Dock)
  {
    actionMsg_t *msg = (actionMsg_t *)_received;
    action = (InOut)msg->value;
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &vel, float &vyaw)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == Vel)
  {
    velMsg_t *msg = (velMsg_t *)_received;
    vel.x = 0.001f*msg->vx;
    vel.y = 0.001f*msg->vy;
    vel.z = 0.001f*msg->vz;
    vyaw  = 0.001f*msg->vyaw;
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &pos, float &heading, float &speed)
{
  bool retval = false;

  if(_receiving == false && _type == Command && _subtype == GoTo)
  {
    gotoMsg_t *msg = (gotoMsg_t *)_received;
    pos.x   = 0.001f*msg->x;
    pos.y   = 0.001f*msg->y;
    pos.z   = 0.001f*msg->z;
    heading = 0.001f*msg->heading;
    speed   = 0.001f*msg->speed;
    retval = true;
  }
  return retval;
} // Comms::getCmd
