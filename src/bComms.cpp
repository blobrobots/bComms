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
#endif

static const byte sync[2] = {0xFF, 0xFF};

#if defined(__linux__)
  blob::Comms::Comms (std::string port) : Serial(port.c_str()) {}
#endif

void blob::Comms::init (int32_t baudrate)
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(baudrate);

#if defined(__AVR_ATmega32U4__)
  // wait for Leonardo enumeration, others continue immediately
  while (!Serial);
#endif
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
  return crc;
}

bool blob::Comms::send (blob::Comms::MsgSubType st)
{
  emptyMsg_t msg;
  bool retval = false;

  memset(msg.frame, 0, sizeof(B_COMMS_EMPTY_MSG_SIZE));

  if(st == blob::Comms::Health)
  {
    msg.field.header.type = blob::Comms::Data;
    msg.field.header.subtype = st;

    msg.field.crc = calcCrc(msg.frame,B_COMMS_EMPTY_MSG_SIZE-2);

    Serial.write((byte *) sync, sizeof(sync));
    Serial.write(msg.frame, B_COMMS_EMPTY_MSG_SIZE);

    retval = true;
  }
  return retval;
} // Comms::send

bool blob::Comms::send(blob::Vector3d<float> e, blob::Vector3d<float> a, blob::Vector3d<float> v, blob::Vector3d<float> p)
{
  bool retval = true;

  stateMsg_t msg;

  memset(msg.frame, 0, sizeof(B_COMMS_STATE_MSG_SIZE));

  msg.field.header.type = blob::Comms::Data;
  msg.field.header.subtype = blob::Comms::State;

  msg.field.ex = (int16_t)(1000.f*e.x);
  msg.field.ey = (int16_t)(1000.f*e.y);
  msg.field.ez = (int16_t)(1000.f*e.z);
  msg.field.ax = (int16_t)(1000.f*a.x);
  msg.field.ay = (int16_t)(1000.f*a.y);
  msg.field.az = (int16_t)(1000.f*a.z);
  msg.field.vx = (int16_t)(1000.f*v.x);
  msg.field.vy = (int16_t)(1000.f*v.y);
  msg.field.vz = (int16_t)(1000.f*v.z);
  msg.field.px = (int16_t)(1000.f*p.x);
  msg.field.py = (int16_t)(1000.f*p.y);
  msg.field.pz = (int16_t)(1000.f*p.z);

  msg.field.crc = calcCrc(msg.frame,B_COMMS_STATE_MSG_SIZE-2);

  Serial.write((byte *) sync, sizeof(sync));
  Serial.write(msg.frame, B_COMMS_STATE_MSG_SIZE);

  return retval;
} // Comms::send

bool blob::Comms::cmd (blob::Comms::MsgSubType st)
{
  emptyMsg_t msg;
  bool retval = false;

  memset(msg.frame, 0, sizeof(B_COMMS_EMPTY_MSG_SIZE));

  if(st == blob::Comms::Start)
  {
    msg.field.header.type = blob::Comms::Command;
    msg.field.header.subtype = st;

    msg.field.crc = calcCrc(msg.frame,B_COMMS_EMPTY_MSG_SIZE-2);

    Serial.write((byte *) sync, sizeof(sync));
    Serial.write(msg.frame, B_COMMS_EMPTY_MSG_SIZE);

    retval = true;
  }
  return retval;
} // Comms::cmd

bool blob::Comms::cmd (blob::Comms::OnOff action)
{
  actionMsg_t msg;
  bool retval = false;

  memset(msg.frame, 0, sizeof(B_COMMS_EMPTY_MSG_SIZE));

  msg.field.header.type = blob::Comms::Command;
  msg.field.header.subtype = blob::Comms::Motors;
  msg.field.value = action;
  msg.field.crc = calcCrc(msg.frame,B_COMMS_EMPTY_MSG_SIZE-2);

  Serial.write((byte *) sync, sizeof(sync));
  Serial.write(msg.frame, B_COMMS_EMPTY_MSG_SIZE);

  retval = true;

  return retval;
} // Comms::cmd


bool blob::Comms::cmd (blob::Comms::InOut action)
{
  actionMsg_t msg;
  bool retval = false;

  memset(msg.frame, 0, sizeof(B_COMMS_EMPTY_MSG_SIZE));

  msg.field.header.type = blob::Comms::Command;
  msg.field.header.subtype = blob::Comms::Dock;
  msg.field.value = action;
  msg.field.crc = calcCrc(msg.frame,B_COMMS_EMPTY_MSG_SIZE-2);

  Serial.write((byte *) sync, sizeof(sync));
  Serial.write(msg.frame, B_COMMS_EMPTY_MSG_SIZE);

  retval = true;

  return retval;
} // Comms::cmd

bool blob::Comms::cmd (blob::Vector3d<float> vel, float vyaw)
{
  bool retval = false;

  velMsg_t msg;

  memset(msg.frame, 0, sizeof(B_COMMS_VEL_MSG_SIZE));

  msg.field.header.type = blob::Comms::Command;
  msg.field.header.subtype = blob::Comms::Vel;

  msg.field.vx = (int16_t)(1000.f*vel.x);
  msg.field.vy = (int16_t)(1000.f*vel.y);
  msg.field.vz = (int16_t)(1000.f*vel.z);
  msg.field.vyaw = (int16_t)(1000.f*vyaw);

  msg.field.crc = calcCrc(msg.frame,B_COMMS_VEL_MSG_SIZE-2);

  Serial.write((byte *) sync, sizeof(sync));
  Serial.write(msg.frame, B_COMMS_VEL_MSG_SIZE);

  retval = true;

  return retval;
} // Comms::cmd

bool blob::Comms::cmd (blob::Vector3d<float> pos, float heading, float speed)
{
  bool retval = false;

  gotoMsg_t msg;

  memset(msg.frame, 0, sizeof(B_COMMS_GOTO_MSG_SIZE));

  msg.field.header.type = blob::Comms::Command;
  msg.field.header.subtype = blob::Comms::Vel;

  msg.field.x = (int16_t)(1000.f*pos.x);
  msg.field.y = (int16_t)(1000.f*pos.y);
  msg.field.z = (int16_t)(1000.f*pos.z);
  msg.field.heading = (int16_t)(1000.f*heading);
  msg.field.speed = (int16_t)(1000.f*speed);

  msg.field.crc = calcCrc(msg.frame,B_COMMS_GOTO_MSG_SIZE-2);

  Serial.write((byte *) sync, sizeof(sync));
  Serial.write(msg.frame, B_COMMS_GOTO_MSG_SIZE);

  retval = true;

  return retval;
} // Comms::cmd


bool blob::Comms::receive ()
{
  bool retval = false;

  if(Serial.available() > 3)
  {
    if(_receiving == false)
    { // receive header
      if (Serial.read() == sync[0] && Serial.read() == sync[1] &&
         isMsgTypeValid(_received[0] = Serial.read()) &&
         isMsgSubTypeValid(_received[1] = Serial.read()))
      {
         _receiving = true;
      }
    }
    else
    { // retrieve body
      switch (_received[0]) // type
      {
        case blob::Comms::Data:
          switch(_received[1]) // subtype
          {
            case blob::Comms::Health:
              retval = retrieve(B_COMMS_EMPTY_MSG_SIZE-2);
              break;
            case blob::Comms::State:
              retval = retrieve(B_COMMS_STATE_MSG_SIZE-2);
              break;

            default: break;
          }
          break;

        case blob::Comms::Command:
          switch(_received[1]) // subtype
          {
            case blob::Comms::Start:
              retval = retrieve(B_COMMS_EMPTY_MSG_SIZE-2);
              break;
            case blob::Comms::Motors:
              retval = retrieve(B_COMMS_ONOFF_MSG_SIZE-2);
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
  }
} // Comms::receive

bool blob::Comms::retrieve (size_t length)
{
  bool retval = false;

  if (Serial.available() >= length)
  {
    // retrieve message including crc
    Serial.readBytes((char *)&_received[2], length);

    uint16_t receivedCrc = *((uint16_t *)&_received[length-1]);
    uint16_t frameCrc = calcCrc(_received, length);

    if (receivedCrc != frameCrc) // && Serial.peek() != sync[0]
      _type = blob::Comms::NotValid;
    else
      retval = true;

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

  if(_type == Data && _subtype == State)
  {
    stateMsg_t *msg = (stateMsg_t *)_received;
    euler.x = 0.001f*msg->field.ex;
    euler.y = 0.001f*msg->field.ey;
    euler.z = 0.001f*msg->field.ez;
    acc.x = 0.001f*msg->field.ax;
    acc.y = 0.001f*msg->field.ay;
    acc.z = 0.001f*msg->field.az;
    vel.x = 0.001f*msg->field.vx;
    vel.y = 0.001f*msg->field.vy;
    vel.z = 0.001f*msg->field.vz;
    pos.x = 0.001f*msg->field.px;
    pos.y = 0.001f*msg->field.py;
    pos.z = 0.001f*msg->field.pz;
    retval = true;
  }
  return retval;
} // Comms::getData

bool blob::Comms::getCmd (MsgSubType subtype)
{
  bool retval = false;

  if(_type == Command && _subtype == subtype)
  {
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (OnOff &action)
{
  bool retval = false;

  if(_type == Command && _subtype == Motors)
  {
    actionMsg_t *msg = (actionMsg_t *)_received;
    action = (OnOff)msg->field.value;
    retval = true;
  }
  return retval;
} // Comms::getCmd


bool blob::Comms::getCmd (InOut &action)
{
  bool retval = false;

  if(_type == Command && _subtype == Dock)
  {
    actionMsg_t *msg = (actionMsg_t *)_received;
    action = (InOut)msg->field.value;
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &vel, float &vyaw)
{
  bool retval = false;

  if(_type == Command && _subtype == Vel)
  {
    velMsg_t *msg = (velMsg_t *)_received;
    vel.x = 0.001f*msg->field.vx;
    vel.y = 0.001f*msg->field.vy;
    vel.z = 0.001f*msg->field.vz;
    vyaw  = 0.001f*msg->field.vyaw;
    retval = true;
  }
  return retval;
} // Comms::getCmd

bool blob::Comms::getCmd (Vector3d<float> &pos, float &heading, float &speed)
{
  bool retval = false;

  if(_type == Command && _subtype == GoTo)
  {
    gotoMsg_t *msg = (gotoMsg_t *)_received;
    pos.x     = 0.001f*msg->field.x;
    pos.y     = 0.001f*msg->field.y;
    pos.z     = 0.001f*msg->field.z;
    heading   = 0.001f*msg->field.heading;
    speed     = 0.001f*msg->field.speed;
    retval = true;
  }
  return retval;
} // Comms::getCmd
