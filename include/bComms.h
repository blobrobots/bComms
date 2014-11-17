/********* blob robotics 2014 *********
 *  title: bComms.h
 *  brief: interface communication manager
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_COMMS_H
#define B_COMMS_H

#if defined(__AVR_ATmega32U4__)
  #include "Arduino.h"
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  #include <stdlib.h>
  #include "bSerial.h"
#endif // defined(__linux__)

#include "bTypes.h"
#include "bMath.h"

#define B_COMMS_MAX_MSG_SIZE   64 // bytes
#define B_COMMS_STATE_MSG_SIZE 32 // bytes
#define B_COMMS_VEL_MSG_SIZE   16 // bytes
#define B_COMMS_GOTO_MSG_SIZE  16 // bytes
#define B_COMMS_ONOFF_MSG_SIZE  8 // bytes
#define B_COMMS_EMPTY_MSG_SIZE  4 // bytes

namespace blob {

class Comms
{
  public:
    enum MsgType {Data = 0, Command = 1, Request = 2, Reply = 3, NotValid = 255};
    enum MsgSubType {Health = 0, State = 1, Start = 10, Motors = 11, Dock = 12, Vel = 13, GoTo = 14};
    enum OnOff {Off = 0, On = 1};
    enum InOut {Out = 0, In = 1};
    enum Response {Nack = -1, Ack = 0};

    typedef struct {
      uint8_t type;
      uint8_t subtype;
    } headerMsg_t;

    typedef struct {
      headerMsg_t header;
      uint8_t flags;
      uint8_t mode;
      int16_t ex;
      int16_t ey;
      int16_t ez;
      int16_t ax;
      int16_t ay;
      int16_t az;
      int16_t vx;
      int16_t vy;
      int16_t vz;
      int16_t px;
      int16_t py;
      int16_t pz;
      uint16_t crc;
    } stateSt_t;

    typedef union {
      byte frame[B_COMMS_STATE_MSG_SIZE];
      stateSt_t field;
    } stateMsg_t;

    typedef struct {
      headerMsg_t header;
      uint16_t prio;
      int16_t  vx;
      int16_t  vy;
      int16_t  vz;
      int16_t  vyaw;
      uint16_t crc;
    } velSt_t;

    typedef union {
      byte frame[B_COMMS_VEL_MSG_SIZE];
      velSt_t field;
    } velMsg_t;

    typedef struct {
      headerMsg_t header;
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t heading;
      int16_t speed;
      uint16_t crc;
    } gotoSt_t;

    typedef union {
      byte frame[B_COMMS_GOTO_MSG_SIZE];
      gotoSt_t field;
    } gotoMsg_t;

    typedef struct {
      headerMsg_t header;
      uint16_t value;
      uint16_t crc;
    } actionSt_t;

    typedef union {
      byte frame[B_COMMS_ONOFF_MSG_SIZE];
      actionSt_t field;
    } actionMsg_t;

    typedef struct {
      headerMsg_t header;
      uint16_t crc;
    } emptySt_t;

    typedef union {
      byte frame[B_COMMS_EMPTY_MSG_SIZE];
      emptySt_t field;
    } emptyMsg_t;

#if defined(__linux__)
    Comms(std::string port = "/dev/ttyACM0");
#endif // defined(__linux__)

    void init    (int32_t baudrate = 115200);
    bool isReady ();

    bool send (Vector3d<float> euler, Vector3d<float> acc, Vector3d<float> vel, Vector3d<float> pos) ;
    bool send (MsgSubType subtype);

    bool cmd (MsgSubType subtype);
    bool cmd (OnOff action);
    bool cmd (InOut action);
    bool cmd (Vector3d<float> vel, float vyaw);
    bool cmd (Vector3d<float> pos, float heading, float speed);

    bool req   (MsgSubType subtype);
    bool reply (MsgSubType subtype, Response resp);

    bool receive ();

    bool getData (Vector3d<float> &euler, Vector3d<float> &acc, Vector3d<float> &vel, Vector3d<float> &pos);

    bool getCmd (MsgSubType subtype);
    bool getCmd (OnOff &action);
    bool getCmd (InOut &action);
    bool getCmd (Vector3d<float> &vel, float &vyaw);
    bool getCmd (Vector3d<float> &pos, float &heading, float &speed);

    bool getReq  ();

    MsgType getMsgType () {return _type;}

  private:
    int16_t calcCrc (byte *frame, size_t length);
    bool retrieve (size_t length);
    bool isMsgTypeValid (int8_t type);
    bool isMsgSubTypeValid (int8_t subtype);

    bool _receiving;
    MsgType _type;
    MsgSubType _subtype;
    byte _received[B_COMMS_MAX_MSG_SIZE];

#if defined(__linux__)
    blob::Serial Serial;
#endif // defined(__linux__)

};

}

#endif /* B_COMMS_H */
