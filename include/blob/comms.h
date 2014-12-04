/********* blob robotics 2014 *********
 *  title: comms.h
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
  #include "blob/serial.h"
#endif // defined(__linux__)

#include "blob/types.h"
#include "blob/math.h"

#define BCOMMS_MAX_LENGTH     64 // bytes

namespace blob {

class Comms
{
  public:
    enum MsgType {Data = 0, Command = 1, Request = 2, Reply = 3, NotValid = 255};
    enum MsgSubType {Health = 0, State = 1, Start = 10, Motors = 11, Dock = 12, Vel = 13, GoTo = 14};
    enum OnOff {Off = 0, On = 1};
    enum InOut {Out = 0, In = 1};
    enum Response {Nack = -1, Ack = 0};
/*
    typedef struct {
      uint8_t type;       // MsgType
      uint8_t subtype;    // MsgSubType      
      uint32_t timestamp; // ms
    } headerMsg_t; // 6 bytes

    typedef struct {
      headerMsg_t header;
      uint8_t flags;
      uint8_t mode;
      float ex;
      float ey;
      float ez;
      float ax;
      float ay;
      float az;
      float vx;
      float vy;
      float vz;
      float px;
      float py;
      float pz;
    } stateMsg_t; // 58 bytes

    typedef struct {
      headerMsg_t header;
      uint16_t prio;
      float  vx;
      float  vy;
      float  vz;
      float  vyaw;
    } velMsg_t; // 26 bytes

   typedef struct {
      headerMsg_t header;
      float x;
      float y;
      float z;
      float heading;
      float speed;
    } gotoMsg_t; // 28 bytes

    typedef struct {
      headerMsg_t header;
      uint8_t value;
    } actionMsg_t; // 9 bytes

    typedef struct {
      headerMsg_t header;
    } emptyMsg_t; // 8 bytes
*/
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
    uint32_t getTimestamp () {return _timestamp;}

  private:
    int16_t calcCrc (byte *frame, size_t length);
    uint32_t timestamp ();
    bool sync ();
    bool retrieve (size_t length);
    bool isMsgTypeValid (int8_t type);
    bool isMsgSubTypeValid (int8_t subtype);

    template <typename T> bool push (T const& v);
    template <typename T> T pop ();
    bool flushTx ();
    bool flushRx ();
    
    bool _receiving;

    MsgType _type;
    MsgSubType _subtype;
    uint32_t _timestamp;

    byte _received[BCOMMS_MAX_LENGTH]; // read buffer
    byte _buffer[BCOMMS_MAX_LENGTH];   // write buffer
    uint8_t _irx, _itx;

#if defined(__linux__)
    blob::Serial Serial;
#endif // defined(__linux__)

};

}

#endif /* B_COMMS_H */
