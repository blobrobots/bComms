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
 * \file       comms.h
 * \brief      interface for communication manager
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_COMMS_H
#define B_COMMS_H

#include "blob/types.h"
#include "blob/math.h"

#if defined(__linux__)
  #include <stdlib.h>
  #include <string>
  #include <cstdarg>
#endif // defined(__linux__)

#define BCOMMS_MAX_LENGTH     64 // bytes

namespace blob {

/**
 * Implements communication services among blob devices.
 */
class Comms
{
  public:
    
    /**
     * Control characters for serialization.
     * SYN: Synchronous idle
     * SOH: Start of heading
     * STX: Start of text
     * ETB: End of transmission block
     * ETX: End of text
     * EOT: End of transmission
     * ENQ: Enquiry
     * NAK: Negative acknowledgement
     * DLE: Data link escape
     * ITB: Intermediate block check character
     */
    enum CtrlChar { SYN=0x32, SOH=0x01, STX=0x02, ETB=0x26, ETX=0x03, 
                    EOT=0x37, ENQ=0x2D, NAK=0x3D, DLE=0x10, ITB=0x1F };
    
    /**
     * Types of message that can be sent over comms.
     */
    enum MsgType { Data=0, Command=1, Request=2, Reply=3, InvalidMsg=0xFF };

    /**
     * Types of commands that can be sent over comms.
     */
    enum Cmd { Setup=0, OnOff=1, Dock=2, Move=3, InvalidCmd=0xFF };

    /**
     * Types of data that can be sent over comms.
     */
    enum Data { Time=0, State=1, Motors=2, Imu=3, Mag=4, Ahrs=5, Baro=6, Vel=7, 
                Pos=8, Gps=9, Misc=0xFE, InvalidData=0xFF };
    /**
     * Response types available.
     */
    enum Resp { Nack=-1, Ack=0 };

    /**
     * Interface for communication channel.
     */
    class Channel
    {
      public:

      /**
       * Initializes communication over communication channel.
       * \return  true if successful, false otherwise
       */
      virtual bool init () {return false;}

      /**
       * Stops communication over communication channel.
       * \return  true if successful, false otherwise
       */
      virtual bool end () {return false;}

      /**
       * Indicates whether communications are ready or not.
       * \return  true if communications are ready, false otherwise
       */
      virtual bool isReady () {return false;}

      /**
       * Provides number of bytes available in communication channel.
       * \return  number of bytes available or -1 if error
       */
      virtual int available () {return -1;}

      /**
       * Empties reception buffer of available bytes.
       */
      virtual void flush () {return;}

      /**
       * Provides next byte available in communication channel without removing 
       * it from the internal serial buffer. Successive calls would provide the 
       * same byte.
       * \param[out] b  byte read
       * \return        true if a byte is available and peek is successful, false 
       *                otherwise     
       */
      virtual bool peek (byte& b) {return false;}
      
      /**
       * Reads next byte available in communication channel.
       * \param[out] b  byte read
       * \return        true if a byte is available and read is successful, false 
       *                otherwise     
       */
      virtual bool read (byte& b) {return false;}

      /**
       * Reads bytes from the communication channel into a buffer. It terminates
       * if the determined length has been read, or it times out.
       * \param[in]  length  number of bytes to be read
       * \param[out] buffer  bytes read
       * \return             number of bytes read or -1 if error
       * \sa setTimeout()
       */
      virtual int read (const uint16_t& length, byte *buffer) {return -1;}

      /**
       * Writes a frame of bytes into the communication channel.
       * \param[in]  length  number of bytes to be written
       * \param[out] buffer  bytes to write
       * \return             number of bytes written or -1 if error
       */
      virtual int write (const uint16_t& length, const byte *buffer) {return -1;}

      /**
       * Sets timeout for read operations over communication channel.
       * \param ms  timeout in milliseconds
       * \return    true if successful, false otherwise
       */
      virtual bool setTimeout (const long& ms) {return false;}

    };
    
    /**
     * Interface for data receiver.
     */
    class DataReceiver
    {
      public:

        /**
         * Callback executed when timestamp data is received.
         * \param ts    new data timestamp in ms
         * \return      true if successful, false otherwise
         */
        virtual bool onTimeData (const uint32_t& ts) {return false;}

        /**
         * Callback executed when state data is received.
         * \param state  mask describing system state
         * \return       true if successful, false otherwise
         */
        virtual bool onStateData (const uint16_t& state) {return false;}
        /**
         * Callback executed when motor data is received.
         * \param motors  array of motors value in rpm
         * \param num     number of motors
         * \return        true if successful, false otherwise
         */
        virtual bool onMotorData (const uint8_t& num, 
                                  const uint16_t* motors) {return false;}
        /**
         * Callback executed when imu data is received.
         * \param gyro  three axis gyroscope data
         * \param acc   three axis accelerometer data
         * \return      true if successful, false otherwise
         */
        virtual bool onImuData (const Vector3d<float>& gyro, 
                                const Vector3d<float>& acc) {return false;}
        /**
         * Callback executed when magnetometer data is received.
         * \param mag   three axis magnetometer data
         * \return      true if successful, false otherwise
         */
        virtual bool onMagData (const Vector3d<float>& mag) {return false;}
        /**
         * Callback executed when ahrs data is received.
         * \param euler  euler angles for attitude and orientation
         * \param ts     timestamp in ms
         * \return       true if successful, false otherwise
         */
        virtual bool onAhrsData (const Vector3d<float>& euler) {return false;}
        /**
         * Callback executed when ahrs data is received.
         * \param pressure  air pressure in Pa
         * \param temp      air temperature in Celsius deg.
         * \param altitude  altitude over sea level (101325 Pa)
         * \param roc       rate of climb
         * \param ts        timestamp in ms
         * \return          true if successful, false otherwise
         */
        virtual bool onBaroData (const float& pressure, 
                                 const float& temperature, 
                                 const float& altitude, 
                                 const float& roc) {return false;}
    };

    /**
     * Interface for command receiver.
     */
    class CommandReceiver
    {
      public:
        /**
         * Callback executed when setup command is received.
         * \param mask  setup mask with data message types to be sent back
         * \return      true if successful, false otherwise
         */
        virtual bool onSetupCmd (const uint32_t& mask) {return false;}
        
        /**
         * Callback executed when motor command is received.
         * \param action  motor mode (on/off) to change to
         * \return      	true if successful, false otherwise
         */
        virtual bool onOnOffCmd (const uint8_t& action) {return false;}

        /**
         * Callback executed when setup command is received.
         * \param action  docking action (in/out) to perform
         * \return        true if successful, false otherwise
         */
        virtual bool onDockCmd (const uint8_t& action) {return false;}
        
        /**
         * Callback executed when setup command is received.
         * \param movement  three axis movement to perform
         * \param heading   heading movement
         * \return          true if successful, false otherwise
         */
        virtual bool onMoveCmd (const Vector3d<float>& movement, 
                                const float& heading) {return false;}
    };
    
    /**
     * Initializes communication parameters and internal variables.
     */
    Comms ();

    /**
     * Indicates whether communications are ready or not.
     * \return   true if communications are ready, false otherwise
     */
    bool isReady ();

    /**
     * Connects to communication channel which needs to be already initialized.
     * \param channel  pointer to communication channel
     * \return         true if successful, false otherwise
     */
    bool connectTo (Channel* ch);

    /**
     * Disconnects from communication channel.
     * \param channel  pointer to the connected communication channel 
     * \return  true if successful, false otherwise
     */
    bool disconnectFrom (Channel* ch);

    /**
     * Registers a data receiver if no other data receiver is registered.
     * \param receiver  pointer to data receiver
     * \return          true if successful, false otherwise
     */
    bool subscribe (DataReceiver* receiver);

    /**
     * Unregisters a data receiver that is already registered.
     * \param receiver  pointer to data receiver
     * \return          true if successful, false otherwise
     */
    bool unsubscribe (DataReceiver* receiver);

    /**
     * Registers a command receiver if no other command receiver is registered.
     * \param receiver  pointer to command receiver
     * \return          true if successful, false otherwise
     */
    bool subscribe (CommandReceiver* receiver);

    /**
     * Unregisters a command receiver that is already registered.
     * \param receiver  pointer to command receiver
     * \return          true if successful, false otherwise
     */
    bool unsubscribe (CommandReceiver* receiver);

    
    /**
     * Prepares system timestamp to be sent.
     * \param ts  system timestamp in ms
     * \return    true if successful, false otherwise
     */
    bool prepareTime (const uint32_t& ts);
    
    /**
     * Prepares system state to be sent.
     * \param state  mask describing system state
     * \return       true if successful, false otherwise
     */
    bool prepareState (const uint16_t& state);
    
		/**
     * Prepares motors state to be sent.
     * \param motors  array of motors value in rpm
     * \param num     number of motors
     * \return        true if successful, false otherwise
     */    
    bool prepareMotors (const uint8_t& num, const uint16_t* motors);
    
		/**
     * Prepares Imu data to be sent.
     * \param gyro  three axis gyroscope data
     * \param acc   three axis accelerometer data
     * \return      true if successful, false otherwise
     */ 
    bool prepareImu (const Vector3d<float>& gyro, 
                  	 const Vector3d<float>& acc);
    /**
     * Prepares Magnetometer data to be sent.
     * \param mag  three axis magnetometer data
     * \return     true if successful, false otherwise
     */
    bool prepareMag (const Vector3d<float>& mag);

    /**
     * Prepares Ahrs data to be sent.
     * \param euler  euler angles for attitude and orientation
     * \return       true if successful, false otherwise
     */
		bool prepareAhrs (const Vector3d<float>& euler);

    /**
     * Prepares Baro data to be sent.
     * \param pressure  air pressure in Pa
     * \param temp      air temperature in Celsius deg.
     * \param altitude  altitude over sea level (101325 Pa)
     * \param roc       rate of climb
     * \return          true if successful, false otherwise
     */        
    bool prepareBaro (const float& pressure, const float& temperature, 
                  		const float& altitude, const float& roc);

    /**
     * Sends all data previusly prepared into tx buffer.
     * \return  true if successful, false otherwise
     */
    bool send ();

    /**
     * Sends setup command. 
     * \param num  number of data messages to be setup to send
     * \param ...  data message type to be setup to send
     * \return     true if successful, false otherwise
     */
    bool cmdSetup (const uint8_t& num, ...);          

    /**
     * Sends motors start/stop command.
     * \param action  On/Off action
     * \return        true if successful, false otherwise
     */
    bool cmdOnOff (const uint8_t& action);

    /**
     * Sends dock in/out command.
     * \param action  In/Out action
     * \return        true if successful, false otherwise
     */
    bool cmdDock (const uint8_t& action);

    /**
     * Sends movement command.
     * \param movement  three movement
     * \param heading   heading movement
     * \return          true if successful, false otherwise
     */    
    bool cmdMove (const Vector3d<float>& movement, const float& heading);
    
//    bool req   (MsgSubType subtype); TODO
//    bool reply (MsgSubType subtype, Response resp); TODO

    /**
     * Receives and parses communication frame, including data, commands and
     * requests. Invoques callbacks if a receiver is registered. 
     * \return  true if a complete frame read successfully, false otherwise
     */        
    bool receive ();

//    bool getReq  (); TODO

    /**
     * Provides communication channel handler
     * \return  pointer to communication channel
     */
    Channel* channel () {return _channel;}

  private:

    /**
     * Checks if system is little or big endian.
     * \return  true if system is little endian, false if system is big endian 
     */
    bool isLittleEndian()
    {
      short int n=0x1;
      return (*((char*)&n) == 1);
    }

    /**
     * Calculates 16 bit checksum over frame (excluding control bytes).
     * \param[in] length  frame number of bytes excluding control bytes and crc
     * \param[in] frame   bytes of frame over which crc is calculated
     * \param[out] c0     least significant byte of checksum
     * \param[out] c1     most significant byte of checksum
     * \return            true if successful, false otherwise 
     */
    bool calcCrc (const size_t& length, const byte* frame, byte& c0, byte& c1);
        
    bool serialize (const size_t& length, const byte* frame);

    bool deserialize (const size_t& length, const byte* frame);


    /**
     * Finds frame sync bytes over input stream.
     * \return  true if successful, false otherwise
     */
    bool sync ();

    /**
     * Isolates frame of specific length from input stream.
     * \param length  number of bytes of frame to be parsed
     * \return        true if successful, false otherwise
     */ 
    bool parse (const size_t& length);

    /**
     * Checks if a message type value is valid.
     * \param type  message type to be checked
     * \return      true if type is valid, false otherwise
     */ 
    bool isMsgTypeValid (const int8_t& type);

    /**
     * Checks if a message subtype value is valid.
     * \param type  message type of message subtype to be checked
     * \param type  message subtype to be checked
     * \return      true if subtype is valid, false otherwise
     */     
    bool isMsgSubTypeValid (const int8_t& type, const int8_t& subtype);

    /**
     * Adds data of any type to a buffer.
     * \param v       data to be added to transmission buffer
     * \param index   index of next byte available in buffer
     * \param buffer  buffer data is pushed to
     * \return        true if successful, false otherwise
     */
    template <typename T> bool push (T const& v, const uint8_t& length,
                                                 byte* buffer, uint8_t& index );

    /**
     * Adds data of any type to transmission buffer.
     * \param v       data to be added to transmission buffer
     * \return        true if successful, false otherwise
     */
    template <typename T> bool push (T const& v);

    /**
     * Extracts data of any type from received buffer.
     * \return  data extracted from received buffer
     */
    template <typename T> T pop ();

    /**
     * Empties transmission buffer without sending the data.
     * \return   true if successful, false otherwise
     */
    bool flushTx ();

    /**
     * Empties reception buffer without processing the received data.
     * \return   true if successful, false otherwise
     */
    bool flushRx ();

    /**
     * Processes data frame and executes callbacks from data listener.
     * \return   true if successful, false otherwise
     */
    bool processData ();

    /**
     * Processes command frame and executes callbacks from data listener.
     * \return   true if successful, false otherwise
     */
    bool processCmd ();

  private:
    
    bool _receiving;          /**< flag indicating if a frame is being parsed */

    uint32_t _dataMask;       /**< mask indicating data setup to transmit */

    byte _rxBuffer[BCOMMS_MAX_LENGTH];  /**< reception buffer */
    byte _txBuffer[BCOMMS_MAX_LENGTH];  /**< transmission buffer */
    uint8_t _irx;                       /**< reception buffer index */
    uint8_t _itx;                       /**< transmission buffer index */

    Channel *_channel;                  /**< communication channel handler */

    DataReceiver *_dataListener;        /**< data listener register */
    CommandReceiver *_cmdListener;      /**< command listener register */

};

}

#endif /* B_COMMS_H */
