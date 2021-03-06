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
 * \file       serial.h
 * \brief      interface for serial port communication
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_SERIAL_H
#define B_SERIAL_H

#include <blob/types.h>
#include <blob/comms.h>

#if defined(__linux__)
 #include <termios.h>
#endif

namespace blob {

/**
 * Implements communication over serial port.
 */
class SerialPort : public Comms::Channel
{
  public:
   
    /**
     * Initializes communication parameters of serial port.
     * \param port      name of serial port to be used
     * \param baudrate  baudrate to be used over serial port
     * \param cfg       serial port configuration (parity)
     * \return          true if successful, false otherwise
     */
    SerialPort (const char* port="/dev/ttyUSB0", 
                const uint32_t &baudrate=115200, 
                const char* config="8N1");

    /**
     * Initializes communication over serial port.
     * \return  true if successful, false otherwise
     */
    virtual bool init ();

    /**
     * Stops communication over serial port.
     * \return  true if successful, false otherwise
     */
    virtual bool end ();

    /**
     * Indicates whether serial port is ready or not.
     * \return  true if communications are ready, false otherwise
     */
    virtual bool isReady ();

    /**
     * Provides number of bytes available in serial port.
     * \return  number of bytes available or -1 if none
     */
    virtual int available ();    
    /**
     * Empties serial port buffer of available bytes.
     */
    virtual void flush ();

    /**
     * Provides next byte available in serial port without removing it from the 
     * internal serial buffer. Successive calls would provide the same byte.
     * \param[out] b  byte read
     * \return        true if a byte is available and peek is successful, false 
     *                otherwise     
     */
    virtual bool peek (byte& b);
    
    /**
     * Reads next byte available in serial port.
     * \param[out] b  byte read
     * \return        true if a byte is available and read is successful, false 
     *                otherwise     
     */
    virtual bool read (byte& b);

    /**
     * Reads bytes from the serial port into a buffer. It terminates if the 
     * determined length has been read, or it times out.
     * \param[in]  length  number of bytes to be read
     * \param[out] buffer  bytes read
     * \return             number of bytes read
     * \sa setTimeout()
     */
    virtual int read (const uint16_t& length, byte *buffer);

    /**
     * Writes a buffer of bytes to the serial port.
     * \param[in]  length  number of bytes to be written
     * \param[out] buffer  bytes to write
     * \return             number of bytes written
     */
    virtual int write (const uint16_t& length, const byte *buffer);

    /**
     * Sets timeout for read operations over serial port.
     * \param ms  timeout in milliseconds
     * \return    true if successful, false otherwise
     */
    virtual bool setTimeout (const uint32_t& ms);

  private:
    unsigned int   _baudrate; /**< serial port baudrate */
    const char*    _port;     /**< serial port name */
    const char*    _config;   /**< serial port configuration */
#if defined(__linux__)
    struct termios _options;  /**< unix termios specific options */
#endif /* defined(__linux__) */
    int            _fd;       /**< serial port descriptor */
    int            _status;   /**< serial port status */
    int            _timeout;  /**< read operations timeout in milliseconds */
};

}

#endif /* B_SERIAL_H */
