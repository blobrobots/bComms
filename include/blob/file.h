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
 * \file       file.h
 * \brief      interface for file based communication
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_FILE_H
#define B_FILE_H

#include <blob/types.h>
#include <blob/comms.h>

#if defined(__linux__)
 #include <fstream>
#endif

namespace blob {

/**
 * Implements communication over file.
 */
class File : public Comms::Channel
{
  public:
   
    /**
     * Initializes file.
     * \param path      name of file to be used
     */
    File (const char* path="filename.bin");

    /**
     *  Closes file before destroying file descriptor
     */
    ~File ();

    /**
     * Initializes communication over serial port.
     * \return  true if successful, false otherwise
     */
    virtual bool init ();

    /**
     * Stops communication over file.
     * \return  true if successful, false otherwise
     */
    virtual bool end ();

    /**
     * Indicates whether file is ready or not.
     * \return  true if communications are ready, false otherwise
     */
    virtual bool isReady ();

    /**
     * Provides number of bytes available in file.
     * \return  number of bytes available or -1 if none
     */
    virtual int available ();    
    /**
     * Empties file buffer of available bytes (pointer to eof).
     */
    virtual void flush ();

    /**
     * Provides next byte available in file without removing it from the 
     * internal buffer. Successive calls would provide the same byte.
     * \param[out] b  byte read
     * \return        true if a byte is available and peek is successful, false 
     *                otherwise     
     */
    virtual bool peek (byte& b);
    
    /**
     * Reads next byte available in file.
     * \param[out] b  byte read
     * \return        true if a byte is available and read is successful, false 
     *                otherwise     
     */
    virtual bool read (byte& b);

    /**
     * Reads bytes from the file into a buffer. It terminates if the 
     * determined length has been read, or it times out.
     * \param[in]  length  number of bytes to be read
     * \param[out] buffer  bytes read
     * \return             number of bytes read
     */
    virtual int read (const uint16_t& length, byte *buffer);

    /**
     * Writes a buffer of bytes to the file.
     * \param[in]  length  number of bytes to be written
     * \param[out] buffer  bytes to write
     * \return             number of bytes written
     */
    virtual int write (const uint16_t& length, const byte *buffer);

  private:
    enum {MAX_PATH_LENGTH=256};
    char    _path[MAX_PATH_LENGTH];     /**< file path and name */
#if defined(__linux__)
    std::fstream   _fs;       /**< file stream */
#endif // defined(__linux__)
};

}

#endif /* B_FILE_H */
