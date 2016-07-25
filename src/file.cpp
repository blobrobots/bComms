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
 * \file       serial.cpp
 * \brief      driver for serial port communication
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/file.h"

#if defined(__linux__)

 #include <cstdio>
 #include <cstring>
#if defined(__DEBUG__)
 #include <iostream>
#endif

blob::File::File (const char* path)
{
   strcpy(_path, path);
} // File::File

bool blob::File::init ()
{
  _fs.open (_path, std::fstream::in | std::fstream::out | 
                   std::fstream::app | std::fstream::binary);

  if(!_fs.is_open()) 
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::File::begin] Error opening file: " << _path << std::endl;
#endif
    return false;
  }

  return true;
} // File::init

int blob::File::available ()
{
  if(!isReady())
    return -1;

  long int bytes_avail = -1;
  long int pos = _fs.tellg();
  std::ios_base::seekdir current = std::ios_base::cur;
  _fs.seekg (0, _fs.end);
  bytes_avail = _fs.tellg()-pos;
  _fs.seekg (0, current);

  return bytes_avail;

} // File::available

bool blob::File::end ()
{
  if(!isReady())
    return false;

// close file
  _fs.close();
  return true; 

} // File::end

void blob::File::flush ()
{
  if(!isReady())
    return;

  _fs.seekg (0, _fs.end);

} // File::flush

bool blob::File::peek (byte& b)
{
  if(!isReady())
    return false;

  return _fs.peek();

} // File::peek

bool blob::File::read (byte& b)
{
  if(!isReady())
    return false;

  char res;

  _fs.read (&res,1);
  
  if(!_fs)
    return false;

 #if defined(__DEBUG__)
  std::cout << "r0x" << std::hex << (int)res << std::dec << std::endl;
 #endif // defined(__DEBUG__)
  b = *(byte*)&res;

  return true;
} // File::read

int blob::File::read (const uint16_t& length, byte *buffer)
{
  if(!isReady())
    return -1;

  _fs.read((char *)buffer, length);
  int retval = _fs.gcount();

#if defined(__DEBUG__)  
  for (int i = 0; i < length; i++)
    std::cout << "r0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
#endif // defined(__DEBUG__)

  return retval;

} // File::readBytes

int blob::File::write (const uint16_t& length, const byte *buffer)
{
  if(!isReady())
    return -1;
 #if defined(__DEBUG__)
  for (int i = 0; i < length; i++)
    std::cout << "w0x" << std::hex << (int)buffer[i] << std::dec << std::endl;
 #endif // defined(__DEBUG__)
  _fs.write((char*)buffer, length);

  if(!_fs)
    return -1;
  else
    return length;

} // File::write


bool blob::File::isReady () 
{ 
  if(!_fs)
  {
#if defined(__DEBUG__)
    std::cerr << "[blob::File::isReady] Error fs=false" << std::endl;
#endif // defined(__DEBUG__)
    return false;
  }
  else
    return true;
}
#endif // defined(__linux__)
