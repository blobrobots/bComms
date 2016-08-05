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
 * \file       test_file.cpp
 * \brief      test for file-based comms library
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/file.h"

#if defined(__linux__)

#include <iostream>
#include <unistd.h>
#include <cstring>

int main(int argc, char* argv[])
{
  const char* filename = "file.bin";
  const char* abcd = "abcdefghijklmnñopqrstuvwxyz";
  uint32_t time = 0;

  if(argc > 1)
    filename = argv[1];
  
  blob::File writer(filename);
   
  std::cout << "[blobtest] - starting file test at " << filename << " ..." << std::endl;  
  
  writer.init();
  
  if (!writer.isReady())
  {
    std::cout << "file init error... aborting" << std::endl;
    return -1;
  }

  writer.write(strlen(abcd), (const byte*)abcd);

  writer.end();

  byte result[100];
  byte ch='0';

  blob::File reader(filename);
  reader.init();
  //reader.flush();

  std::cout << "[blobtest] - file reader available bytes = " << reader.available() << std::endl;
  
  if(reader.available() && reader.peek(ch))
    std::cout << "[blobtest] - file reader peek       = " << (char)ch << std::endl;

  std::cout << "[blobtest] - file reader available bytes = " << reader.available() << std::endl;
  std::cout << "[blobtest] - file reader read bytes = " << reader.read(100, result) << std::endl;
  std::cout << "[blobtest] - file reader read result= " << result << std::endl;

  reader.end();
  return 0;
}


#endif //defined(__linux__)

