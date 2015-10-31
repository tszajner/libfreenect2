/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/jetsonGPIO.h>

#include <string>
#include <unistd.h>

bool protonect_shutdown = false;

typedef enum { IR, Depth, DepthEdge, IREdge } State;

//Some Canny Threshold
int const max_lowThreshold = 100;
int ratio = 3;
//

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(int argc, char *argv[])
{
  

  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";
  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }


  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

  if(dev == 0)
  {
    std::cout << "no device connected or failure opening the default one!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  float scale = 63;

  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
/*
#ifndef LIBFREENECT2_WITH_TEGRA_JPEG_SUPPORT
    cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
#else
    unsigned char **pprgba = reinterpret_cast<unsigned char **>(rgb->data);
    cv::Mat rgba(1080, 1920, CV_8UC4, pprgba[0]);
    cv::Mat bgra(1080, 1920, CV_8UC4);
    cv::cvtColor(rgba, bgra, cv::COLOR_RGBA2BGRA);
    cv::imshow("rgb", bgra);
#endif
*/
    cv::Mat OriginalImage = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f;
    cv::flip(OriginalImage, OriginalImage, -1);
    float height = float(OriginalImage.rows) * (scale / 100.0f);
    float width= float(OriginalImage.cols) * (scale / 100.0f); 
    //std::cout << height << std::endl;
    //std::cout << width  << std::endl; 
    int a, b, c, d;
    a =  (OriginalImage.cols - int(width)) / 2;
    b = OriginalImage.cols - 1 -  (OriginalImage.cols - int(width)) / 2;
    c = (OriginalImage.rows -  int(height)) / 2;
    d = OriginalImage.rows - 1 -  (OriginalImage.rows - int(height)) / 2;

    //std::cout << a << "   " << b << "    " << c << "    " << d << std::endl;
    cv::Mat zoomedImg = OriginalImage(cv::Range(c, d),
                                      cv::Range(a, b));// d'oh
    cv::resize(zoomedImg, zoomedImg,OriginalImage.size()); 
    zoomedImg.convertTo(zoomedImg, CV_8UC1, 255, 0); 
    cv::equalizeHist(zoomedImg, zoomedImg); 
    cv::Canny(zoomedImg, zoomedImg, 80, 240);
    cv::imshow("Kinect", zoomedImg); 
    int key = cv::waitKey(1);
    if ((key & 0xFF) == 113)
    {
        if (scale > 1)
        {
         scale--;
         std::cout << scale << std::endl;
        }
    }

    if ((key & 0xFF) == 97)
    {
        if (scale <=  99)
        {
         scale++;
         std::cout << scale << std::endl;
        }
    }
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  return 0;
}
