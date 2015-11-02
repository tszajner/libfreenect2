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
int const ratio = 3;
int const cannythresh = 50;
//

//Zoom scale
float scale = 100;
int const STREAM_HEIGHT = 424;
int const STREAM_WIDTH = 512;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(int argc, char *argv[])
{
  

  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";
  State stream = IR; 
  /*jetsonGPIO input = gpio166 ;
  gpioExport(input);
  gpioOpen(input);
  gpioSetDirection(input, inputPin);
  //gpioSetEdge(input, "rising");
  unsigned int value = 0;
  unsigned int trigger = 0; */
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
  cv::namedWindow("Kinect", CV_WINDOW_NORMAL);
  
  //int pos, threshold = 0;
  //cv::createTrackbar("EdgeThresholds", "Kinect", &threshold, max_lowThreshold);
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

    float height = float(STREAM_HEIGHT) * (scale / 100.0f); 
    float width = float(STREAM_WIDTH) * (scale / 100.0f);
    int a,b,c,d;
    a = (STREAM_WIDTH - int(width)) / 2;
    b = STREAM_WIDTH - 1 - (STREAM_WIDTH - int(width)) / 2;
    c = (STREAM_HEIGHT - int(height)) / 2;
    d = STREAM_HEIGHT - 1 - (STREAM_HEIGHT - int(height)) / 2;
    switch (stream)
    {
        case IR:
	{

              cv::Mat IR =  cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 4500.0f;
              IR  = IR(cv::Range(c, d), cv::Range(a,b));
              cv::resize(IR, IR, IR.size()); 
              cv::imshow("Kinect", IR);
	}       
	break;
        case Depth:
	{
              cv::Mat Depth =  cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f;
              Depth = Depth(cv::Range(c, d), cv::Range(a,b));
              cv::resize(Depth, Depth, Depth.size()); 
              cv::imshow("Kinect", Depth);
	}         
	break;
        case DepthEdge:
        { 
	       /*cv::Mat DEImage = (cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
               DEImage = DEImage(cv::Range(c, d), cv::Range(a,b));
               cv::resize(DEImage, DEImage, DEImage.size()); 
               DEImage.convertTo(DEImage,CV_8UC1, 255, 0);
               cv::Canny(DEImage, DEImage, cannythresh, cannythresh*ratio);
               cv::imshow("Kinect", DEImage);*/

               cv::Mat IRImage = (cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
               IRImage.convertTo(IRImage,CV_8UC1, 255, 0);
               cv::equalizeHist(IRImage, IRImage);
               cv::blur(IRImage, IRImage, cv::Size(3, 3));
             //  cv::erode(IRImage, IRImage, cv::Mat());  
              // pos = cv::getTrackbarPos("EdgeThresholds", "Kinect"); 
               cv::Canny(IRImage, IRImage, cannythresh ,cannythresh*ratio);
               IRImage = IRImage(cv::Range(c, d), cv::Range(a,b));
               cv::resize(IRImage, IRImage, IRImage.size()); 
               //Image zoom functionality... may want to make this seperate function
    	       //cv::Mat ucharIRImage, ucharIRImageScaled;
               cv::imshow("Kinect", IRImage);
        }
      	break;
        case IREdge:
	{
               cv::Mat IRImage = (cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
               IRImage = IRImage(cv::Range(c, d), cv::Range(a,b));
               cv::resize(IRImage, IRImage, IRImage.size()); 
               //Image zoom functionality... may want to make this seperate function
    	       //cv::Mat ucharIRImage, ucharIRImageScaled;
               IRImage.convertTo(IRImage,CV_8UC1, 255, 0);
               cv::equalizeHist(IRImage, IRImage);
              // cv::blur(IRImage, IRImage, cv::Size(3, 3));
             //  cv::erode(IRImage, IRImage, cv::Mat());  
              // pos = cv::getTrackbarPos("EdgeThresholds", "Kinect"); 
              // cv::Canny(IRImage, IRImage, cannythresh ,cannythresh*ratio);
               cv::imshow("Kinect", IRImage);
	}        
        break;
    }

    int key = cv::waitKey(1);
    //gpioGetValue(input, &trigger);
  // if ((trigger^value) && (trigger))
    if ((key & 0xFF) == 116) // T?
    {
        //gpioSetValue(redLED, on);
        //usleep(1000000);         // on for 200ms
        //gpioSetValue(redLED, off);
        //usleep(200000);         // off for 200ms
        switch(stream)
        {
            case IR: stream = Depth; break;
            case Depth: stream = DepthEdge; break;
            case DepthEdge: stream = IREdge; break;
            case IREdge: stream = IR; break;
        
        }
    } 

   //Zoom Keys
   if ((key & 0xFF) == 113) // Q?
   {
   	if (scale > 1)
	{
		scale--;
	}
   }
   if ((key & 0xFF) == 97) // A?
   {
   	if (scale <= 99)
   	{
        	scale++;
	}
   }

   if ((key & 0xFF) == 114) //R?
   {
        	scale = 100;
   }
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
   // value = trigger;
    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  return 0;
}
