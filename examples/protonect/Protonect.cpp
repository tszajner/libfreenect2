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
#include <libfreenect2/Adafruit_ADS1015.h>
#include <libfreenect2/TrueSight.h>

#include <string>
#include <unistd.h>

bool protonect_shutdown = false;
/*
typedef enum { IR, Depth, DepthEdge, IREdge } State;
std::string text = "Infrared";
int fontFace = CV_FONT_HERSHEY_SIMPLEX;
//Some Canny Threshold
int const ratio = 3;
int const cannythresh = 50;
//

//Zoom scale
float scale = 1;
int x = 148;
int y = 28; 
int const STREAM_HEIGHT = 424;
int const STREAM_WIDTH = 512;
int const LCD_HEIGHT = 480;
int const LCD_WIDTH = 800;
int minZoomADC = 1056;
int maxZoomADC = 1649;
int ZoomADCRange = maxZoomADC - minZoomADC;
float minZoomBound = 1 / maxZoomADC - minZoomADC;
int top_border = LCD_HEIGHT - STREAM_HEIGHT / 2;
int side_border = LCD_WIDTH - STREAM_WIDTH / 2;
//int borderType = BORDER_CONSTANT;
*/
void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(int argc, char *argv[])
{
  

  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

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
  //ADC
  Adafruit_ADS1015 ads(0x48);
  ads.setGain(GAIN_ONE);
  ads.setSps(SPS_250);

  //TrueSight
  TrueSight TrueSight(50, 50, 1300);

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

  //Set Up window  
  cv::namedWindow("Kinect", CV_WINDOW_NORMAL);
  //cv::setWindowProperty("Kinect", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

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
    switch (TrueSight.DisplayState)
    {
        case Edge:
	{ 
		cv::imshow("Kinect", TrueSight.Draw(cv::Mat((cv::Mat(ir->height, ir->width, CV_32FC1, ir->data)
												 / 20000.0f))));    
	}       
	break;
        case IR:
	{

		cv::imshow("Kinect", TrueSight.Draw(cv::Mat((cv::Mat(ir->height, ir->width, CV_32FC1, ir->data)
												 / 20000.0f))));    
	}         
	break;
        case Depth:
	{

		cv::imshow("Kinect", TrueSight.Draw(cv::Mat(
					(cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f))));    
	}        
        break;
    }


    int key = cv::waitKey(1);
    if ((key & 0xFF) == 116) // T?
    {
        switch(TrueSight.DisplayState)
        {
            case IR: TrueSight.DisplayState = Depth; break;
            case Depth: TrueSight.DisplayState = Edge; break;
            case Edge: TrueSight.DisplayState = IR; break;
        
        }
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
