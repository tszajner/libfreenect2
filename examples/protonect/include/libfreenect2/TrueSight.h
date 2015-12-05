#ifndef TRUESIGHT_H
#define TRUESIGHT_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef enum { Edge, IR, Depth } State;

class TrueSight
{
	public:
	//Screen Constants
	static const int STREAM_HEIGHT = 424;
	static const int STREAM_WIDTH = 512;
	static int const LCD_HEIGHT = 480;
	static int const LCD_WIDTH = 800;


	//Blank Display Image to-be-drawn-upon
	cv::Mat Display;

	//Control Variables
	float scale;
	int x;
	int y;
	State DisplayState;

	//Functions
	TrueSight(int16_t x,int16_t y,int16_t z); //Initialize values
	void Calibrate(int16_t x,int16_t y,int16_t z); //Read in values from ADC, use for Drawing			
	cv::Mat Draw(cv::Mat src);

	private:

	//Canny Edge Detection Constants
	static const int canny_ratio = 3;
	static const int canny_threshold = 50;

	//ADC Voltage Divider constants
	static const int minZoomADC = 1056;
	static const int maxZoomADC = 1649;
	static const int ZoomADCRange = maxZoomADC - minZoomADC;
};

#endif
