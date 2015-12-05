#include <libfreenect2/TrueSight.h>


TrueSight::TrueSight(int16_t xpot, int16_t ypot, int16_t zpot)
{
//set some initial values
	DisplayState = Edge;
	Display = cv::Mat(LCD_HEIGHT, LCD_WIDTH, CV_32FC1);
	x = int(xpot);
	y = int(ypot);
	scale = (float(zpot) - float(minZoomADC)) / float(ZoomADCRange); // Make this better
}

void TrueSight::Calibrate(int16_t xpot, int16_t ypot, int16_t zpot)
{
	x = int(xpot);
	y = int(ypot);
	scale = (float(zpot) - float(minZoomADC)) / float(ZoomADCRange); // Make this better
}

cv::Mat TrueSight::Draw(cv::Mat src)
{

	Display = cv::Mat::zeros(LCD_HEIGHT, LCD_WIDTH, CV_32FC1); // Clear Display

	//Pro Algorithm ;p
	float height = float(STREAM_HEIGHT) * (scale); 
	float width = float(STREAM_WIDTH) * (scale);
	int a,b,c,d;
	a = (STREAM_WIDTH - int(width)) / 2;
	b = STREAM_WIDTH - 1 - (STREAM_WIDTH - int(width)) / 2;
	c = (STREAM_HEIGHT - int(height)) / 2;
	d = STREAM_HEIGHT - 1 - (STREAM_HEIGHT - int(height)) / 2;
	src = src(cv::Range(c,d), cv::Range(a,b));
	cv::resize(src, src, cv::Size(STREAM_WIDTH, STREAM_HEIGHT));
	
	//Perform Correct Image Processing
	switch (DisplayState)
	{

		case Edge:
		{
			src.convertTo(src, CV_8UC1, 255, 0);
			cv::equalizeHist(src, src);
			cv::blur(src, src, cv::Size(3, 3));
			cv::Canny(src, src, canny_threshold, canny_threshold*canny_ratio);
			src.copyTo(Display(cv::Rect(x, y, src.cols, src.rows)));

			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Edge", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3);  
		}
		break;

		case IR:
		{
			src.convertTo(src, CV_8UC1, 255, 0);
			cv::equalizeHist(src, src);

			src.copyTo(Display(cv::Rect(x, y, src.cols, src.rows)));
			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Infrared", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3);  
		}
		break;

		case Depth:
		{
			src.copyTo(Display(cv::Rect(x, y, src.cols, src.rows)));

			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Depth", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3);  
		}
		break;
	}
	
	
	return Display;
}



