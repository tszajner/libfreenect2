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
	//Normalization
	x = ((float(xpot) - float(minZoomADC)) / float(ZoomADCRange)) * (LCD_WIDTH - STREAM_WIDTH); 

	y = ((float(ypot) - float(minZoomADC)) / float(ZoomADCRange)) * (LCD_HEIGHT - STREAM_HEIGHT); 

	scale = (float(zpot) - float(minZoomADC)) / float(ZoomADCRange); 

	//Boundaries	
	if (scale > 1)
	{
		scale = 1;
	}

	if (scale < 0.1)
	{
		scale = 0.1;
	}

	if (x > LCD_WIDTH - STREAM_WIDTH)
	{
		x = LCD_WIDTH- STREAM_WIDTH - 1;
	}

	if (x < 0)
	{
		x = 0;
	}

	if (y > LCD_HEIGHT - STREAM_HEIGHT)
	{
		x = LCD_HEIGHT- STREAM_HEIGHT - 1;
	}

	if (y < 0)
	{
		y = 0;
	}
}

cv::Mat TrueSight::Draw(cv::Mat src)
{

	Display = cv::Mat::zeros(LCD_HEIGHT, LCD_WIDTH, CV_8UC1); // Clear Display


	
	//Perform Correct Image Processing
	switch (DisplayState)
	{

		case Edge:
		{
			src.convertTo(src, CV_8UC1, 255, 0);
			cv::equalizeHist(src, src);
			cv::blur(src, src, cv::Size(3, 3));
			cv::Canny(src, src, canny_threshold, canny_threshold*canny_ratio);
			cv::Mat zoom = Zoom(src);
			zoom.copyTo(Display(cv::Rect(x, y, zoom.cols, zoom.rows)));

			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Edge", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3);  
		}
		break;

		case IR:
		{
			src.convertTo(src, CV_8UC1, 255, 0);
			cv::equalizeHist(src, src);

			cv::Mat zoom = Zoom(src);
			zoom.copyTo(Display(cv::Rect(x, y, zoom.cols, zoom.rows)));
			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Infrared", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3);  
		}
		break;

		case Depth:
		{

			cv::Mat zoom = Zoom(src);
			zoom.copyTo(Display(cv::Rect(x, y, zoom.cols, zoom.rows)));
			//Text
			cv::Point textOrg(0, LCD_HEIGHT-50);
              		cv::putText(Display, "Depth", textOrg, CV_FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3);  
		}
		break;
	}
	
	
	return Display;
}


cv::Mat TrueSight::Zoom(cv::Mat src)
{

	//Pro Algorithm ;p
	float height = float(STREAM_HEIGHT) * (scale); 
	float width = float(STREAM_WIDTH) * (scale);
	int a,b,c,d;
	a = (STREAM_WIDTH - int(width)) / 2;
	b = STREAM_WIDTH - 1 - (STREAM_WIDTH - int(width)) / 2;
	c = (STREAM_HEIGHT - int(height)) / 2;
	d = STREAM_HEIGHT - 1 - (STREAM_HEIGHT - int(height)) / 2;
	cv::Mat subsrc = src(cv::Range(c,d), cv::Range(a,b));
	cv::resize(subsrc, subsrc, cv::Size(STREAM_WIDTH, STREAM_HEIGHT));
	
	return subsrc;
}


