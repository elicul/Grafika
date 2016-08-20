#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <Box2D/Box2D.h>

class ball
{
	public:
		b2Body* body;
	public:
		ball();	
};

void init();
void cam_setup(cv::VideoCapture capture);
void cam_stop(cv::VideoCapture capture);
#endif 