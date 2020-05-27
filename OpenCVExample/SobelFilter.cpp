#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace cv;
using namespace std;



int main(int argc, char* argv) {

	Mat src, src_gray;  // source, dest
	Mat grad;           // matrix for gradient
	const char* window_name = "Sobel Demo  - Simple edge detector";
	int scale = 1; 
	int delta = 0;
	int ddepth = CV_16S;
	int c; 

   // load a colour image
	//src = imread("starry_night.jpg", IMREAD_COLOR);

	// loading a piston image 
	src = imread("Kolben_IO_01_01.tif", IMREAD_GRAYSCALE);

	// check whether data exists or not
	if (!src.data) 
	{
		return -1;
	}

	// Adding a Gaussian blur
	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);

	// converting it to gray
	//cvtColor(src, src_gray, COLOR_BGR2GRAY);

	//  Create window
	namedWindow(window_name, WINDOW_AUTOSIZE);

	//Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	// gradient x example of sobel and Scharr
	//Scharr(src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);

	// gradient y example of sobel and Scharr
	//Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta);
	Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);

	// Total gradient (approx)
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

	// write a filter that sets the values between +- 20 as 20. 

	imshow(window_name, grad);

	waitKey(0);

	return 0;
}


