// Code for priniting the pixel vales of an image, our case: piston


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {

	// in case a very big image take a small snapshot
	Mat source = imread("Kolben_IO_01_01.tif", IMREAD_GRAYSCALE);
	// try reading the image for a specific roi
	// calculate the no. of rows and cols

	// check if image is empty or not
	if (source.empty()){
		
		cout<< "Error, no image"<< endl;
			return -1;
	}
	// Select ROI
	//Rect2d r = selectROI(inputImg);
	Rect roiToSelect(600, 620, 3, 3);



	// ROI image
	Mat roi = source(roiToSelect);


	namedWindow("Display window with Kolben", WINDOW_AUTOSIZE);
	imshow("ROI Piston image", roi);

	// print number of channels, add check for 1 or 3 channel 

	cout << "image channel\t" << source.channels() << endl;
	// print the no. of rows & cols of ROI 
	cout << "rows   \t" << roi.rows << endl;
	cout << "columns \t" << roi.cols << endl;

	// if image has 1 channel 
	if (roi.channels() == 1) {
		for (int y = 0; y < roi.cols; y++) {
			for (int x = 0; x < roi.rows; x++) {

				Scalar pixel = roi.at<uchar>(x,y);
				cout << pixel.val[0] << ", ";

			}
			cout << endl;
		}

	}
	else if (roi.channels() == 3) {
		for (int y = 0; y < roi.cols; y++) {
			for (int x = 0; x < roi.rows; x++) {
				Vec3b pixel = roi.at<Vec3b>(x,y); // 3 channeled image for opencv the channels start at bgr
				int blue = pixel.val[0]; // channel 1
				int green = pixel.val[1]; // channel 2
				int red = pixel.val[2]; // channel 3
				
				cout << "[" << blue << " " << green << " " << red << "],"; 
			}
			cout << endl;
		}		
	}

	else {
		
		cout << "this is not a single channel image" << endl;
	}

	waitKey(0);
	return 0;
}


