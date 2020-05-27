#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

	String imageName("Kolben_IO_01_01.tif"); // image for piston
	if (argc > 1)
	{
		imageName = argv[1];
	}

	// defining a mat image
	Mat image;

	// Reading the file
	image = imread(imageName, IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);

	if (image.empty()) // check for input image validity
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	imwrite("Gray_Kolben_Image.tif", image); // here the no. of bits are lost and the size is of the image is reduced.
	namedWindow("Display window with Kolben", WINDOW_AUTOSIZE); // Create a window for display
	imshow("Display window with piston", image);
	waitKey(0); // Wait for a keystroke in the window.

	return 0;
}



