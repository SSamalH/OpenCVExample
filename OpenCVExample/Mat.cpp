#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>



using namespace cv;
using namespace std;

int main(int argc, char** argv ) {
	

	Mat A, C; // creates the header parts
	A = imread(argv[1], IMREAD_COLOR);  // here the matrix is allocated, we'll know which method is used

	Mat B(A); // Using a copy instructer

	C = A;  // Assignement operator


// The above objects all point to the same single data matrix, modification in any one of the data will modify the other matrix as well. In practise, the different objects provide access to different access
// methods to the same underlying data. 
// The assignment operator and the copy constructor only copies the header.
//	The underlying matrix of an image may be copied using the cv::Mat::clone() and cv::Mat::copyTo() functions.
	

// Creating a ROI in an image to just create a new header with new boundaries
	Mat D(A, Rect(10, 10, 100, 100)); // using a rectangle
	Mat E = A(Range::all(), Range(1,3));  // all the rows of A and cols from 1 to 3 are assigned to E.

	Mat F = A.clone();
	Mat G;
	A.copyTo(G); 

}


