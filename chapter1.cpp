#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iostream>

using namespace std;
using namespace cv;



//binary search
int BinarySearch(int MyDot[][4], int MyMatrice[][4]) {
	// number of columns
	int len = sizeof MyMatrice[0] / sizeof(int);
	int DotCoordinate = -1, low = 1;
	int high = len;

	while (low <= high) {
		int mid = (low + high) / 2;
		if (MyMatrice[mid][1] < MyDot[1][1]) { // right half search
			low = mid + 1;
		}
		else if (MyMatrice[mid][1] > MyDot[1][1]) { //left half search
			high = mid - 1;
		}
		else {
			int DotCoordinate[1][2];
			for (int i = 2; i < 4; i++) {
				DotCoordinate[mid][i] = MyMatrice[mid][i];
			}
		}
		return DotCoordinate;
	}



}




// distance calculator
float CalculateDistance(float a[1][2], float b[1][2]) {
	float dist;
	dist = sqrtf(((a[1][1] - b[1][1]) * (a[1][1] - b[1][1]) + (a[1][2] - b[1][2]) * (a[1][1] - b[1][1])));
	return dist;
}






void main() {
	// reading images
	String pathimg1L = "Resources/img1L.png";
	Mat img1L = imread(pathimg1L);

	String pathimg1R = "Resources/img1R.png";
	Mat img1R = imread(pathimg1R);

	String pathimg2L = "Resources/img2L.png";
	Mat img2L = imread(pathimg2L);

	String pathimg2R = "Resources/img2R.png";
	Mat img2R = imread(pathimg2R);

	Mat img1L_gray, img1R_gray, img2L_gray, img2R_gray;

	// converting images to gray
	cvtColor(img1L, img1L_gray, COLOR_BGR2GRAY);
	cvtColor(img1R, img1R_gray, COLOR_BGR2GRAY);
	cvtColor(img2L, img2L_gray, COLOR_BGR2GRAY);
	cvtColor(img2R, img2R_gray, COLOR_BGR2GRAY);
	
	
	// extraxting good featurs
	vector<Point2f> p0, p1;

	goodFeaturesToTrack(img1L_gray, p0, 375, 0.01, 7, Mat(), 7, false, 0.04);
	

	// calculate optical flow 1L 1R
	vector<uchar> status1;
	vector<float> err1;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(img1R_gray, img1L_gray, p0, p1, status1,err1);
	
	//good points selection
	vector<Point2f> good_new1;
	for (uint i = 0; i < p0.size(); i++) {
		//selecting good points
		if (status1[i] == 1) {
			good_new1.push_back(p1[i]);
		}
	}
	// updating p0
	p0 = good_new1;

	
	//optical flow p1, image  2l
	Mat fixed_p1 = Mat(p1);
	



	//good points selection
	vector<Point2f> good_new2;
	for (uint i = 0; i < p0.size(); i++) {
		//selecting good points
		if (status2[i] == 1) {
			good_new2.push_back(p1[i]);
		}
	}
	p0 = good_new2;
	
	//float array[100] = {{p0}, {p1}};

	//optical flow p2, image 2R
	Mat fixed_p2 = Mat(p2);
	vector<Point2f> p3, status3, err3;
	calcOpticalFlowPyrLK(fixed_p2, img2L_gray, p0, p3, status3, err3);
	

	//good points selection
	vector<Point2f> good_new3;
	for (uint i = 0; i < p0.size(); i++) {
		//selecting good points
		if (status3[i] == 1) {
			good_new3.push_back(p1[i]);
		}
	}
	p0 = good_new3;


	//optical flow p3, image 1L
	Mat fixed_p3 = Mat(p3);
	vector<Point2f> p4, status4, err4;
	calcOpticalFlowPyrLK(fixed_p3, img2L_gray, p0, p3, status4,err4);


	//good points selection
	vector<Point2f> good_new4;
	for (uint i = 0; i < p0.size(); i++) {
		//selecting good points
		if (status4[i] == 1) {
			good_new4.push_back(p1[i]);
		}
	}
	p0 = good_new4;




	waitKey(0);
	
	
}




int BinarySearch(int MyDot[], int MyMatrice[])
{
	return 0;
}
