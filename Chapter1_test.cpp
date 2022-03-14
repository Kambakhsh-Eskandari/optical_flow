#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace cv;

float angle(float x0, float y0, float x1, float y1);


bool binarysearch_x(float x, vector<Point2f> vec);
bool binarysearch_y(float y, vector<Point2f> vec);
	

void main() {
	// reading images----------------------------------------------------
	String pathimg1L = "Resources/img1L.png";
	Mat img1L = imread(pathimg1L);

	String pathimg1R = "Resources/img1R.png";
	Mat img1R = imread(pathimg1R);

	String pathimg2L = "Resources/img2L.png";
	Mat img2L = imread(pathimg2L);

	String pathimg2R = "Resources/img2R.png";
	Mat img2R = imread(pathimg2R);

	Mat img1L_gray, img1R_gray, img2L_gray, img2R_gray;
	//--------------------------------------------------------------------
	// 
	// converting images to gray-------------------------------------------
	cvtColor(img1L, img1L_gray, COLOR_BGR2GRAY);
	cvtColor(img1R, img1R_gray, COLOR_BGR2GRAY);
	cvtColor(img2L, img2L_gray, COLOR_BGR2GRAY);
	cvtColor(img2R, img2R_gray, COLOR_BGR2GRAY);
	//-----------------------------------------------------------------
	
	// Create random colors--------------------------------------------
	vector<Scalar> colors;
	RNG rng;
	for (int i = 0; i < 100; i++)
	{
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}
	//---------------------------------------------------------------------------------------------------------------------------------------------

	


	// extraxting good featurs
	vector<Point2f> p0, p1;

	goodFeaturesToTrack(img1L_gray, p0, 1000000, 0.01, 7,Mat(),7,false,0.04);
	
	
	// Create a mask image for drawing purposes---------------------------------------------------------------------------------------------------------
	Mat mask = Mat::zeros(img1L_gray.size(), img1L_gray.type());

	

	// Create a mask image for drawing purposes
	Mat mask = Mat::zeros(img1L_gray.size(), img1L_gray.type());
	



	
	
	// 1) calculate optical flow 1L 1R
	vector<uchar> status1;
	vector<float> err1;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(img1L_gray, img1R_gray, p0, p1, status1, err1,Szie(15,3),2,criteria);
	



	//good points selection for p1
	vector<Point2f> good_new1;
	for (uint i = 0; i < p0.size(); i++) {
		//selecting good points
		
		
		float Angle1; 
		Angle1 = angle(p0[i].x, p0[i].y, p1[i].x, p1[i].y);
		if (Angle1 > 177 || Angle1 < 183){
		if (status1[i] == 1) {
				good_new1.push_back(p1[i]);
			}
		}
	}


	// Visualization part
	for (uint i = 0; i < p0.size(); i++)
	{
		
			// Draw the tracks
			line(mask, p1[i], p0[i], colors[i], 1);
			
		}



	// Display the demo
	Mat img;
	add(img1R_gray, mask, img);
	imshow("flow", img);
	waitKey(0);

	// updating p1
	vector<Point2f> p1_for_opticalFlow_2;
	p1_for_opticalFlow_2= good_new1;
	
	
	// 2) calculating optical flow 1R 2R
	vector<uchar> status2;
	vector<float> err2;
	vector<Point2f> p2;
	calcOpticalFlowPyrLK(img1R_gray, img2R_gray, p1_for_opticalFlow_2, p2, status2, err2);
	
	//good point selections for p2
	vector<Point2f> good_new2;
	for (uint i = 0; i < p1_for_opticalFlow_2.size(); i++) {
		//selecting good points
		float Angle2;
		Angle2 = angle(p1_for_opticalFlow_2[i].x, p1_for_opticalFlow_2[i].y, p2[i].x, p2[i].y);

			if (Angle2 > 177 || Angle2 < 183){
				if (status2[i] == 1) {
					good_new2.push_back(p2[i]);
				}
		}
	}
	// updating p2
	vector<Point2f> p2_for_opticalFlow_3;
	p2_for_opticalFlow_3 = good_new2;
	

	// 3) calculating optical flow 2R 2L
	vector<uchar> status3;
	vector<float> err3;
	vector<Point2f> p3;
	calcOpticalFlowPyrLK(img2R_gray, img2L_gray, p2_for_opticalFlow_3, p3, status3, err3);

	//good point selections for p3
	vector<Point2f> good_new3;
	for (uint i = 0; i < p2_for_opticalFlow_3.size(); i++) {
		//selecting good points
		float Angle3;
		Angle3 = angle(p2_for_opticalFlow_3[i].x, p2_for_opticalFlow_3[i].y, p3[i].x, p3[i].y);
		if (Angle3 > 177 || Angle3 <183) {
			if (status3[i] == 1) {
				good_new3.push_back(p3[i]);
			}
		}
	}
	// updating p3
	vector<Point2f> p3_for_opticalFlow_4;
	p3_for_opticalFlow_4 = good_new3;
	

	// 4) calculating optical flow 2L 1L
	vector<uchar> status4;
	vector<float> err4;
	vector<Point2f> p4;
	calcOpticalFlowPyrLK(img2L_gray, img1L_gray, p3_for_opticalFlow_4, p4, status4, err4);

	//good point selections for p4
	vector<Point2f> good_new4;
	for (uint i = 0; i < p3_for_opticalFlow_4.size(); i++) {
		//selecting good points
		float Angle4;
		Angle4 = angle(p3_for_opticalFlow_4[i].x, p3_for_opticalFlow_4[i].y, p4[i].x, p4[i].y);
		if (Angle4 > 177 || Angle4 < 183) {

			if (status4[i] == 1) {
				good_new4.push_back(p4[i]);
			}
		}
	}
	// updating p4
	vector<Point2f> p4_for_opticalFlow_4;
	p4_for_opticalFlow_4 = good_new4;
	//cout << p0.size() << endl << p1_for_opticalFlow_2.size() << endl << p2_for_opticalFlow_3.size() << endl << p3_for_opticalFlow_4.size()<< endl << p4_for_opticalFlow_4.size();

	


	/*
	//cout << endl;
	//sorting p0
	vector<Point2f> p0_sorted = p0;

	sort(p0_sorted.begin(), p0_sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		if (a.x != b.x) {
			return (a.x < b.x);
		}
		else {
			return (a.y < b.y);
		}
		});
	

	//sorting p1_for_opticalFlow_2
	
	vector<Point2f> p1_for_opticalFlow_2_sorted = p1_for_opticalFlow_2;

	sort(p1_for_opticalFlow_2_sorted.begin(), p1_for_opticalFlow_2_sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		if (a.x != b.x) {
			return (a.x < b.x);
		}
		else {
			return (a.y < b.y);
		}
		});
	//sorting p2_for_opticalFlow_3_sorted
	vector<Point2f> p2_for_opticalFlow_3_sorted = p2_for_opticalFlow_3;
	
	sort(p2_for_opticalFlow_3_sorted.begin(), p2_for_opticalFlow_3_sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		if (a.x != b.x) {
			return (a.x < b.x);
		}
		else {
			return (a.y < b.y);
		}
		});
	//sorting p3_for_opticalFlow_4_sorted
	
	vector<Point2f> p3_for_opticalFlow_4_sorted = p3_for_opticalFlow_4;

	sort(p3_for_opticalFlow_4_sorted.begin(), p3_for_opticalFlow_4_sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		if (a.x != b.x) {
			return (a.x < b.x);
		}
		else {
			return (a.y < b.y);
		}
		});
	// sorting p4_for_opticalFlow_4
	vector<Point2f> p4_for_opticalFlow_4_sorted = p4_for_opticalFlow_4;
	sort(p4_for_opticalFlow_4_sorted.begin(), p4_for_opticalFlow_4_sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		if (a.x != b.x) {
			return (a.x < b.x);
		}
		else {
			return (a.y < b.y);
		}
		});
		*/
	/*
	cout << p1_for_opticalFlow_2_sorted[0].y;
	int table[1000][6] = { 0 };
	for (int i = 0; i < 1000; i++) {
		table[i][0] = 1;
	}
	
	for (int i = 0; i < p1.size(); i++) {
		bool binary_x = binarysearch_x(p1[i].x, p1_for_opticalFlow_2_sorted);
		bool binary_y = binarysearch_y(p1[i].y, p1_for_opticalFlow_2_sorted);
		if ((binary_x == 1) && (binary_y==1)) {
			table[i][1] = 1;
		}
	}
	//cout << 1;
	for (int i = 0; i < p2.size(); i++) {
		bool binary_x = binarysearch_x(p2[i].x, p2_for_opticalFlow_3_sorted);
		bool binary_y = binarysearch_y(p2[i].y, p2_for_opticalFlow_3_sorted);
		if ((binary_x == 1) && (binary_y == 1)) {
			table[i][2] = 1;
		}
	}


	for (int i = 0; i < p3.size(); i++) {
		bool binary_x = binarysearch_x(p3[i].x, p3_for_opticalFlow_4_sorted);
		bool binary_y = binarysearch_y(p3[i].y, p3_for_opticalFlow_4_sorted);
		if ((binary_x == 1) && (binary_y == 1)) {
			table[i][3] = 1;
		}
	}
	
	for (int i = 0; i < p4.size(); i++) {
		bool binary_x = binarysearch_x(p4[i].x, p4_for_opticalFlow_4_sorted);
		bool binary_y = binarysearch_y(p4[i].y, p4_for_opticalFlow_4_sorted);
		if ((binary_x == 1) && (binary_y == 1)) {
			table[i][4] = 1;
		}
	}

	for (int i = 0; i < p4_for_opticalFlow_4.size(); i++) {
		bool binary_x = binarysearch_x(p4_for_opticalFlow_4[i].x, p0_sorted);
		bool binary_y = binarysearch_y(p4_for_opticalFlow_4[i].y, p0_sorted);
		if ((binary_x == 1) && (binary_y == 1)) {
			table[i][5] = 1;
		}
	}

	//for (int i = 0; i < 1000; i++) {
	//	cout << table[i][0] << " " << table[i][1] << " " << table[i][2] << " " << table[i][2] << " " << table[i][4] << " " << table[i][5] << endl;
	//}

	*/
}

float angle(float x1, float y1, float x2, float y2)
{
	int deltaY = y2 - y1;
	int deltaX = x2 - x1; 
	float angleInDegrees = atan2(deltaY, deltaX) * 180 / 3.141;
	return angleInDegrees;
}



bool binarysearch_x(float x, vector<Point2f> vec) {
	int low_x = 0, high_x = vec.size();
	while (low_x < high_x) {
		int mid_x = (low_x + high_x) / 2;
		
		if (x < vec[mid_x].x) {
			high_x = mid_x - 1;
		}
		else if (x > vec[mid_x].x) {
			low_x = mid_x - 1;
		}
		
		else {
			if (x == vec[mid_x].x) {
				return true;
			}
		}
	}
	return false;
}

bool binarysearch_y(float y, vector<Point2f> vec) {
	int low_y = 0, high_y = vec.size();
	while (low_y < high_y) {
		int mid_y = (low_y + high_y) / 2;

		if (y < vec[mid_y].y) {
			high_y = mid_y - 1;
		}
		else if (y > vec[mid_y].y) {
			low_y = mid_y - 1;
		}

		else {
			if (y == vec[mid_y].y) {
				return true;
			}
		}
	}
	return false;
}
