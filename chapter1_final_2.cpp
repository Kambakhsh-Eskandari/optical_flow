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
using namespace std::chrono;

float angle(float x1, float y1, float x2, float y2)
{
	int deltaY = y2 - y1;
	int deltaX = x2 - x1;
	float angleInDegrees = atan2(deltaY, deltaX) * 180 / 3.141;
	return angleInDegrees;
}



void main() {
	

	auto start = high_resolution_clock::now();
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

	// extraxting good featurs
	vector<Point2f> p0;

	goodFeaturesToTrack(img1L_gray, p0, 1000000, 0.01, 0.01, noArray(), 3);

	

	//---------------------------------------------------------------------
	// 1) calculate optical flow 1L 1R
	vector<Point2f> next_points;
	vector<uchar> status;
	vector<float> err;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(img1L_gray, img1R_gray, p0, next_points, status, err, Size(20, 20), 2, criteria);

	vector<Point2f> points_1L1,points_1R1;

	for (size_t i = 0; i < p0.size(); i++) {
		if (status[i] == 1) {
			points_1L1.push_back(p0[i]);
			points_1R1.push_back(next_points[i]);
		}
		else {
			points_1L1.push_back(Point2f(0.f, 0.f));
			points_1R1.push_back(Point2f(0.f, 0.f));
			
		}
	
	}
	
	/*
	for (uint i = 0; i < points_for_op2.size(); i++)
	{


		// Draw the tracks
		if (points_for_op2[i].x != 0.f && points_for_op2[i].y != 0.f) {
			line(img1L, next_points[i], points_for_op2[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/

	//points_for_op2[0] = Point2f(0.3f, 0.f);

	//phase analyse
	//phaseAnalyse(matched_1Li, matched_1Ri, false);
	
	float phase;
	for (size_t i = 0; i < points_1L1.size(); i++) {
		// get phase of the current points
		if (points_1L1[i].x != 0.f && points_1L1[i].y != 0.f) {
			phase = angle(points_1L1[i].x, points_1L1[i].y, points_1R1[i].x, points_1R1[i].y);
		}

		if (phase < 177 || phase > 183) {
			points_1L1[i] = Point2f(0.f, 0.f);
			points_1R1[i] = Point2f(0.f, 0.f);
		}
	}
	
	/*
	for (uint i = 0; i < points_1L1.size(); i++)
	{


		// Draw the tracks
		if (points_1L1[i].x != 0.f && points_1L1[i].y != 0.f) {
			line(img1L, points_1R1[i], points_1L1[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/


	//second optical flow------------------------------------------------------------------------------
	vector<Point2f> next_points2;
	calcOpticalFlowPyrLK(img1R_gray, img2R_gray, points_1R1, next_points2, status, err, Size(20, 20), 2, criteria);

	vector<Point2f> points_1R2, points_2R2;

	for (size_t i = 0; i < points_1R1.size(); i++) {
		if (status[i] == 1) {
			points_1R2.push_back(points_1R1[i]);
			points_2R2.push_back(next_points2[i]);
		}
		else {
			points_1R2.push_back(Point2f(0.f, 0.f));
			points_2R2.push_back(Point2f(0.f, 0.f));

		}

	}
	
	
	/*
	for (uint i = 0; i < points_for_op2.size(); i++)
	{


		// Draw the tracks
		if (points_for_op3[i].x != 0.f && points_for_op3[i].y != 0.f) {
			line(img1L, next_points2_prime[i], points_for_op3[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/

	
	//third optical flow --------------------------------------------------------------------------------------
	vector<Point2f> next_points3;
	calcOpticalFlowPyrLK(img2R_gray, img2L_gray, points_2R2, next_points3, status, err, Size(20, 20), 2, criteria);

	

	vector<Point2f> points_2R3, points_2L3;

	for (size_t i = 0; i < points_2R2.size(); i++) {
		if (status[i] == 1) {
			points_2R3.push_back(points_2R2[i]);
			points_2L3.push_back(next_points3[i]);
		}
		else {
			points_2R3.push_back(Point2f(0.f, 0.f));
			points_2L3.push_back(Point2f(0.f, 0.f));

		}

	}
	
	/*
	for (uint i = 0; i < points_for_op3.size(); i++)
	{
		// Draw the tracks
		if (points_for_op4[i].x != 0.f && points_for_op4[i].y != 0.f) {
			line(img1L, next_points3[i], points_for_op4[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/


	
	// phase analysis
	float phase_prime{};
	for (size_t i = 0; i < points_2R3.size(); i++) {
		// get phase of the current points
		if (points_2R3[i].x != 0.f && points_2R3[i].y != 0.f) {
			phase_prime = angle(points_2R3[i].x, points_2R3[i].y, points_2L3[i].x, points_2L3[i].y);
		}

		if (phase_prime < -3 || phase_prime > 3) {
			points_2R3[i] = Point2f(0.f, 0.f);
			points_2L3[i] = Point2f(0.f, 0.f);

		}
	}
	/*
	for (uint i = 0; i < points_for_op3.size(); i++)
	{
		// Draw the tracks
		if (points_for_op4[i].x != 0.f && points_for_op4[i].y != 0.f) {
			line(img1L, next_points3[i], points_for_op4[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/

	// fourth optical flow ----------------------------------------------------
	vector<Point2f> next_points4;
	calcOpticalFlowPyrLK(img2L_gray, img1L_gray, points_2L3, next_points4, status, err, Size(20, 20), 2, criteria);

	vector<Point2f> points_2L4, points_1L4;
	vector<Point2f> status_present;
	for (size_t i = 0; i < points_2L3.size(); i++) {
		if (status[i] == 1) {
			points_2L4.push_back(points_2L3[i]);
			points_1L4.push_back(next_points4[i]);
			status_present.push_back(Point2f(1.f, 1.f));
		}
		else {
			points_2L4.push_back(Point2f(0.f, 0.f));
			points_1L4.push_back(Point2f(0.f, 0.f));
			status_present.push_back(Point2f(0.f, 0.f));
		}

	}
	/*
	for (uint i = 0; i < points_for_op4.size(); i++)
	{
		// Draw the tracks
		if (points_for_op5[i].x != 0.f && points_for_op5[i].y != 0.f) {
			line(img1L, next_points4_prime[i], points_for_op5[i], Scalar(102, 50, 168));
		}
	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/

	// -----------------------------final draw-------------------------
	/*
	for (int i = 0; i < status_present.size(); i++) {
		cout << status_present[i] << endl;
	}
	*/

	
	/*
	vector<vector<Point2f >> look_up_table;
	size_t row_number = points_1L1.size();
	for (size_t i = 0; i < row_number; i++) {

		vector<Point2f> temp;
		if (status_present[i].x == 1) {
			temp.push_back(points_1L1[i]);

			if (status_present[i].x == 1) {

				temp.push_back(points_1R1[i]);

				if ((status_present[i].x == 1)) {
					temp.push_back(points_1R2[i]);

					if (status_present[i].x == 1) {
						temp.push_back(points_2L3[i]);

						if (status_present[i].x == 1) {
							temp.push_back(points_2L4[i]);

							look_up_table.push_back(temp);

						}
					}
				}
			}
		}
	}
	*/


	vector<vector<Point2f >> look_up_table;
	size_t row_number = points_1L1.size();
	for (size_t i = 0; i < row_number; i++) {

		vector<Point2f> temp;
		if (status_present[i].x == 1) {
				temp.push_back(points_1L1[i]);

			

				temp.push_back(points_1R1[i]);

				
					temp.push_back(points_1R2[i]);

					
						temp.push_back(points_2L3[i]);

						
							temp.push_back(points_2L4[i]);

							look_up_table.push_back(temp);

						
					
				
			
		}
	}

	

	/*
	for (size_t i = 0; i < look_up_table.size(); i++) {
		cout << look_up_table[i][0] << " " << look_up_table[i][1] << " " << look_up_table[i][2] << " " << look_up_table[i][3] << endl;
	}
	*/

	Mat img_visual = imread(pathimg1L);
	for (size_t i = 0; i < look_up_table.size(); i++) {
		line(img_visual, look_up_table[i][1], look_up_table[i][0], Scalar(255,0,0));
		line(img_visual, look_up_table[i][2], look_up_table[i][1], Scalar(255, 0, 0));
		line(img_visual, look_up_table[i][3], look_up_table[i][2], Scalar(255, 0, 0));
		line(img_visual, look_up_table[i][0], look_up_table[i][3], Scalar(255, 0, 0));
	}

	imshow("result", img_visual);
	waitKey(0);
	auto stop = high_resolution_clock::now();


	auto duration = duration_cast<seconds>(stop - start);

	cout << "Time taken by function: "
		<< duration.count() << "seconds" << endl;


	/*
	for (int i = 0; i < 5624; i++) {
		cout << table[i][0] << " " << table[i][1] << " " << table[i][2] << " " << table[i][3] << endl;
	}
	*/
	/*
	for (int i = 0; i < 5624; i++) {
		if (table[i][0] = !0) {
			line(img1L, Point(table[i][2], table[i][3]), Point(table[i][0], table[i][1]), Scalar(255, 0, 0));
		}
	}
	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	*/


	
}


