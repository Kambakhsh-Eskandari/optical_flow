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



void getMatchedPoints(
	vector<Point2f>& input_curr_points,
	vector<Point2f>& input_next_points,
	vector<uchar>& status,
	vector<Point2f>& output_curr_points,
	vector<Point2f>& output_next_points) {
	size_t row_number = input_curr_points.size();

	for (size_t i = 0; i < row_number; i++) {
		if (status[i] == 1) {
			output_curr_points.push_back(input_curr_points[i]);
			output_next_points.push_back(input_next_points[i]);
		}
	}

}


//calculate the phase


void phaseAnalyse(vector<Point2f>& input_curr_points,
	vector<Point2f>& input_next_points,
	bool isthirdmatch);


float getphase(Point2f& point0, Point2f& point1);


float angle(float x1, float y1, float x2, float y2)
{
	int deltaY = y2 - y1;
	int deltaX = x2 - x1;
	float angleInDegrees = atan2(deltaY, deltaX) * 180 / 3.141;
	return angleInDegrees;
}



vector<vector<Point2f>> generateLookuptable(
	vector<Point2f>& matched_1Li, vector<Point2f>& matched_1Ri,
	vector<Point2f>& matched_1Rii, vector<Point2f>& matched_2Rii,
	vector<Point2f>& matched_2Riii, vector<Point2f>& matched_2Liii,
	vector<Point2f>& matched_2Liv, vector<Point2f>& matched_1Liv);




bool exists(Point2f& target, vector<Point2f>& vec, int& next_point_index);

void printVectorPoint2f(vector<Point2f>& vec);


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
	cout << img1L.size();
	// extraxting good featurs
	vector<Point2f> initial_feature_points;

	goodFeaturesToTrack(img1L_gray, initial_feature_points, 1000000, 0.01, 0.01, noArray(), 3);


	//---------------------------------------------------------------------
	// 1) calculate optical flow 1L 1R
	vector<Point2f> next_points;
	vector<uchar> status;
	vector<float> err;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(img1L_gray, img1R_gray, initial_feature_points, next_points, status, err, Size(20, 20), 2, criteria);

	
	
	
	
	
	//match points
	vector<Point2f> matched_1Li, matched_1Ri;
	//getMatchedPoints(initial_feature_points, next_points, status, matched_1Li, matched_1Ri);
	getMatchedPoints(initial_feature_points, next_points, status, matched_1Li, matched_1Ri);
	
	for (uint i = 0; i < matched_1Li.size(); i++)
	{


		// Draw the tracks
		line(img1L, matched_1Ri[i], matched_1Li[i], Scalar(102, 50, 168));

	}

	// Display the demo
	imshow("flow1", img1L);
	waitKey(0);
	
	

	//phase analyse
	//phaseAnalyse(matched_1Li, matched_1Ri, false);
	vector<Point2f>output_curr_points, output_next_points;
	float phase;
	for (size_t i = 0; i < matched_1Li.size(); i++) {
		// get phase of the current points
		phase = angle(matched_1Li[i].x, matched_1Li[i].y, matched_1Ri[i].x, matched_1Ri[i].y);
		
		if (phase > 177 && phase < 183) {
			output_curr_points.push_back(matched_1Li[i]);
			output_next_points.push_back(matched_1Ri[i]);
		}
	}
	matched_1Li.shrink_to_fit();
	matched_1Ri.shrink_to_fit();

	matched_1Li = output_curr_points;
	matched_1Ri = output_next_points;

	
	
	
	Mat img1L_prime1 = imread(pathimg1L);

	for (uint i = 0; i < matched_1Li.size(); i++)
	{


		// Draw the tracks
		line(img1L_prime1, output_next_points[i], output_curr_points[i], Scalar(102, 50, 168));

	}

	// Display the demo
	//imshow("flow_first", img1L_prime1);
	//waitKey(0);
	
	
	// -------------------second optical flow----------------------------------------------------

	calcOpticalFlowPyrLK(img1R_gray, img2R_gray, matched_1Ri, next_points, status, err, Size(20, 20), 2, criteria);


	//get match no phase analyze
	vector<Point2f> matched_1Rii, matched_2Rii;
	getMatchedPoints(matched_1Ri, next_points, status, matched_1Rii, matched_2Rii);
	
	
	Mat img1L_prime2 = imread(pathimg1L);
	
	for (uint i = 0; i < matched_1Li.size(); i++)
	{


		// Draw the tracks
		line(img1L_prime2,next_points[i], matched_1Ri[i],  Scalar(102, 50, 168));

	}

	//Display the demo
	imshow("flow_second", img1L_prime2);
	waitKey(0);
	
	

	//---------------------------third optical flow----------------------------

	calcOpticalFlowPyrLK(img2R_gray, img2L_gray, matched_2Rii, next_points, status, err, Size(20, 20), 2, criteria);
	//get matched points
	vector<Point2f> matched_2Riii, matched_2Liii;
	getMatchedPoints(matched_2Rii, next_points, status, matched_2Riii, matched_2Liii);

	


	//phase analyse
	//phaseAnalyse(matched_2Riii, matched_2Liii, true);
	vector<Point2f>output_curr_points2, output_next_points2;
	float phase2;
	for (size_t i = 0; i < matched_2Liii.size(); i++) {
		// get phase of the current points
		phase = angle( matched_2Riii[i].x, matched_2Riii[i].y,matched_2Liii[i].x, matched_2Liii[i].y);

		if (phase > -3 && phase < 3) {
			output_curr_points2.push_back(matched_2Riii[i]);
			output_next_points2.push_back(matched_2Liii[i]);
		}
	}
	
	matched_2Riii.shrink_to_fit();
	matched_2Liii.shrink_to_fit();

	matched_2Riii = output_curr_points2;
	matched_2Liii = output_next_points2;
	
	Mat img1L_prime3 = imread(pathimg1L);

	for (uint i = 0; i < matched_2Riii.size(); i++)
	{


		// Draw the tracks
		line(img1L_prime3, matched_2Liii[i], matched_2Riii[i], Scalar(102, 50, 168));

	}

	// Display the demo
	imshow("flow_third", img1L_prime3);
	waitKey(0);
	/*
	for (size_t i = 0; i < status.size(); i++) {
		if (status[i] == 1) {
			cout << i<<endl;
		}
	}
	*/

	//-----------------------------------optcal flow 4----------------------------------



	calcOpticalFlowPyrLK(img2L_gray, img1L_gray, matched_2Liii, next_points, status, err, Size(20, 20), 2, criteria);

	//match points no phase analyse
	vector<Point2f> matched_2Liv, matched_1Liv;
	getMatchedPoints(matched_2Liii, next_points, status, matched_2Liv, matched_1Liv);

	
	Mat img1L_prime4 = imread(pathimg1L);

	for (uint i = 0; i < matched_2Liv.size(); i++)
	{


		// Draw the tracks
		line(img1L_prime4, matched_1Liv[i], matched_2Liv[i], Scalar(102, 50, 168));

	}

	// Display the demo
	imshow("flow_fourth", img1L_prime4);
	waitKey(0);
	
	

	//--------------------------part two--------------------------------

	//cout << "number of polised points" << matched_1Liv.size();

	vector<vector<Point2f>> table = generateLookuptable(matched_1Li,
		matched_1Ri, matched_1Rii, matched_2Rii, matched_2Riii,
		matched_2Liii, matched_2Liv, matched_1Liv);

	//-------------------------end--------------------------------------
	size_t row_number = table.size();
	/*
	// the resultss of lookup table
	printVectorPoint2f(matched_1Liv);
	cout << row_number << endl;
	for (size_t i = 0; i < row_number; i++)
		printVectorPoint2f(table[i]);
	*/
	
	Mat img_visual = imread(pathimg1L);
	for (size_t i = 0; i < row_number; i++) {
		line(img_visual, table[i][1], table[i][0], Scalar(102, 50, 168));
		line(img_visual, table[i][2], table[i][1], Scalar(255,0,0));
		line(img_visual, table[i][3], table[i][2], Scalar(102, 50, 168));
		line(img_visual, table[i][0], table[i][3], Scalar(255, 0, 0));
	}

	imshow("result", img_visual);
	waitKey(0);


	auto stop = high_resolution_clock::now();


	auto duration = duration_cast<seconds>(stop - start);

	cout << "Time taken by function: "
		<< duration.count() << "seconds" << endl;
}



//phase analyse
void phaseAnalyse(vector<Point2f>& input_curr_points,
	vector<Point2f>& input_next_points,
	bool isthirdmatch)
{
	size_t row_number = input_curr_points.size();
	bool acceptable = false;
	float phase;

	vector<Point2f> output_curr_points, output_next_points;

	if (isthirdmatch) {
		for (size_t i = 0; i < row_number; i++) {
			// get phase of the current points
			phase = getphase(input_curr_points[i], input_next_points[i]);

			if (phase > 360 - 3 || phase < 3) {
				output_curr_points.push_back(input_curr_points[i]);
				output_next_points.push_back(input_next_points[i]);
			}
		}
		input_curr_points.shrink_to_fit();
		input_next_points.shrink_to_fit();

		input_curr_points = output_curr_points;
		input_next_points = output_next_points;
		return;
	}
	for (size_t i = 0; i < row_number; i++) {
		// get phase of the current points
		phase = getphase(input_curr_points[i], input_next_points[i]);

		if (phase > 177 || phase < 183) {
			output_curr_points.push_back(input_curr_points[i]);
			output_next_points.push_back(input_next_points[i]);
		}
	}
	input_curr_points.shrink_to_fit();
	input_next_points.shrink_to_fit();

	input_curr_points = output_curr_points;
	input_next_points = output_next_points;




}






float getphase(Point2f& point0, Point2f& point1) {
	float x0, y0, x1, y1;
	x0 = point0.x;
	y0 = point0.y;
	x1 = point1.x;
	y1 = point1.y;

	// atan 
	if (x1 > x0 && y1 > y0) {
		return atan2(abs(y1 - y0), abs(x1 - x0)) * (180 / 3.1415926);
	}

	if (x1 > x0 && y1 < y0)
		return 360 - (atan2(abs(y1 - y0), abs(x1 - x0)) * (180 / 3.1415926));

	if (x1 < x0 && y1 > y0)
		return 180 - (atan2(abs(y1 - y0), abs(x1 - x0)) * (180 / 3.1415926));

	return 180 + (atan2(abs(y1 - y0), abs(x1 - x0)) * (180 / 3.1415926));
}





// look up table
vector<vector<Point2f>> generateLookuptable(
	vector<Point2f>& matched_1Li, vector<Point2f>& matched_1Ri,
	vector<Point2f>& matched_1Rii, vector<Point2f>& matched_2Rii,
	vector<Point2f>& matched_2Riii, vector<Point2f>& matched_2Liii,
	vector<Point2f>& matched_2Liv, vector<Point2f>& matched_1Liv) {
	//check if the initital points are in next lines
	vector<vector<Point2f >> output_vec;
	size_t row_number = matched_1Li.size();
	for (size_t i = 0; i < row_number; i++) {

		int next_point_index1;
		vector<Point2f> temp;
		temp.push_back(matched_1Li[i]);

		if (exists(matched_1Ri[i], matched_1Rii, next_point_index1)) {
			int next_point_index2;
			temp.push_back(matched_1Ri[i]);
			if (exists(matched_2Rii[next_point_index1], matched_2Riii, next_point_index2)) {

				int next_point_index3;
				temp.push_back(matched_2Rii[next_point_index1]);

				if (exists(matched_2Liii[next_point_index2], matched_2Liv, next_point_index3)) {
					temp.push_back(matched_2Liii[next_point_index2]);
					temp.push_back(matched_1Liv[next_point_index3]);
					output_vec.push_back(temp);
				}

			}
		}
	}
	return output_vec;
}



bool exists(Point2f& target, vector<Point2f>& vec, int& next_point_index) {
	size_t row_number = vec.size();
	for (size_t i = 0; i < row_number; i++) {
		if (vec[i] == target) {
			next_point_index = i;
			return true;
		}
	}
	return false;
}


void printVectorPoint2f(vector<Point2f>& vec) {
	cout << endl;
	size_t size = vec.size();
	for (size_t i = 0; i < size; i++)
		cout << vec[i] << endl;
	cout << endl;
}