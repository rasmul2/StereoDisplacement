#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace std;

class StereoCalibrateSetup{
public: 
	StereoCalibrateSetup(int w, int h, vector<string> imagelist);
	~StereoCalibrateSetup();

	cv::Mat Qcal;
	cv::Mat Rotation;
	cv::Mat Translation;
	cv::Mat p1;
	cv::Mat p2;
	cv::Mat r1;
	cv::Mat r2;
	cv::Mat m1;
	cv::Mat m2;
	cv::Mat d1;
	cv::Mat d2;

private:
	void Calibrate(const vector<string>& imagelist, cv::Size boardSize, bool displayCorners, bool useCalibrated, bool showRectified);
	bool readStringList(const string& filename, vector<string>& l);
};