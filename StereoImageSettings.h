#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "opencv2\ximgproc\disparity_filter.hpp"
#include <opencv2\imgcodecs.hpp>
#include "StereoCalibrateSetup.h"
#include <opencv2/highgui/highgui.hpp>


using namespace std;

class StereoImageSettings
{
public:
	StereoImageSettings(cv::Mat image1, cv::Mat image2, StereoCalibrateSetup cal, int settings);
	~StereoImageSettings();

	cv::Mat normalizeddisplay;
	cv::Mat PointCloudImage;
	cv::Mat filteredimage;
	
protected:
	cv::Mat savedImage1;
	cv::Mat savedImage2;

	
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter; 
	cv::Ptr<cv::StereoBM> sbm;
	cv::Mat displayleft;
	cv::Mat displayright;
	

	void CalculateStereo(cv::Mat color1, cv::Mat color2, StereoCalibrateSetup calibration);
};


