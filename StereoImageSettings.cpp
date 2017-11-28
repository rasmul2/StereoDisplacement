#include "StereoImageSettings.h"


StereoImageSettings::StereoImageSettings(cv::Mat image1, cv::Mat image2, StereoCalibrateSetup cal, int settings)
{

	savedImage1 = cv::Mat();
	savedImage2 = cv::Mat();

	cv::Mat leftcolored = image1;
	cv::Mat rightcolored = image2;

	//create and assign all the variables for the class to use
	cvtColor(image1, savedImage1, CV_BGR2GRAY);
	cvtColor(image2, savedImage2, CV_BGR2GRAY);

	//psssibly downscale images, listed improvement
	resize(savedImage1, savedImage1, savedImage1.size() / 2);
	resize(savedImage2, savedImage2, savedImage2.size() / 2);



	cout << "Made it into initialization" << endl;
	//set the stereoBMState with the specified paramaters
	int PreFilterType = 0;
	int PreFilterSize = 5;
	int PreFilterCap = 0;
	int MinDisparity = 0;
	int TextureThreshold = 0;
	int SpeckleRange = 0;
	int SpackleWindowSize = 0; 
	int UniqnessRatio = 0;
	int SADWindowSize = 5; 
	int numDisparities = 16; 
	
	sbm = cv::StereoBM::create(112, 19);
	assert(sbm != NULL);

	//set for now, we should pass this so that we can skip it after the first setting or automate it
	if (settings == 1){

		while (settings)
		{


			if (SADWindowSize % 2 == 0)
			{
				SADWindowSize = SADWindowSize + 1;
			}

			if (SADWindowSize < 5)
			{
				SADWindowSize = 5;
			}


			if (numDisparities % 16 != 0)
			{
				numDisparities = numDisparities + (16 - numDisparities % 16);
			}
			if (PreFilterSize < 5){
				PreFilterSize = 5;
			}

			if (PreFilterSize % 2 == 0){
				PreFilterSize += 1;
			}

			sbm->setPreFilterType(PreFilterType);
			sbm->setPreFilterSize(PreFilterSize);
			sbm->setPreFilterCap(PreFilterCap + 1);
			sbm->setMinDisparity(MinDisparity - 100);
			sbm->setTextureThreshold(TextureThreshold*0.0001);
			sbm->setSpeckleRange(SpeckleRange);
			sbm->setSpeckleWindowSize(SpackleWindowSize);
			sbm->setUniquenessRatio(0.01*UniqnessRatio);
			sbm->setSmallerBlockSize(15);
			sbm->setDisp12MaxDiff(32);
			sbm->setNumDisparities(numDisparities);

			cv::namedWindow("Track Bar Window", CV_WINDOW_NORMAL);
			cvCreateTrackbar("Pre Filter Type", "Track Bar Window", &PreFilterType, 1, 0);
			cvCreateTrackbar("Pre Filter Size", "Track Bar Window", &PreFilterSize, 255);
			cvCreateTrackbar("Pre Filter Cap", "Track Bar Window", &PreFilterCap, 61);
			cvCreateTrackbar("Minimum Disparity", "Track Bar Window", &MinDisparity, 200);
			cvCreateTrackbar("Uniqueness Ratio", "Track Bar Window", &UniqnessRatio, 2500);
			cvCreateTrackbar("Texture Threshold", "Track Bar Window", &TextureThreshold, 10000);
			cvCreateTrackbar("Speckle Range", "Track Bar Window", &SpeckleRange, 500);
			cvCreateTrackbar("Block Size", "Track Bar Window", &SADWindowSize, 100);
			cvCreateTrackbar("Speckle Window Size", "Track Bar Window", &SpackleWindowSize, 200);
			cvCreateTrackbar("Number of Disparity", "Track Bar Window", &numDisparities, 500);

			cv::Mat test = cv::Mat(); 
			sbm->compute(savedImage1, savedImage2, test);
			cv::Mat normalized = cv::Mat();
			normalize(test, normalized, 0, 255, CV_MINMAX, CV_8U);
			imshow("Disparity Test", normalized);

			test.release();
			normalized.release();

			char key = (char)cv::waitKey(30);   // explicit cast
			if (key == 27) break;                // break if `esc' key was pressed. 
		}
	}


	//set and create filter as well
	wls_filter = cv::ximgproc::createDisparityWLSFilter(sbm);
	wls_filter->setLambda(500);
	wls_filter->setSigmaColor(0.5);
 

	PointCloudImage = cv::Mat();
	displayleft = cv::Mat();
	displayright = cv::Mat();
	normalizeddisplay = cv::Mat();

	cout << "About to calculate stereo" << endl;
	CalculateStereo(leftcolored, rightcolored, cal);
	
}

void StereoImageSettings::CalculateStereo(cv::Mat color1, cv::Mat color2, StereoCalibrateSetup calibration) {

	//compute and normalize disparity image
	//improvement, should compute for left to right and right to left to perform filtering afterwards
	int matching_time = (double)cv::getTickCount();
	savedImage1.convertTo(savedImage1, CV_8UC1);
	savedImage2.convertTo(savedImage2, CV_8UC1);


	//this shit fixes it pretty damn well
	cv::Mat filter1, filter2;
	cv::bilateralFilter(savedImage1, filter1, 9, 150, 150);
	cv::bilateralFilter(savedImage2, filter2, 9, 150, 150);

	imshow("filter1", filter1);

	sbm->compute(filter1, filter2, displayleft);
	sbm->compute(filter2, filter1, displayright);
	matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();

	cout << "The matching time was " << matching_time << endl; 

	//normalize for display
	normalize(displayleft, normalizeddisplay, 0, 255, CV_MINMAX, CV_8U);

	//filter
	cv::Mat filtered;
	wls_filter->filter(displayleft, savedImage1, filtered, displayright);
	
	
	cv::ximgproc::getDisparityVis(filtered, filteredimage);
	cout << "The time it took to match the images is: " << matching_time << endl;

	
	//negative seems to work, idk why
	cv::Mat Q = calibration.Qcal;
	//Q.convertTo(Q, CV_64FC1, (double) 1. / 16);

	cv::reprojectImageTo3D(filtered, PointCloudImage, Q, false, -1);
	//normalize(filtered, filtered, 0, 255, CV_MINMAX, CV_8U);
	//cvtColor(PointCloudImage, PointCloudImage, CV_BGR2GRAY);
	//PointCloudImage.convertTo(PointCloudImage, CV_64FC1);
	
}

StereoImageSettings::~StereoImageSettings()
{
	sbm->clear();
	displayleft.release();
	displayright.release();
	normalizeddisplay.release();
}

