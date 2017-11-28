#include <opencv2/core/core.hpp>
#include <opencv2\imgcodecs.hpp>
#include <opencv2\stitching.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include "StereoImageSettings.h"
#include "StereoCalibrateSetup.h"

//PCL
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/console/parse.h>
#include <pcl-1.8/pcl/common/transforms.h>


using namespace std;
using namespace cv;

/*------------------------------------------------EQUALIZE 3 CHANNELS FRAMES------------------------------------------------------*/
Mat EqualizeHistogram(Mat image){
	Mat yuv;

	cvtColor(image, yuv, CV_RGB2YCrCb);

	vector<Mat> channels;
	split(yuv, channels);

	//normalize the lighting, definitely keep YUV space, much less noise
	normalize(channels[0], channels[0], 16, 235, CV_MINMAX, CV_8U);

	equalizeHist(channels[0], channels[0]);

	Mat result;
	merge(channels, yuv);

	cvtColor(yuv, result, CV_YCrCb2RGB);

	return result;

}

/*------------------------------------------------FIND DEPTH------------------------------------------------------*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud, cv::Mat ColoredLeft)
{
	/*
	*  Function: Get from a Mat to pcl pointcloud datatype
	*  In: cv::Mat
	*  Out: pcl::PointCloud
	*/
	resize(ColoredLeft, ColoredLeft, ColoredLeft.size() / 2);
	//char pr=100, pg=100, pb=100;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);

	cout << "The number of rows in the point cloud image is :" << OpencVPointCloud.rows << endl;
	cout << "The number of columns in the point cloud image is :" << OpencVPointCloud.cols << endl;
	point_cloud_ptr->width = OpencVPointCloud.rows;
	point_cloud_ptr->height = OpencVPointCloud.cols;
	for (int y = 0; y<OpencVPointCloud.rows; y++)
	{
		for (int x = 0; x < OpencVPointCloud.cols; x++){
			//std::cout<<i<<endl;

			pcl::PointXYZRGB point;
			point.x = -OpencVPointCloud.at<cv::Vec3f>(y, x).val[0];
			//cout << "The x value is " << point.x << endl;
			point.y = OpencVPointCloud.at<cv::Vec3f>(y, x).val[1];
			//cout << "The y value is " << point.y << endl;
			point.z = OpencVPointCloud.at<cv::Vec3f>(y, x).val[2];
			//cout << "The z value is " << point.z << endl;
			//point.a = OpencVPointCloud.at<cv::Vec4b>(y, x)[3];

			if (point.z >= 0.0f )
				continue;

			//point.r = ColoredLeft.at<cv::Vec4b>(y, x)[0];
			//point.g = ColoredLeft.at<cv::Vec4b>(y, x)[1];
			//point.b = ColoredLeft.at<cv::Vec4b>(y, x)[2];

			point.r = 255;
			point.g = 255;
			point.b = 255;

			point_cloud_ptr->push_back(point);
		}


	}

	
	return point_cloud_ptr;

}



int main(int argc, char* argv[])
{

	string file1;
	string file2;

	VideoCapture cap1(1);
	VideoCapture cap2(2);

	if (!cap1.isOpened() || !cap2.isOpened()){
		cout << "Couldn't find the stereo cameras, using test images" << endl;
		cout << "Not enough arguments: " << endl;
		cout << "Use case: " << endl;
		cout << "Executable, Right Eye Stereo Image, Left Eye Stere Image, window size, minimum disparity, number of disparity, texture threshold, uniqueness ratio, prefilter size, prefilter cap," << endl;
		cout << "If only Right eye, left eye as arguments, runs with default paramters" << endl;

		cout << "Using example images" << endl;
		//for now
		file1 = "C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/lefteye.png";
		file2 = "C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/righteye.png";
	}

	//if the video is running run continuely else do the two images
	if (cap1.isOpened() && cap2.isOpened()){
		vector<string> CalibrationImageList;
		
		bool recalibrate = true;
		cout << "Recalibrate camera? y/n" << endl;
		string answer;
		cin >> answer;
		if (answer == "y"){
			recalibrate = true;
		}
		if (answer == "n"){
			recalibrate = false;
		}
		if(answer != "y" && answer != "n"){
			cout << "Input string was incorrect, using default to recalibrate" << endl;
		}

			for (int i = 0; i < 100; i += 2){
				string img1name = "C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/images/snapshot" + to_string(i) + ".png";
				string img2name = "C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/images/snapshot" + to_string(i + 1) + ".png";
				CalibrationImageList.push_back(img1name);
				CalibrationImageList.push_back(img2name);
				if (recalibrate == true){
					waitKey(0);
					Mat frameleft;
					Mat frameright;

					cap1.grab();
					cap2.grab();
					cap1.retrieve(frameleft);
					cap2.retrieve(frameright);


					Mat resultleft = EqualizeHistogram(frameleft);
					Mat resultright = EqualizeHistogram(frameright);


					Size sz1 = resultleft.size();
					Size sz2 = resultright.size();

					Mat im(sz1.height, sz1.width + sz2.width, resultleft.type());
					Mat leftwindow(im, Rect(0, 0, sz1.width, sz1.height));
					resultleft.copyTo(leftwindow);
					Mat rightwindow(im, Rect(sz1.width, 0, sz2.width, sz2.height));
					resultright.copyTo(rightwindow);
					imshow("Calibration Pair", im);


					resize(resultleft, resultleft, resultleft.size() / 2);
					resize(resultright, resultright, resultright.size() / 2);
					imwrite(img1name, resultleft);
					imwrite(img2name, resultright);
				}

			}
			

		StereoCalibrateSetup calibration = StereoCalibrateSetup(9, 6, CalibrationImageList);

		//first time with disparity settings
		Mat left;
		Mat right;

		cap1.grab();
		cap2.grab();
		cap1.retrieve(left);
		cap2.retrieve(right);


		if (left.rows == 0 || right.rows == 0) {
			cout << "There was a problem loading the image frame from stereo video" << endl;
			return -1;
		}


		Mat resultleft = EqualizeHistogram(left);
		Mat resultright = EqualizeHistogram(right);

		Size sz1 = resultleft.size();
		Size sz2 = resultright.size();

		Mat im3(sz1.height, sz1.width + sz2.width, resultleft.type());
		Mat leftwindow(im3, Rect(0, 0, sz1.width, sz1.height));
		resultleft.copyTo(leftwindow);
		Mat rightwindow(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
		resultright.copyTo(rightwindow);

		imshow("Stereo Images", im3);

		StereoImageSettings stereo = StereoImageSettings(left, right, calibration, 1);
		bool first = true;
		for (;;){
			
			Mat left;
			Mat right;

			cap1.grab();
			cap2.grab();
			cap1.retrieve(left);
			cap2.retrieve(right);
			
			
			if (left.rows == 0 || right.rows == 0) {
				cout << "There was a problem loading the image frame from stereo video" << endl;
				return -1;
			}
			
			

			Mat resultleft = EqualizeHistogram(left);
			Mat resultright = EqualizeHistogram(right);

			

			Size sz1 = resultleft.size();
			Size sz2 = resultright.size();

			Mat im3(sz1.height, sz1.width + sz2.width, resultleft.type());
			Mat leftwindow(im3, Rect(0, 0, sz1.width, sz1.height));
			resultleft.copyTo(leftwindow);
			Mat rightwindow(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
			resultright.copyTo(rightwindow);

			imshow("Stereo Images", im3);



			StereoImageSettings stereo = StereoImageSettings(left, right, calibration, 0);
			imshow("Disparity", stereo.normalizeddisplay);
			imshow("Point Cloud", stereo.PointCloudImage);

			//grow the keypoint area

			imwrite("C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/pointcloud.png", stereo.PointCloudImage);

			imshow("CorrectedDisparity", stereo.filteredimage);

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = MatToPoinXYZ(stereo.PointCloudImage, left);
			

			viewer->setBackgroundColor(0, 0, 0);
			viewer->addPointCloud(cloud, "cloud", 0);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", 0);
			viewer->addCoordinateSystem(1.0, "cloud", 0);
			viewer->initCameraParameters();
			pcl::io::savePCDFileASCII("C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/cloud_test.pcd", *cloud);

			viewer->spin();
			
			

			if (waitKey(30) >= 0) break;
		}
	}
	else{
		vector<string> CalibrationImageList;

		
		
		Mat im1 = imread(file1, CV_LOAD_IMAGE_COLOR);
		Mat im2 = imread(file2, CV_LOAD_IMAGE_COLOR);
		
		CalibrationImageList.push_back(file1);
		CalibrationImageList.push_back(file2);

		StereoCalibrateSetup calibration = StereoCalibrateSetup(6, 9, CalibrationImageList);

		if (im1.rows == 0 || im2.rows == 0) {
			cout << "There was a problem loading the image specified" << endl;
			return -1;
		}

		imshow("Right Eye", im1);
		imshow("Left Eye", im2);

		StereoImageSettings stereo = StereoImageSettings(im1, im2, calibration, 1);

		imshow("Disparity", stereo.normalizeddisplay);
		imshow("Point Cloud", stereo.PointCloudImage);

		imwrite("pointcloud.jpg", stereo.PointCloudImage);

		imshow("CorrectedDisparity", stereo.filteredimage);


		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = MatToPoinXYZ(stereo.PointCloudImage, im1);

		//viewer->setBackgroundColor(0, 0, 0);
		//viewer->addPointCloud(cloud, "cloud", 0);
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", 0);
		//viewer->addCoordinateSystem(1.0, "cloud", 0);
		//viewer->initCameraParameters();
		//pcl::io::savePCDFileASCII("C:/Users/lkrasmussen/Documents/StereoDisplacement/x64/Release/cloud_test.pcd", *cloud);

		//viewer->spin();

		waitKey(0);
	}

	

	



	return -1;
}