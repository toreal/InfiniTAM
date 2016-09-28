

#include "MeshFusion.h"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

//目前的2d 與3D 點:m_backup(m_latest_paired_corners_curr), newPoints
//之前的2d 與3D 點: m_latest_paired_corners_base ,objectPoints
void MeshFusion::goodFeature(ITMPose * posd)
{

	cout << m_latest_paired_corners_curr.size() << ",";
	cout << m_latest_paired_corners_base.size() << endl;

	cout << newPoints.size() << ",";
	cout << objectPoints.size() << endl;

	float* cur= mainView->curvature->GetData(MEMORYDEVICE_CPU);
	int w = mainView->curvature->noDims.x;

	int  clusterCount = 3;
	//int  sampleCount = 20;
	Mat labels;
	Mat centers;
//	Mat points2 = Mat(newPoints);

	cv::Mat points(m_latest_paired_corners_curr.size(),2, CV_32FC1);
	cv::Mat newPoints3D(newPoints.size(), 3, CV_32FC1);//存目前的3D點
	cv::Mat objectPoints3D(objectPoints.size(), 3, CV_32FC1);//存之前的3D點
	
	for (size_t i = 0, end = m_latest_paired_corners_curr.size(); i < end; ++i) {
		points.at<float>(i, 0) = m_latest_paired_corners_curr[i].x;
		points.at<float>(i, 1) = m_latest_paired_corners_curr[i].y;
	//	points.at<float>(i, 2) = newPoints[i].z;
	}

	for (size_t i = 0, end = newPoints.size(); i < end; i++) {
		newPoints3D.at<float>(i, 0) = newPoints[i].x;
		newPoints3D.at<float>(i, 1) = newPoints[i].y;
		newPoints3D.at<float>(i, 2) = newPoints[i].z;
	}

	for (size_t i = 0, end = objectPoints.size(); i < end; i++) {
		objectPoints3D.at<float>(i, 0) = objectPoints[i].x;
		objectPoints3D.at<float>(i, 1) = objectPoints[i].y;
		objectPoints3D.at<float>(i, 2) = objectPoints[i].z;
	}

	kmeans(points, clusterCount, labels,
		TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0),
		3, KMEANS_PP_CENTERS, centers);

	//Mat new_image(640,480,CV_8U);
	int cBlockSize = 5;
	Scalar cc0 = cv::Scalar(0, 0, 0, 200);
	Scalar cc1=cv::Scalar(0, 0, 255, 200);
	Scalar cc2 = cv::Scalar(0, 255, 0, 200);
	Scalar cc3 = cv::Scalar(255, 0, 0, 200);

	int red_num = 0, green_num = 0, blue_num = 0; //計算紅色、綠色、藍色區域的3D點個數
	int points_areanum[20];//紀錄3D點所在的顏色區域
	for (int y = 0; y < points.rows; y++)
	{
		int cluster_idx = labels.at<int>(y, 0);
		 int nx=points.at<float>(y, 0) ;
		 int ny=points.at<float>(y, 1) ;
		 cv::Point pt_txt1(nx,ny);

		 float cvalue = cur[nx + ny*w];
		 std::stringstream ssout1;
		 cout << cluster_idx << ":" << cvalue << endl;
		 ssout1 << cluster_idx << ":" << cvalue << endl;
		 cv::putText(m_normal, ssout1.str(), pt_txt1, CV_FONT_HERSHEY_SIMPLEX, 0.3, cc0, 1, LINE_AA, false);
		 int len = sizeof(long);
		 points_areanum[y] = cluster_idx;
		 switch (cluster_idx)
		 {
		 case 0:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc1);
			 red_num++;
			 break;
		 case 1:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc2);
			 green_num++;
			 break;
		 case 2:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc3);
			 blue_num++;
		 }
		
	}
		for (int x = 0; x < points.cols; x++)
		{
			int f1 = centers.at<float>(0, x);
			int f2 = centers.at<float>(1, x);
		//	float f3 = centers.at<float>(2, x);
			m_normal.at<unsigned char >(f1, f2) = 127;
		}
	cv::Mat newPoints_red(red_num, 3, CV_32FC1); //目前的3D點在紅色區域
	cv::Mat newPoints_green(green_num, 3, CV_32FC1);//目前的3D點在綠色區域
	cv::Mat newPoints_blue(blue_num, 3, CV_32FC1);//目前的3D點在藍色區域
	cv::Mat objectPoints_red(red_num, 3, CV_32FC1);//之前的3D點在紅色區域
	cv::Mat objectPoints_green(green_num, 3, CV_32FC1);//之前的3D點在綠色區域
	cv::Mat objectPoints_blue(blue_num, 3, CV_32FC1);//之前的3D點在藍色區域
	cv::Mat newPoints_rand(3, 3, CV_32FC1);//從3個區域各取1個目前的3D點
	cv::Mat objectPoints_rand(3, 3, CV_32FC1);//從3個區域各取1個之前的3D點
	red_num = 0;
	green_num = 0;
	blue_num = 0;
	//將目前的3D點和之前的3D點各自對應不同顏色區域
	for (int i = 0; i < points.rows; i++)
	{
		switch (points_areanum[i])
		{
		case 0:
			if (red_num == 0)
			{
				int nx = points.at<float>(i, 0);
				int ny = points.at<float>(i, 1);
				
				cv::rectangle(m_normal, cv::Rect(nx - cBlockSize-1, ny - cBlockSize-1, cBlockSize * 2 + 3, cBlockSize * 2 + 3), cc0);

			}
				
			newPoints_red.at<float>(red_num, 0) = newPoints3D.at<float>(i, 0);
			newPoints_red.at<float>(red_num, 1) = newPoints3D.at<float>(i, 1);
			newPoints_red.at<float>(red_num, 2) = newPoints3D.at<float>(i, 2);
			objectPoints_red.at<float>(red_num, 0) = objectPoints3D.at<float>(i, 0);
			objectPoints_red.at<float>(red_num, 1) = objectPoints3D.at<float>(i, 1);
			objectPoints_red.at<float>(red_num, 2) = objectPoints3D.at<float>(i, 2);
			red_num++;
			break;
		case 1:
			if (green_num == 1)
			{
				int nx = points.at<float>(i, 0);
				int ny = points.at<float>(i, 1);

				cv::rectangle(m_normal, cv::Rect(nx - cBlockSize - 1, ny - cBlockSize - 1, cBlockSize * 2 + 3, cBlockSize * 2 + 3), cc0);

			}
			newPoints_green.at<float>(green_num, 0) = newPoints3D.at<float>(i, 0);
			newPoints_green.at<float>(green_num, 1) = newPoints3D.at<float>(i, 1);
			newPoints_green.at<float>(green_num, 2) = newPoints3D.at<float>(i, 2);
			objectPoints_green.at<float>(green_num, 0) = objectPoints3D.at<float>(i, 0);
			objectPoints_green.at<float>(green_num, 1) = objectPoints3D.at<float>(i, 1);
			objectPoints_green.at<float>(green_num, 2) = objectPoints3D.at<float>(i, 2);
			green_num++;
			break;
		case 2:
			if (blue_num == 1)
			{
				int nx = points.at<float>(i, 0);
				int ny = points.at<float>(i, 1);

				cv::rectangle(m_normal, cv::Rect(nx - cBlockSize - 1, ny - cBlockSize - 1, cBlockSize * 2 + 3, cBlockSize * 2 + 3), cc0);

			}
			newPoints_blue.at<float>(blue_num, 0) = newPoints3D.at<float>(i, 0);
			newPoints_blue.at<float>(blue_num, 1) = newPoints3D.at<float>(i, 1);
			newPoints_blue.at<float>(blue_num, 2) = newPoints3D.at<float>(i, 2);
			objectPoints_blue.at<float>(blue_num, 0) = objectPoints3D.at<float>(i, 0);
			objectPoints_blue.at<float>(blue_num, 1) = objectPoints3D.at<float>(i, 1);
			objectPoints_blue.at<float>(blue_num, 2) = objectPoints3D.at<float>(i, 2);
			blue_num++;
			break;
		}
	}
	int count = 0;
	cv::Mat Transform;
	//從3個顏色區域各取1點做rigid_transformPose
	//for (int i = 0; i < 1; i++) {
	//	newPoints_rand.at<float>(0, 0) = newPoints_red.at<float>(i, 0);
	//	newPoints_rand.at<float>(0, 1) = newPoints_red.at<float>(i, 1);
	//	newPoints_rand.at<float>(0, 2) = newPoints_red.at<float>(i, 2);
	//	objectPoints_rand.at<float>(0, 0) = objectPoints_red.at<float>(i, 0);
	//	objectPoints_rand.at<float>(0, 1) = objectPoints_red.at<float>(i, 1);
	//	objectPoints_rand.at<float>(0, 2) = objectPoints_red.at<float>(i, 2);
	//	for (int j = 1; j < 2; j++) {
	//		newPoints_rand.at<float>(1, 0) = newPoints_green.at<float>(j, 0);
	//		newPoints_rand.at<float>(1, 1) = newPoints_green.at<float>(j, 1);
	//		newPoints_rand.at<float>(1, 2) = newPoints_green.at<float>(j, 2);
	//		objectPoints_rand.at<float>(1, 0) = objectPoints_green.at<float>(j, 0);
	//		objectPoints_rand.at<float>(1, 1) = objectPoints_green.at<float>(j, 1);
	//		objectPoints_rand.at<float>(1, 2) = objectPoints_green.at<float>(j, 2);
	//		for (int k = 1; k < 2; k++) {
	//			newPoints_rand.at<float>(2, 0) = newPoints_blue.at<float>(k, 0);
	//			newPoints_rand.at<float>(2, 1) = newPoints_blue.at<float>(k, 1);
	//			newPoints_rand.at<float>(2, 2) = newPoints_blue.at<float>(k, 2);
	//			objectPoints_rand.at<float>(2, 0) = objectPoints_blue.at<float>(k, 0);
	//			objectPoints_rand.at<float>(2, 1) = objectPoints_blue.at<float>(k, 1);
	//			objectPoints_rand.at<float>(2, 2) = objectPoints_blue.at<float>(k, 2);
	//			Transform[count] = rigid_transformPose(objectPoints_rand, newPoints_rand,posd);
	//			cout << count << ":" << i << "," << j << "," << k << endl;
	//			count++;
	//		}
	//	}
	//}

	Transform = rigid_transformPose(objectPoints_rand, newPoints_rand, posd);
//	for (int i = 0; i < count; i++) {
		cout << "Transform = \n" << Transform << endl;
//	}
	
		imshow("tt", m_normal);
		waitKey(0);

	return;
}
