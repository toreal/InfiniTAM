

#include "MeshFusion.h"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;


//Ω첿ず2d 팒3D 헕:m_backup(m_latest_paired_corners_curr), newPoints
//ㄷ첿ず2d 팒3D 헕: m_latest_paired_corners_base ,objectPoints
void MeshFusion::goodFeature()
{

	cout << m_latest_paired_corners_curr.size() << ",";
	cout << m_latest_paired_corners_base.size() << endl;

	cout << newPoints.size() << ",";
	cout << objectPoints.size() << endl;


	int  clusterCount = 3;
	//int  sampleCount = 20;
	Mat labels;
	Mat centers;
//	Mat points2 = Mat(newPoints);

	cv::Mat points(m_latest_paired_corners_curr.size(),2, CV_32FC1);

	for (size_t i = 0, end = m_latest_paired_corners_curr.size(); i < end; ++i) {
		points.at<float>(i, 0) = m_latest_paired_corners_curr[i].x;
		points.at<float>(i, 1) = m_latest_paired_corners_curr[i].y;
	//	points.at<float>(i, 2) = newPoints[i].z;
	}


	kmeans(points, clusterCount, labels,
		TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0),
		3, KMEANS_PP_CENTERS, centers);

	//Mat new_image(640,480,CV_8U);
	int cBlockSize = 5;
	Scalar cc1=cv::Scalar(0, 0, 255, 200);
	Scalar cc2 = cv::Scalar(0, 255, 0, 200);
	Scalar cc3 = cv::Scalar(255, 0, 0, 200);
	for (int y = 0; y < points.rows; y++)
	{
		int cluster_idx = labels.at<int>(y, 0);
		 int nx=points.at<float>(y, 0) ;
		 int ny=points.at<float>(y, 1) ;
		 int len = sizeof(long);
		 switch (cluster_idx)
		 {
		 case 0:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc1);
			 break;
		 case 1:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc2);
			 break;
		 case 2:
			 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc3);

		 }
		 //m_normal.data[(nx + ny * 640) * 3] = 254;
		 //m_normal.at<unsigned char>(nx, ny) = 254;
		 //m_normal.at<unsigned char >(nx - 1, ny) = 254;
		 //m_normal.at<unsigned char >(nx+1, ny) = 254;
		 //m_normal.at<unsigned char >(nx, ny - 1) = 254;
		 //m_normal.at<unsigned char >(nx - 1, ny - 1) = 254;
		 //m_normal.at<unsigned char >(nx + 1, ny - 1) = 254;
		 //m_normal.at<unsigned char >(nx, ny + 1) = 254;
		 //m_normal.at<unsigned char >(nx - 1, ny + 1) = 254;
		 //m_normal.at<unsigned char >(nx + 1, ny + 1) = 254;
		
	}
		for (int x = 0; x < points.cols; x++)
		{
			int f1 = centers.at<float>(0, x);
			int f2 = centers.at<float>(1, x);
		//	float f3 = centers.at<float>(2, x);
			m_normal.at<unsigned char >(f1, f2) = 127;
		}


		imshow("tt", m_normal);
		waitKey(0);

	return;
}