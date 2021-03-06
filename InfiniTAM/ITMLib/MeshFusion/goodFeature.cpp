

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
	int nnode = m_latest_paired_corners_curr.size();


	cv::Mat newPoints_red(nnode, 3, CV_32FC1); //目前的3D點在紅色區域
	cv::Mat newPoints_green(nnode, 3, CV_32FC1);//目前的3D點在綠色區域
	cv::Mat newPoints_blue(nnode, 3, CV_32FC1);//目前的3D點在藍色區域
	cv::Mat objectPoints_red(nnode, 3, CV_32FC1);//之前的3D點在紅色區域
	cv::Mat objectPoints_green(nnode, 3, CV_32FC1);//之前的3D點在綠色區域
	cv::Mat objectPoints_blue(nnode, 3, CV_32FC1);//之前的3D點在藍色區域

	cv::Mat newPoints_rand(3, 3, CV_32FC1);//從3個區域各取1個目前的3D點
	cv::Mat objectPoints_rand(3, 3, CV_32FC1);//從3個區域各取1個之前的3D點

	cout << nnode << ",";
	cout << m_latest_paired_corners_base.size() << endl;

	cout << newPoints.size() << ",";
	cout << objectPoints.size() << endl;

	if (newPoints.size() != nnode)
		return;
	if (objectPoints.size() != nnode)
		return;
	
	Vector4u* segimg = segImage->GetData(MEMORYDEVICE_CPU);

	float* cur= mainView->curvature->GetData(MEMORYDEVICE_CPU);
	int w = mainView->curvature->noDims.x;
	int h = mainView->curvature->noDims.y;

	int  clusterCount = 3;
	//int  sampleCount = 20;
	Mat labels;
	Mat centers;
//	Mat points2 = Mat(newPoints);

	cv::Mat points(nnode,2, CV_32FC1);
	cv::Mat newPoints3D(nnode, 3, CV_32FC1);//存目前的3D點
	cv::Mat objectPoints3D(nnode, 3, CV_32FC1);//存之前的3D點
	
	for (size_t i = 0, end = nnode; i < end; ++i) {
		points.at<float>(i, 0) = m_latest_paired_corners_curr[i].x;
		points.at<float>(i, 1) = m_latest_paired_corners_curr[i].y;
	//	points.at<float>(i, 2) = newPoints[i].z;
	}

	for (size_t i = 0, end = nnode; i < end; i++) {
		newPoints3D.at<float>(i, 0) = newPoints[i].x;
		newPoints3D.at<float>(i, 1) = newPoints[i].y;
		newPoints3D.at<float>(i, 2) = newPoints[i].z;
	}

	for (size_t i = 0, end = nnode; i < end; i++) {
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

	float curr = -1;
	float curg = -1;
	float curb = -1;
	int curri = 0; 
	int curgi = 0;
	int curbi=0;

	for (int y = 0; y < points.rows; y++)
	{
		int cluster_idx = labels.at<int>(y, 0);
		 int nx=points.at<float>(y, 0) ;
		 int ny=points.at<float>(y, 1) ;
		 cv::Point pt_txt1(nx,ny);

		 float cvalue = cur[nx + ny*w];

		 bool bedge = false;
		 const int winsize = 3;
		 for(int wx =-winsize; wx <= winsize; wx++)
		 {
			 for (int wy = -winsize; wy <= winsize; wy++)
			 {
				 
				 int wwx = nx + wx;
				 int wwy = ny + wy;
				 if ((wwx > 1 && wwx < (w - 1)) && (wwy > 1 && wwy < (h - 1)))
				 {
					 Vector4u seg = segimg[wwx + (wwy)*w];
					// cout << wwx << "," << wwy << ":" << seg << endl;
					 if (seg.x <=0 || seg.y <=0)
						 bedge = true;
					
				 }
			 }
		 }
		 if (bedge)
			 continue;

		 std::stringstream ssout1;
		 cout << cluster_idx << ":" << cvalue << endl;
		 ssout1 << cluster_idx << ":" << cvalue << endl;
		 cv::putText(m_normal, ssout1.str(), pt_txt1, CV_FONT_HERSHEY_SIMPLEX, 0.3, cc0, 1, LINE_AA, false);
		 int len = sizeof(long);
		 //points_areanum[y] = cluster_idx;
		 
		 switch (cluster_idx)
		 {
		 case 0:
			 if (cvalue > curr)
			 {
				 curr = cvalue;
				 curri = y;
				 cout << "red:" << red_num << endl;

				 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc1);
			 }
			 red_num++;
			 break;
		 case 1:
			 if (cvalue > curg)
			 {
				 curg = cvalue;
				 curgi = y;
				 cout << "green:" << green_num << endl;


				 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc2);
			 }
			 green_num++;
			 break;
		 case 2:
			 if (cvalue > curb)
			 {
				 curb = cvalue;
				 curbi = y;
				 cout << "blue:" << blue_num << endl;


				 cv::rectangle(m_normal, cv::Rect(nx - cBlockSize, ny - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc3);
			 }
			 blue_num++;
		 }
		
	}

	newPoints_rand.at<float>(0, 0) = newPoints3D.at<float>(curri, 0);
		newPoints_rand.at<float>(0, 1) = newPoints3D.at<float>(curri, 1);
		newPoints_rand.at<float>(0, 2) = newPoints3D.at<float>(curri, 2);
			objectPoints_rand.at<float>(0, 0) = objectPoints3D.at<float>(curri, 0);
			objectPoints_rand.at<float>(0, 1) = objectPoints3D.at<float>(curri, 1);
			objectPoints_rand.at<float>(0, 2) = objectPoints3D.at<float>(curri, 2);

			newPoints_rand.at<float>(1, 0) = newPoints3D.at<float>(curgi, 0);
			newPoints_rand.at<float>(1, 1) = newPoints3D.at<float>(curgi, 1);
			newPoints_rand.at<float>(1, 2) = newPoints3D.at<float>(curgi, 2);
			objectPoints_rand.at<float>(1, 0) = objectPoints3D.at<float>(curgi, 0);
			objectPoints_rand.at<float>(1, 1) = objectPoints3D.at<float>(curgi, 1);
			objectPoints_rand.at<float>(1, 2) = objectPoints3D.at<float>(curgi, 2);

			newPoints_rand.at<float>(2, 0) = newPoints3D.at<float>(curbi, 0);
			newPoints_rand.at<float>(2, 1) = newPoints3D.at<float>(curbi, 1);
			newPoints_rand.at<float>(2, 2) = newPoints3D.at<float>(curbi, 2);
			objectPoints_rand.at<float>(2, 0) = objectPoints3D.at<float>(curbi, 0);
			objectPoints_rand.at<float>(2, 1) = objectPoints3D.at<float>(curbi, 1);
			objectPoints_rand.at<float>(2, 2) = objectPoints3D.at<float>(curbi, 2);
	


	int count = 0;
	cv::Mat Transform;
	Transform = rigid_transformPose(objectPoints_rand, newPoints_rand, posd);

	
		cout << "Transform = \n" << Transform << endl;
	
	//	imshow("tt", m_normal);
	//	waitKey(0);

	return;
}
