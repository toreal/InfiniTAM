

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
	int  sampleCount = 20;
	Mat points(sampleCount, 1, CV_32FC2), labels;
	Mat centers;

	kmeans(points, clusterCount, labels,
		TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0),
		3, KMEANS_PP_CENTERS, centers);



}