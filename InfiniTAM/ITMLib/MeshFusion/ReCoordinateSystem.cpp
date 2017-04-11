#include "MeshFusion.h"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "..\InfiniTAM\Engine\calibrate.h"

using namespace cv;
using namespace std;
static int count2 = 0; //紀錄目前是第幾個Camera
bool readTransform(string  s, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs, vector<cv::Mat>& theta)
{
	FileStorage fs(s, FileStorage::READ);

	if (!fs.isOpened())
		return false;

	cv::Mat bigmat, bigmat2;
	fs["Yaxis_theta"] >> bigmat;
	int nrows = bigmat.rows;
	for (int i = 0; i < nrows; i++)
	{
		cv::Mat r = bigmat(Range(i, i + 1), Range(0, 4));
		theta.push_back(r);
	}
	fs["Extrinsic_Parameters"] >> bigmat2;
	nrows = bigmat2.rows;
	for (int i = 0; i < nrows; i++)
	{
		Mat r = bigmat2(Range(i, i + 1), Range(0, 3));
		Mat t = bigmat2(Range(i, i + 1), Range(3, 6));
		rvecs.push_back(r);
		tvecs.push_back(t);
	}

	return true;
}

void MeshFusion::ReCoordinateSystem(ITMPose * posd) {
	const string outputFileName2 = "images\\camera_data2.xml";
	cv::Mat cameraMatrix, distCoeffs;
	vector<cv::Mat> rvecs, tvecs, theta; // rotation , transform , theta(yaxis 22.5)
	Size imageSize;

	if (!readTransform(outputFileName2, rvecs, tvecs, theta))//取得Camera的rotation 和 transform
	{

	}
	
	cv::Mat RF(3, 3, cv::DataType<double>::type); //存放Camera的R
	cv::Rodrigues(rvecs[count2], RF);	//向量轉旋轉矩陣，旋轉矩陣轉向量
	cv::Mat invRF = RF.inv();
	Matrix3f R[16],invR[16];	//存共16個Camera的R與R逆
	Matrix4f Rtheta;	//Rθ,θ=22.5
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			float v = RF.at<double>(i, j);
			R[count2].m[i + j * 3] = v;
			v = invRF.at<double>(i, j);
			invR[count2].m[i + j * 3] = v;
		}

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			float v = theta[i].at<double>(j);
			Rtheta.m[i + j * 4] = v;
		}
	//cout << "Rtheta = \n" << Rtheta << endl;
	Vector3f T(tvecs[count2].at<double>(0), tvecs[count2].at<double>(1), tvecs[count2].at<double>(2));
	Vector3f TS[16];	//存共16個Camera的T
	TS[count2] = T;

	ITMPose newpos;
	newpos.SetR(R[count2]);
	newpos.SetT(T);
	newpos.Coerce();
	posd->MultiplyWith(&newpos);//新坐標系

	//還原坐標系測試↓↓↓
	for (int i = 0; i < count2; i++)
	{
		newpos.SetM(Rtheta);
		newpos.Coerce();
		posd->MultiplyWith(&newpos);
	}
	//還原坐標系測試↑↑↑
	count2++;
	if (count2 == 16)count2 = 0;
}