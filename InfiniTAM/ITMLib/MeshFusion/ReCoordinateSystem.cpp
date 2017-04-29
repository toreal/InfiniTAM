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


struct AxisAngle4d
{
	double angle;
	double x;
	double y;
	double z;
};

//http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
void matrixFromAxisAngle(AxisAngle4d a1, Mat & r21) {


	double c = cos(a1.angle);
	double s = sin(a1.angle);
	double t = 1.0 - c;
	//  if axis is not already normalised then uncomment this
	// double magnitude = Math.sqrt(a1.x*a1.x + a1.y*a1.y + a1.z*a1.z);
	// if (magnitude==0) throw error;
	// a1.x /= magnitude;
	// a1.y /= magnitude;
	// a1.z /= magnitude;

	double m00 = c + a1.x*a1.x*t;
	double m11 = c + a1.y*a1.y*t;
	double m22 = c + a1.z*a1.z*t;


	double tmp1 = a1.x*a1.y*t;
	double tmp2 = a1.z*s;
	double m10 = tmp1 + tmp2;
	double m01 = tmp1 - tmp2;
	tmp1 = a1.x*a1.z*t;
	tmp2 = a1.y*s;
	double 	m20 = tmp1 - tmp2;
	double 	m02 = tmp1 + tmp2;
	tmp1 = a1.y*a1.z*t;
	tmp2 = a1.x*s;
	double m21 = tmp1 + tmp2;
	double m12 = tmp1 - tmp2;

	r21.at<double>(0, 0) = m00;
	r21.at<double>(1, 0) = m01;
	r21.at<double>(2, 0) = m02;

	r21.at<double>(0, 1) = m10;
	r21.at<double>(1, 1) = m11;
	r21.at<double>(2, 1) = m12;


	r21.at<double>(0, 2) = m20;
	r21.at<double>(1, 2) = m21;
	r21.at<double>(2, 2) = m22;

}




bool binitread = false;

Mat rot0;
Mat vtrans;
void MeshFusion::rotateAngle(ITMPose * posd) {

	if (!binitread)
	{
		const string outputFileName2 = ".\\Files\\images\\images\\out_camera_data.xml";

		FileStorage fs(outputFileName2, FileStorage::READ);

		if (!fs.isOpened())
			return;
		Mat vrot;
		cv::Mat  bigmat2;
		fs["Extrinsic_Parameters"] >> bigmat2;
		int	nrows = bigmat2.rows;
		if (nrows > 0 )
		{
			vrot = bigmat2(Range(0,1), Range(0, 3));
			vtrans = bigmat2(Range(0,1), Range(3, 6));
			vtrans = vtrans / 1000;
			
		}


		binitread = true;
		
	
		cv::Rodrigues(vrot, rot0);

		Matrix3f R;	//存共16個Camera的R與R逆
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				float v = rot0.at<double>(i, j);
				R.m[i + j * 3] = v;
			}

		Vector3f T(vtrans.at<double>(0), vtrans.at<double>(1), vtrans.at<double>(2));

		ITMPose newpos;
		posd->SetR(R);
		posd->SetT(T);
		posd->Coerce();
		//posd->MultiplyWith(&newpos);//新坐標系
		count2++;
		return;


	}


	Mat r21(3,3,CV_64F);
	AxisAngle4d a1;
	a1.angle = (count2*22.5f)*M_PI / 180;
	a1.x = 0;
	a1.y = 0;
	a1.z = 1;

	matrixFromAxisAngle(a1, r21);
	cout << r21 << endl;
	cout << vtrans << endl;
	Mat t21 = -r21*vtrans.t() + vtrans.t();

	cout << t21 << endl;

	Mat R2 = r21*rot0;
	//Mat t2 = r21*vtrans.t() + t21;

	Matrix3f R;	//存共16個Camera的R與R逆
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			float v =R2.at<double>(i, j);
			R.m[i + j * 3] = v;
		}

//	Vector3f T(t21.at<double>(0), t21.at<double>(1), t21.at<double>(2));
	Vector3f T(vtrans.at<double>(0), vtrans.at<double>(1), vtrans.at<double>(2));


	ITMPose newpos;
	posd->SetR(R);
	posd->SetT(T);
	posd->Coerce();
	//posd->MultiplyWith(&newpos);//新坐標系

	count2++;

}

void MeshFusion::ReCoordinateSystem(ITMPose * posd) {
	//const string outputFileName2 = "images\\camera_data2.xml";
	  const string outputFileName2 = ".\\Files\\images\\images\\out_camera_data.xml";
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
	
	if (theta.capacity() > 0)
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				float v = theta[i].at<double>(j);
				Rtheta.m[i + j * 4] = v;
			}
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

	////還原坐標系測試↓↓↓
	//for (int i = 0; i < count2; i++)
	//{
	//	newpos.SetM(Rtheta);
	//	newpos.Coerce();
	//	posd->MultiplyWith(&newpos);
	//}
	////還原坐標系測試↑↑↑
	count2++;
	if (count2 == 16)
		count2 = 0;
}