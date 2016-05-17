
#include "MeshFusion.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
using namespace ITMLib::Objects;


void MeshFusion::Generate3DPoints( )
{
	objectPoints.clear();
	//std::vector<cv::Point3f> ret;
	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
	int xlens = proDepth->noDims.x;
	int ylens = proDepth->noDims.y;

	for (int i = 0; i < (int)m_base_corners.size(); i++)
	{
		cv::Point2f p = m_base_corners[i];
		
		cv::Point3f retp;
		bool ret = false;// find3DPos(p, retp);
		if (!ret)
		{
			float z = estivalue(dd, Vector2i(p.x,p.y), Vector2i(-1,-1));
			float x = z * (p.x - intrinparam.z) / intrinparam.x;
			float y = z * (p.y - intrinparam.w) / intrinparam.y;

			retp = cv::Point3f(x, y, z);

		}
		objectPoints.push_back(retp);// cv::Point3f(x, y, z));
	}
	//return ret;
}

void MeshFusion::estimatePose( ITMPose * aposd)
{
	if (m_corners.size() != objectPoints.size()|| m_pre_corners.size()==0)
		return;

	m_backup = m_corners;
	m_backup2 = m_base_corners;

	// Read points
	std::vector<cv::Point2f> imagePoints = m_corners;
	//std::vector<cv::Point3f> objectPoints = Generate3DPoints( );

	std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
	cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);

	cv::setIdentity(cameraMatrix);

	cameraMatrix.at<double>(0, 0)= mainView->calib->intrinsics_rgb.projectionParamsSimple.fx;
	cameraMatrix.at<double>(1, 1) = mainView->calib->intrinsics_rgb.projectionParamsSimple.fy;
	cameraMatrix.at<double>(0, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.px;
	cameraMatrix.at<double>(1, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.py;

	//std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

	cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;

	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);

	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_EPNP	);

	//tvec.at<double>(2) = tvec.at<double>(2) + 50;
	//cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true,  100, 4, 0.99,  cv::noArray(), CV_EPNP);// , false, CV_EPNP);

	std::cout << "rvec: " << rvec << std::endl;
	std::cout << "tvec: " << tvec << std::endl;

	std::vector<cv::Point2f> projectedPoints;
	   cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
	
	   float sumerr = 0;
		   for (unsigned int i = 0; i < projectedPoints.size(); ++i)
		     {
		    // std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
			 cv::Point2f diff = imagePoints[i] - projectedPoints[i];
			 sumerr += sqrt(diff.dot(diff));
		   }

		   std::cout << "Error" << sumerr << std::endl;

	cv::Mat RM(3, 3, cv::DataType<double>::type);

	cv::Rodrigues(rvec, RM);

	std::cout << RM << std::endl;
	
	Matrix3f R ;
	for(int i=0 ; i < 3;i++ )
		for (int j = 0; j < 3; j++)
		{
			float v = RM.at<double>(i,j);
			R.m[i + j * 3] = v;
		}
	Vector3f T(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
	
	ITMPose newpos;
	newpos.SetR(R);
	newpos.SetT(T);
	newpos.Coerce();
	aposd->MultiplyWith(&newpos);

	


}