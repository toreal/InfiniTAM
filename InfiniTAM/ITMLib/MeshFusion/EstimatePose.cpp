
#include "MeshFusion.h"
#include "opencv2/calib3d/calib3d.hpp"
using namespace ITMLib::Objects;


std::vector<cv::Point3f> MeshFusion::Generate3DPoints( )
{
	std::vector<cv::Point3f> ret;
	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
	int xlens = proDepth->noDims.x;
	int ylens = proDepth->noDims.y;

	for (int i = 0; i < (int)m_pre_corners.size(); i++)
	{
		cv::Point2f p = m_pre_corners[i];
		float z = estivalue(dd,(int)p.x + xlens* (int)p.y);
		float x = z * (p.x - intrinparam.z) / intrinparam.x;
		float y = z * (p.y - intrinparam.w) / intrinparam.y;

		ret.push_back(cv::Point3f(x, y, z));
	}
	return ret;
}

void MeshFusion::estimatePose()
{
	if (m_corners.size() != m_pre_corners.size())
		return;
	// Read points
	std::vector<cv::Point2f> imagePoints = m_corners;
	std::vector<cv::Point3f> objectPoints = Generate3DPoints( );

	std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
	cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);

	cv::setIdentity(cameraMatrix);

	cameraMatrix.at<double>(0, 0)= mainView->calib->intrinsics_rgb.projectionParamsSimple.fx;
	cameraMatrix.at<double>(1, 1) = mainView->calib->intrinsics_rgb.projectionParamsSimple.fy;
	cameraMatrix.at<double>(0, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.px;
	cameraMatrix.at<double>(1, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.py;

	std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

	cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;

	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);

	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

	std::cout << "rvec: " << rvec << std::endl;
	std::cout << "tvec: " << tvec << std::endl;



}