
#include "MeshFusion.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

using namespace boost;
using namespace boost::accumulators;

using namespace ITMLib::Objects;

typedef accumulator_set<double, features<tag::density> > acc;
typedef iterator_range<std::vector<std::pair<double, double> >::iterator > histogram_type;


void MeshFusion::Generate3DPoints(std::vector<cv::Point2f> & imp, std::vector<cv::Point3f> &d3p)
{
	d3p.clear();
	//std::vector<cv::Point3f> ret;
	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
	int xlens = proDepth->noDims.x;
	int ylens = proDepth->noDims.y;

	for (int i = 0; i < (int)imp.size(); i++)
	{
		cv::Point2f p =imp[i];
		
		cv::Point3f retp;
		bool ret = false;// find3DPos(p, retp);
		if (!ret)
		{
			float z = estivalue(dd, Vector2i(p.x,p.y), Vector2i(-1,-1));
			float x = z * (p.x - intrinparam.z) / intrinparam.x;
			float y = z * (p.y - intrinparam.w) / intrinparam.y;

			retp = cv::Point3f(x, y, z);

		}
		d3p.push_back(retp);// cv::Point3f(x, y, z));
	}
	//return ret;
}

void MeshFusion::estimatePose( ITMPose * aposd)
{
	if (m_backup.size() != objectPoints.size())//|| m_pre_corners.size()==0)
		return;

	//m_backup = m_corners;
	//m_backup2 = m_base_corners;

	// Read points
	std::vector<cv::Point2f> imagePoints = m_backup;
	//std::vector<cv::Point3f> objectPoints = Generate3DPoints( );

	DEBUG_OUTPUT_STREAM << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << DEBUG_ENDL;
	cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);

	cv::setIdentity(cameraMatrix);

	cameraMatrix.at<double>(0, 0)= mainView->calib->intrinsics_rgb.projectionParamsSimple.fx;
	cameraMatrix.at<double>(1, 1) = mainView->calib->intrinsics_rgb.projectionParamsSimple.fy;
	cameraMatrix.at<double>(0, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.px;
	cameraMatrix.at<double>(1, 2) = mainView->calib->intrinsics_rgb.projectionParamsSimple.py;

	//DEBUG_OUTPUT_STREAM << "Initial cameraMatrix: " << cameraMatrix << std::endl;

	cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;

	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);

	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_EPNP	);
    //cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

	//tvec.at<double>(2) = tvec.at<double>(2) + 50;
	//cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true,  100, 4, 0.99,  cv::noArray(), CV_EPNP);// , false, CV_EPNP);

    DEBUG_OUTPUT_STREAM << "rvec: " << rvec << DEBUG_ENDL;
    DEBUG_OUTPUT_STREAM << "tvec: " << tvec << DEBUG_ENDL;
    

    std::vector<cv::Point2f> projectedPoints;
	   cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
    
    acc myAccumulator( tag::density::num_bins = 20, tag::density::cache_size = 10);
    
    float sumerr = 0;
    for (unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        // DEBUG_OUTPUT_STREAM << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
        cv::Point2f diff = imagePoints[i] - projectedPoints[i];
        float err = sqrt(diff.dot(diff));
        sumerr += err;
        myAccumulator(err);
    }


    DEBUG_OUTPUT_STREAM << "Error: average " << ((projectedPoints.size()==0)?-1:sumerr/projectedPoints.size()) << " sum " << sumerr << DEBUG_ENDL;
    
    histogram_type hist = density(myAccumulator);

    std::vector<float> vplot;
    
    for( size_t i = 0; i < hist.size(); i++ )
    {
        DEBUG_OUTPUT_STREAM << "Bin lower bound: " << std::setw(5) << hist[i].first << ", Value: " << hist[i].second << DEBUG_ENDL;
        vplot.push_back( hist[i].second );
    }
    OutputDebugPlot(200,50,vplot);

	cv::Mat RM(3, 3, cv::DataType<double>::type);

	cv::Rodrigues(rvec, RM);

	DEBUG_OUTPUT_STREAM << RM << DEBUG_ENDL;
	
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