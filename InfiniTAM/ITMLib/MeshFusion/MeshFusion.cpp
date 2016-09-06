


#include "MeshFusion.h"
#include "../Engine/DeviceAgnostic/ITMPixelUtils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <algorithm>
#include <vector> 
#include <iterator> 
# include <iostream> 
#include "../../ORUtils/psimpl.h"
#include "../Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h"
//#include <math.h>
//#include <opencv\cvwimage.h>

//#define WITH_FADE 1

#ifdef WITH_FADE 
#include <Fade_2D.h>
using namespace GEOM_FADE2D;

#else

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>


#include <cassert>
#include <iostream>
#include <fstream>

//typedef CGAL::Exact_predicates_inexact_constructions_kernel K2;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, K> Vb;
//typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;

typedef CGAL::Spatial_sort_traits_adapter_2<K, Point2*> Search_traits;


#endif 


using namespace ITMLib::Objects;
using namespace ITMLib::Engine;

using namespace std;
using namespace cv;


void addvextex(std::vector<Point2> &vInputPoints, Point2 p)
{
	int x = p.x();
	for (int i = 0; i < vInputPoints.size(); i++)
	{
		Point2 ap = vInputPoints[i];
		Point2 dis = Point2(ap.x() - p.x(), ap.y() - p.y());
		float diff = dis.x()*dis.x() + dis.y()*dis.y();

		if (diff < 50)
			return;


	}
	vInputPoints.push_back(p);

}


void MeshFusion::genContour(char * fn)
{
	Mat canny_output;
	Mat src_gray;
	int thresh = 100;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
//	 segImage

	int w = segImage->noDims.x;
	int h = segImage->noDims.y;
	Vector4u* segimg = segImage->GetData(MEMORYDEVICE_CPU);
	cv::Mat seginput(h, w, CV_8UC4, segimg);

	cvtColor(seginput, src_gray, CV_BGR2GRAY);

	/// Detect edges using canny
	Canny(src_gray, canny_output, thresh, thresh * 2, 3);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<Point> contour = contours[0];
	npoint = contour.size();

	FILE* fp = fopen(fn, "w");
	fprintf(fp, "%d\n", npoint);
	

	for (int i = 0; i < contour.size(); i++)
	{
		
	    Point p=	contour[i];
		pointlist[i].x = p.x;
		pointlist[i].y = p.y;

		fprintf(fp, "%d %d\n", pointlist[i].x, pointlist[i].y);
	}
	fclose(fp);
}

void MeshFusion::sortpoint(ITMUChar4Image * draw)
{
	
	int count = npoint;
	
	std::vector <int> generatedPoints ;
	std::vector <int> result;
	
	for (int i = 0; i < npoint; i++)
	{
		generatedPoints.push_back(pointlist[i].x);
		generatedPoints.push_back(pointlist[i].y);

	}
		
	// simplify
	std::vector <int>::const_iterator begin = generatedPoints.begin();
	std::vector <int>::const_iterator end = generatedPoints.end();
		
	psimpl::simplify_douglas_peucker_n<2>(begin, end, count,std::back_inserter(result) );
	
	DEBUG_OUTPUT_STREAM <<result.size() << std::endl;

	if (draw == NULL)
		return;

	Vector4u* data = draw->GetData(MEMORYDEVICE_CPU);
	int i = 0; 
	for (vector<int>::iterator it = result.begin(); it != result.end(); ++it) {
		int x = *it;
		++it;
		int y = *it;
		//DEBUG_OUTPUT_STREAM << x <<"," << y << endl;
		
		this->sellist[i].x = x;
		this->sellist[i].y = y;
		i++;
	}

	this->selp = i;


	vInputPoints.clear();
	for (int i = 0; i < selp; i++)
	{
		addvextex(vInputPoints, Point2(sellist[i].x, sellist[i].y));
	}

	ncon = vInputPoints.size();
	for (int i = 0; i<(int)m_corners.size(); i++)
	{
		addvextex(vInputPoints, Point2(m_corners[i].x, m_corners[i].y));

	}




}

///from nomal
void EstCurvature(ITMFloat4Image * normal_in, ITMFloatImage * curvature)
{
	Vector2i imgDims = normal_in->noDims;

	const Vector4f *depthData_in = normal_in->GetData(MEMORYDEVICE_CPU);

	float *curvature_out = curvature->GetData(MEMORYDEVICE_CPU);

	float maxv = -1;
	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
	{
		int idx = x + y * imgDims.x;
		Vector4f p1 = depthData_in[x + 1 + y * imgDims.x];
		Vector4f p2 = depthData_in[x - 1 + y * imgDims.x];
		Vector4f p3 = depthData_in[x + (y + 1) * imgDims.x];
		Vector4f p4 = depthData_in[x + (y - 1) * imgDims.x];

		if (p1.w < 0 || p2.w < 0 || p3.w < 0 || p4.w < 0)
		{
			curvature_out[idx] = 0;
			continue;
		}




		float  thetax = acos(dot(p1, p2) - 1);
		float  thetay = acos(dot(p3, p4) - 1);

		curvature_out[idx] = abs(thetax) + abs(thetay);

		if (curvature_out[idx] > maxv)
		{
			maxv = curvature_out[idx];
		}

	}
	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
	{
		int idx = x + y * imgDims.x;
		/*if ((curvature_out[idx] / maxv) > 0.5)
		{
			curvature_out[idx] = ((curvature_out[idx] / maxv) + 1) / 2;
		}
		else*/
			curvature_out[idx] = curvature_out[idx] / ( maxv);

	}



}


///from depth 
/*import numpy as np

Zy, Zx = np.gradient(Z)
Zxy, Zxx = np.gradient(Zx)
Zyy, _ = np.gradient(Zy)

# Mean Curvature - equation(3) from Kurita and Boulanger(1992) paper
# See also Surface in 3D space, http://en.wikipedia.org/wiki/Mean_curvature
H = (1 + (Zx ** 2)) * Zyy + (1 + (Zy ** 2)) * Zxx - 2 * Zx * Zy * Zxy      
H = H / ((2 * (1 + (Zx ** 2) + (Zy ** 2))) ** 1.5)

# Gaussian Curvature - equation(4) from Kurita and Boulanger(1992) paper 
K = (Zxx * Zyy - (Zxy ** 2)) / ((1 + (Zx ** 2) + (Zy **2)) ** 2)

# Simplified Mean Curvature - equation(3) from Zhao et.al(1996) paper               
H = Zxx + Zyy                                                              

#  Simplified Gaussian Curvature - equation(3) from Zhao et.al(1996) paper           
K = Zxx * Zyy - (Zxy ** 2)
*/


void EstCurvatureByDepth(ITMFloatImage * depth_in, ITMFloatImage * curvature)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *curvature_out = curvature->GetData(MEMORYDEVICE_CPU);
	float * zxval = new float[imgDims.x*imgDims.y];
	float * zyval = new float[imgDims.x*imgDims.y];
	float maxv = -1;
	float smax = -1;
	int idx;
	for (int y = 2; y < imgDims.y - 2; y++)
		for (int x = 2; x < imgDims.x - 2; x++)
	{
		 idx = x + y * imgDims.x;
		float p1 = depthData_in[x + 1 + y * imgDims.x];
		float p2 = depthData_in[x - 1 + y * imgDims.x];
		float p3 = depthData_in[x + (y + 1) * imgDims.x];
		float p4 = depthData_in[x + (y - 1) * imgDims.x];

		float p5 = depthData_in[x + 2 + y * imgDims.x];
		float p6 = depthData_in[x - 2 + y * imgDims.x];
		float p7 = depthData_in[x + (y + 2) * imgDims.x];
		float p8 = depthData_in[x + (y - 2) * imgDims.x];


		zxval[idx] =( p1 - p2)/2;
		zyval[idx] =( p3 - p4)/2;

		if (zxval[idx] == 0 && (p5 >0 && p6 >0))
		{
			zxval[idx] = (p5 - p6) / 4;
		}

		if (zyval[idx] == 0 && p7>0  && p8 >0)
		{
			zyval[idx] = (p7 - p8) / 4;
		}

	
	}
/* for debug
	maxv = 3;

	unsigned char *buf = new 	unsigned char[imgDims.x*imgDims.y];
	for (int y = 2; y < imgDims.y - 2; y++)
		for (int x = 2; x < imgDims.x - 2; x++)
		{
			idx = x + y * imgDims.x;
			if (zxval[idx] != 0 && fabs(zxval[idx])<3)
				buf[idx] = ((zxval[idx] / maxv) + 1.0) * 127;
			else
				buf[idx] = 0;

			
		}

	maxv = -1;
	cv::Mat m(imgDims.y, imgDims.x, CV_8U, buf);
	imshow("tmp", m);
	cvWaitKey(0);
	delete[] buf;
*/
	for (int y = 2; y < imgDims.y - 2; y++)
		for (int x = 2; x < imgDims.x - 2; x++)
		{
			idx = x + y * imgDims.x;
			float zx = zxval[idx];
			float p1 = zxval[x + 1 + y * imgDims.x];
			float p2 = zxval[x - 1 + y * imgDims.x];
			float p3 = zxval[x + (y + 1) * imgDims.x];
			float p4 = zxval[x + (y - 1) * imgDims.x];

			float zy = zyval[idx];
			float p5 = zyval[x + 1 + y * imgDims.x];
			float p6 = zyval[x - 1 + y * imgDims.x];
			float p7 = zyval[x + (y + 1) * imgDims.x];
			float p8 = zyval[x + (y - 1) * imgDims.x];


			float p11 = zxval[x + 2 + y * imgDims.x];
			float p12 = zxval[x - 2 + y * imgDims.x];
			float p13 = zxval[x + (y + 2) * imgDims.x];
			float p14 = zxval[x + (y - 2) * imgDims.x];

			float p15 = zyval[x + 2 + y * imgDims.x];
			float p16 = zyval[x - 2 + y * imgDims.x];
			float p17 = zyval[x + (y + 2) * imgDims.x];
			float p18 = zyval[x + (y - 2) * imgDims.x];



			float zxx = (p1 - p2)/2;
			float zxy = (p3 - p4)/2;
			float zyx = (p5 - p6)/2;
			float zyy = (p7 - p8)/2;


			if ( zxx ==0 )
				zxx = (p11 - p12) / 4;

			if ( zxy == 0)
				zxy = (p13 - p14) / 4;

			if (zyy == 0 )
				zyy = (p17 - p18) / 4;

			float		H = (1 + pow(zx , 2)) * zyy + (1 + pow(zy , 2)) * zxx - 2 * zx * zy * zxy;
			H = H / pow((2 * (1 + pow(zx , 2) + pow(zy , 2))) , 1.5);
		float	Kv = (zxx * zyy - pow(zxy , 2)) / pow((1 + pow(zx , 2) + pow(zy ,2)) , 2);
				curvature_out[idx] = H;
				if (H < 100)
				{
					if (curvature_out[idx] > maxv)
					{
						if (maxv > smax)
							smax = maxv;
						maxv = curvature_out[idx];
					}
					else if (curvature_out[idx] > smax)
						smax = curvature_out[idx];

				}
		}





	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
	{
		int idx = x + y * imgDims.x;
		if (curvature_out[idx]>0.2)
		{
			curvature_out[idx] =((curvature_out[idx] / smax) + 1) / 2;
		}
		else
			curvature_out[idx] = curvature_out[idx] / ( smax);


		if (curvature_out[idx] > 1)
			curvature_out[idx] = 1;
	}


	delete []zxval;
	delete[] zyval;
}




_CPU_AND_GPU_CODE_ inline void computeNormalAndWeight(const CONSTPTR(float) *depth_in, DEVICEPTR(Vector4f) *normal_out, DEVICEPTR(float) *sigmaZ_out, int x, int y, Vector2i imgDims, Vector4f intrinparam)
{
	Vector3d outNormal;

	int idx = x + y * imgDims.x;

	double z = depth_in[x + y * imgDims.x];
	if (z < 0.0f)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	// first compute the normal
	Vector3d xp1_y, xm1_y, x_yp1, x_ym1;
	Vector3d diff_x(0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f);

	xp1_y.z = depth_in[(x + 1) + (y) * imgDims.x], x_yp1.z = depth_in[(x) + (y + 1) * imgDims.x];
	xm1_y.z = depth_in[(x - 1) + (y) * imgDims.x], x_ym1.z = depth_in[(x) + (y - 1) * imgDims.x];

	if (xp1_y.z <= 0 || x_yp1.z <= 0 || xm1_y.z <= 0 || x_ym1.z <= 0)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	// unprojected
	xp1_y.x = xp1_y.z * ((x + 1.0f) - intrinparam.z) / intrinparam.x; xp1_y.y = xp1_y.z * (y - intrinparam.w) / intrinparam.y;
	xm1_y.x = xm1_y.z * ((x - 1.0f) - intrinparam.z) / intrinparam.x; xm1_y.y = xm1_y.z * (y - intrinparam.w) / intrinparam.y;
	x_yp1.x = x_yp1.z * (x - intrinparam.z) / intrinparam.x; x_yp1.y = x_yp1.z * ((y + 1.0f) - intrinparam.w) / intrinparam.y;
	x_ym1.x = x_ym1.z * (x - intrinparam.z) / intrinparam.x; x_ym1.y = x_ym1.z * ((y - 1.0f) - intrinparam.w) / intrinparam.y;

	// gradients x and y
	diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

	double normx = 1.0 / sqrt(diff_x.x * diff_x.x + diff_x.y * diff_x.y + diff_x.z * diff_x.z);
	diff_x *= normx;

	double normy = 1.0 / sqrt(diff_y.x * diff_y.x + diff_y.y * diff_y.y + diff_y.z * diff_y.z);
	diff_y *= normy;


	// cross product
	outNormal.x = (diff_x.y * diff_y.z - diff_x.z*diff_y.y);
	outNormal.y = (diff_x.z * diff_y.x - diff_x.x*diff_y.z);
	outNormal.z = (diff_x.x * diff_y.y - diff_x.y*diff_y.x);

	if (outNormal.x == 0.0f && outNormal.y == 0 && outNormal.z == 0)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	double  norm = 1.0 / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
	outNormal *= norm;

	normal_out[idx].x = outNormal.x; normal_out[idx].y = outNormal.y; normal_out[idx].z = outNormal.z; normal_out[idx].w = 1.0f;

	// now compute weight
	float theta = acos(outNormal.z);
	float theta_diff = theta / (PI*0.5f - theta);

	sigmaZ_out[idx] = (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * theta_diff * theta_diff);
}


void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);
	memset(normalData_out, 0x00, sizeof(Vector4f)*normal_out->noDims.x*normal_out->noDims.y);

	for (int y = 2; y < imgDims.y - 2; y++)
		for (int x = 2; x < imgDims.x - 2; x++)
			computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}



void MeshFusion::NormalAndCurvature(ITMView **view_ptr, bool modelSensorNoise)
{
	ITMView *view = *view_ptr;

	if (modelSensorNoise)
	{
		ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, proDepth, view->calib->intrinsics_rgb.projectionParamsSimple.all);
		//EstCurvature(view->depthNormal, view->curvature);
		EstCurvatureByDepth(proDepth, view->curvature);
	}

}


void MeshFusion::buildProjDepth()
{
	//ITMFloatImage * floatImage= new ITMFloatImage(mainView->depth->noDims, true, false);

	if (proDepth == NULL  )
	proDepth=	new ITMFloatImage(mainView->depth->noDims, true, false);

	Vector4f intrinD= mainView->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f intrinRGB= mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

	Matrix4f d2rgb = mainView->calib->trafo_rgb_to_depth.calib_inv;
	Matrix4f rgb2d = mainView->calib->trafo_rgb_to_depth.calib;

	float* dp = proDepth->GetData(MEMORYDEVICE_CPU);
	Vector4u* segimg = segImage->GetData(MEMORYDEVICE_CPU);

	int lens = proDepth->noDims.x * proDepth->noDims.y;
	memset(dp, 0x00, sizeof(float) *lens);

	float* dd = mainView->depth->GetData(MEMORYDEVICE_CPU);
	int xlens = mainView->depth->noDims.x;
	int ylens = mainView->depth->noDims.y;
	for (int nx = 0; nx < xlens; nx++)
		for (int ny = 0; ny < ylens; ny++)
		{
			float pz = dd[nx + ny*xlens] *1000;

			if (pz > 0)
			{
				Vector4f pp;
				
				pp.x = pz * (nx - intrinD.z) / intrinD.x; 
				pp.y = pz * (ny - intrinD.w) / intrinD.y;
				pp.z = pz;
				pp.w = 1;

				Vector4f prgb = d2rgb*pp;
				prgb.homogeneousCoordinatesNormalize();

				int ix = prgb.x * intrinRGB.x / prgb.z + intrinRGB.z;
				int iy = prgb.y * intrinRGB.y / prgb.z + intrinRGB.w;

				
				if ( ix >=0 && ix < xlens && iy >=0 && iy < ylens)
				{  

					Vector4u mask = segimg[ix + iy*xlens];

					if (mask.x > 0 || mask.y > 0)
					{
						if (dp[ix + iy*xlens] > 0 && fabs(dp[ix + iy*xlens] - prgb.z) > 10)
						{
							//cout << ix << "," << iy << ":";
							//cout << dp[ix + iy*xlens] << "," << prgb.z << endl;
							prgb.z = (dp[ix + iy*xlens] + prgb.z) / 2;
						}
						dp[ix + iy*xlens] = prgb.z;
					}
					else
						dd[nx + ny*xlens] = -1;

				}
			}
		}

//to check if a pixel is a null or not?

	for (int nx = 0; nx < xlens; nx++)
		for (int ny = 0; ny < ylens; ny++)
		{
			float pz = dp[nx + ny*xlens] ;

			Vector4u mask = segimg[nx + ny*xlens];

			if ((mask.x > 0 || mask.y > 0) && pz == 0)
			{ 
				float minf = 10000, maxf = 0;

				for (int dx = -1; dx <= 1; dx++)
					for (int dy = -1; dy <= 1; dy++)
				{
					float pdz = dp[nx+dx + (ny+dy)*xlens];
					if (pdz > 0)
					{
						if (pdz < minf)
							minf = pdz;
						if (pdz > maxf)
							maxf = pdz;

					}
				}

				if (minf == maxf)
				{
					dp[nx + ny*xlens] = minf;

				}
				else if (maxf > minf)
				{
					Vector4f pp,pb;
					float maxdiff = 10000;
					float fz;
					float delta = maxf - minf;

					float shift = 0.5f;
					if (delta > 2)
						shift = 1 / delta;

					for (float inc = 0; inc <= 1; inc = inc + shift)
					{
						float estz = minf + inc* delta;
						pp.x = estz * (nx - intrinRGB.z) / intrinRGB.x;
						pp.y = estz * (ny - intrinRGB.w) / intrinRGB.y;
						pp.z = estz;
						pp.w = 1;

						Vector4f prgb = rgb2d*pp;
						prgb.homogeneousCoordinatesNormalize();

						int ix = prgb.x * intrinD.x / prgb.z + intrinD.z;
						int iy = prgb.y * intrinD.y / prgb.z + intrinD.w;

						float npz = dd[ix + iy*xlens]*1000;

						float diff = npz - prgb.z;

						if (fabs(diff) < maxdiff && npz > 0 )
						{
							maxdiff = fabs(diff);
							pb = prgb;
							fz = npz;
						}

					}//end of for
					if (maxdiff < 10000)
					{
						Vector4f pbr = d2rgb*pb;
						pbr.homogeneousCoordinatesNormalize();
						dp[nx + ny*xlens] = pbr.z;
						
					}
					else
						dp[nx + ny*xlens] = (maxf + minf) / 2;


				}

			}



		}

	//bool useBilateralFilter = true;
	//
	//	ITMViewBuilder*  vb = new  ITMViewBuilder_CPU( mainView->calib);
	//if (useBilateralFilter)
	//{
	//	//5 steps of bilateral filtering
	//	vb->DepthFiltering(floatImage, proDepth);
	//	vb->DepthFiltering(proDepth, floatImage);
	//	vb->DepthFiltering(floatImage, proDepth);
	//	vb->DepthFiltering(proDepth, floatImage);
	//	//vb->DepthFiltering(floatImage, view->depth);
	//	
	//}

	//delete vb;

	

}
	 

std::string toString(int i)
{
	std::ostringstream oss;
	oss << i;
	return oss.str();
}


float MeshFusion::estivalue(const float * data, Vector2i  p1,Vector2i p2 )
{
	int w = 640;
	int h=480;
	int index=p1.x+ p1.y*w;
	float ret;
	if (p2.x > 0 || p2.y > 0)
	{

		float delx = fabs(p2.x - p1.x);
		float dely = fabs(p2.y - p1.y);
		float  nx, ny;
		if (delx > dely)
		{
			for (int i = 0; i < delx; i++)
			{
				nx = p1.x + (i / delx)*(p2.x - p1.x);
				ny = p1.y + (i / delx)*(p2.y - p1.y);
				ret = data[(int)nx + (int)ny * w];
				if (ret > 0)
					return ret;

			}
		}
		else
		{
			for (int i = 0; i < dely; i++)
			{
				nx = p1.x + (i / dely)*(p2.x - p1.x);
				ny = p1.y + (i / dely)*(p2.y - p1.y);
				ret = data[(int)nx + (int)ny * w];
				if (ret > 0)
					return ret;

			}

		}


	}
		const int lens = w * h - 1;
		ret = data[index];

		for (int i = 1; i < 10; i++)
		{
			if (ret > 0)
				return ret;

			if (ret == 0 && index < (lens - i))
				ret = data[index + i];

			if (ret == 0 && index > i)
				ret = data[index - i];

			if (ret == 0 && index < (lens - i * 640))
				ret = data[index + i * w];

			if (ret == 0 && index > i * w)
				ret = data[index - i * w];


		}
	
	
		 
		
	
	return 300;
}





#ifdef WITH_FADE
void MeshFusion::constructMesh(ITMMesh * mesh )
{
	ITMFloatImage * depth_in = proDepth; 
	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;


	Vector2i imgDims = depth_in->noDims;
	int w = imgDims.x;
	int h = imgDims.y;
	int maxlen = (imgDims.y - 2)*w;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *mask= segImage->GetData(MEMORYDEVICE_CPU);
	

	cv::Mat a;
	

	std::vector<Point2> vInputPoints;

	for (int i = 0; i < selp; i++)
	{
		addvextex(vInputPoints,Point2(sellist[i].x,sellist[i].y));
	}

	int ncon = vInputPoints.size();
	for (int i = 0; i<(int)m_base_corners.size(); i++)
	{
		addvextex(vInputPoints, Point2(m_base_corners[i].x, m_base_corners[i].y));

	}


	Fade_2D dt;
	dt.insert(vInputPoints);
	std::vector<Segment2> vSegments;

	for (int i = 0; i < ncon-1; i++)
	{
		vSegments.push_back(Segment2(vInputPoints[i], vInputPoints[i+1]));
		
	}

	
	ConstraintGraph2* pCG = dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	dt.applyConstraintsAndZones();

	Visualizer2 vis2("example3_withConstraints.ps");
	dt.show(&vis2);

	Color colorBlue(0.0, 0.0, 1.0, 0.01);
	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt.getTrianglePointers(vAllDelaunayTriangles);
	int ti = 0;
	ITMMesh::Triangle * trivec=  mesh->triangles->GetData(MEMORYDEVICE_CPU);

	uvlist.clear();

	for (std::vector<Triangle2*>::iterator it = vAllDelaunayTriangles.begin(); it != vAllDelaunayTriangles.end(); ++it)
	{
		Triangle2* pT(*it);
		Point2 c2 = pT->getBarycenter();
		int cx = c2.x();
		int cy = c2.y();
		if (cx < 0 || cx >= imgDims.width || cy < 0 || cy >= imgDims.height)
			continue;

		int idx = (cx + w* cy);
	  

		Vector4u p = mask[idx];
		if (p.x >0 || p.y > 0 || p.z > 0 )
		{
			Circle2 cc(c2, 2);
			vis2.addObject(cc, colorBlue);


		// An alternative method (just to show how to access the vertices) would be:
			Point2* p0=pT->getCorner(0);
			Point2* p1=pT->getCorner(1);
			Point2* p2=pT->getCorner(2);

			//int i1 = p0->x() + w*((int)p0->y());
			//int i2 = p1->x() + w*((int)p1->y());
			//int i3 = p2->x() + w*((int)p2->y());
			Vector2i pc0=Vector2i(p1->x()+p2->x(),p1->y()+p2->y())/2;
			Vector2i pc1 = Vector2i(p0->x() + p2->x(), p0->y() + p2->y()) / 2;
			Vector2i pc2 = Vector2i(p1->x() + p0->x(), p1->y() + p0->y()) / 2;
			trivec[ti].p0.z = estivalue(depthData_in, Vector2i(p0->x(),p0->y()), pc0);
			trivec[ti].p1.z = estivalue(depthData_in, Vector2i(p1->x(), p1->y()), pc1);
			trivec[ti].p2.z = estivalue(depthData_in, Vector2i(p2->x(), p2->y()), pc2);
			cv::Point2f uv0 = cv::Point2f(p0->x() / w, p0->y() / h);
			uvlist.push_back(uv0);
			cv::Point2f uv1 = cv::Point2f(p1->x() / w, p1->y() / h);
			uvlist.push_back(uv1);

			cv::Point2f uv2 = cv::Point2f(p2->x() / w, p2->y() / h);
			uvlist.push_back(uv2);

			
			//if (trivec[ti].p0.z > 0 && trivec[ti].p1.z > 0 && trivec[ti].p2.z > 0)
			{

				trivec[ti].p0.x = trivec[ti].p0.z * (p0->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p0.y = trivec[ti].p0.z * (p0->y() - intrinparam.w) / intrinparam.y;

				trivec[ti].p1.x = trivec[ti].p1.z * (p1->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p1.y = trivec[ti].p1.z * (p1->y() - intrinparam.w) / intrinparam.y;

				trivec[ti].p2.x = trivec[ti].p2.z * (p2->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p2.y = trivec[ti].p2.z * (p2->y() - intrinparam.w) / intrinparam.y;

				std::string text( toString(ti) );
				Label label_pNeigT(c2, text, false);
				vis2.addObject(label_pNeigT, colorBlue);

				ti++;
			}
		}
	}

	mesh->noTotalTriangles = ti;
	bmesh = true;

}

#else


template <class InputIterator>
void insert_with_info(CDT& cdt, InputIterator first, InputIterator last)
{
	std::vector<std::ptrdiff_t> indices;
	std::vector<Point2> points;
	std::ptrdiff_t index = 0;

	for (InputIterator it = first; it != last; ++it) {
		points.push_back(*it);
		indices.push_back(index++);
		
	}

	CGAL::spatial_sort(indices.begin(), indices.end(), Search_traits(&(points[0]), cdt.geom_traits()));

	CDT::Vertex_handle v_hint;
	CDT::Face_handle hint;
	for (typename std::vector<std::ptrdiff_t>::const_iterator
		it = indices.begin(), end = indices.end();
		it != end; ++it) {
		v_hint = cdt.insert(points[*it], hint);
		if (v_hint != CDT::Vertex_handle()) {
			v_hint->info() = *it;
			hint = v_hint->face();
		}
		else
		{
			DEBUG_OUTPUT_STREAM << "wrong";
		}
	}
}

void MeshFusion::constructMesh(ITMMesh * mesha, MyTri * tridata)
{

	ITMFloatImage * depth_in = proDepth;
	Vector4f* normalsMap = mainView->depthNormal->GetData(MEMORYDEVICE_CPU);

	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;


	Vector2i imgDims = depth_in->noDims;
	int w = imgDims.x;
	int h = imgDims.y;
	int maxlen = (imgDims.y - 2)*w;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *mask = segImage->GetData(MEMORYDEVICE_CPU);


	cv::Mat a;


	


	CDT dt;
	
	insert_with_info(dt, vInputPoints.begin(), vInputPoints.end());


	for (int i = 0; i < ncon - 1; i++)
	{
		dt.insert_constraint(vInputPoints[i], vInputPoints[i + 1]);
		//vSegments.push_back(Segment2(vInputPoints[i], vInputPoints[i + 1]));

	}

	//uvlist.clear();
	//meshVertex.clear();

	CDT::Face_handle face;
	CDT::Face_handle neighbor;
	int idx = 0;

	for (
		CDT::Vertex_iterator vit = dt.vertices_begin(),
		vend = dt.vertices_end();
		vit != vend; ++vit)
	{
		//cv::Point3f  pout;
		Point2 p0 = vit->point();
		//int ndx=vit->info();
		//if (ndx != idx)
			vit->info() = idx;

		//
		//pout.z = -1;// estivalue(depthData_in, Vector2i(p0.x(), p0.y()), NULL);

		

		//cv::Point2f uv0 = cv::Point2f(p0.x() / w, p0.y() / h);
		////uvlist.push_back(uv0);
		////meshVertex[idx].x = pout.x;//   push_back(pout);
		////meshVertex[idx].y = pout.y;
		tridata->meshVertex[idx].z = -1;
		//meshVertex[idx].s0 = uv0.x;
		//meshVertex[idx].t0 = uv0.y;



		idx++;
	}
	tridata->totalVertex = idx;



	CDT::Face_iterator it = dt.faces_begin(),
		beyond = dt.faces_end();


	//int ti = 0;
	//ITMMesh::Triangle * trivec = mesh->triangles->GetData(MEMORYDEVICE_CPU);


	//CDT::Finite_vertices_iterator vit;
	int center = 0;
	int nface = 0;
	fstream fout;
	fout.open("debug.txt", ios::out);
	
	while (it != beyond) {
		face = it;                                //get face
		++it;                                      //advance the iterator
		int count = 0;                             //initialize counter
	//	for (int i = 0; i<3; ++i) {      //for index 0,1,2

		CDT::Vertex_handle vh0 = face->vertex(0);
		CDT::Vertex_handle vh1 = face->vertex(1);
		CDT::Vertex_handle vh2 = face->vertex(2);
		
		
		Point2 p0= vh0->point();
		Point2 p1= vh1->point();
		Point2 p2= vh2->point();


		Point2 c2=CGAL::centroid(p0, p1, p2);
		
		int cx = c2.x();
		int cy = c2.y();
		if (cx < 0 || cx >= imgDims.width || cy < 0 || cy >= imgDims.height)
			continue;

		int idx = (cx + w* cy);


		Vector4u p = mask[idx];
		Vector4f corr3Dnormal;
		if (p.x >0 || p.y > 0 || p.z > 0)
		{
			fout << p0 << "  moveto" << endl;
			fout << p1 << "  lineto" << endl;
			fout << p1 << "  moveto" << endl;
			fout << p2 << "  lineto" << endl;
			fout << p2 << "  moveto" << endl;
			fout << p0 << "  lineto" << endl;

			int v0 = vh0->info();
			int v1 = vh1->info();
			int v2 = vh2->info();
			
			if (tridata->meshVertex[v0].z < 0)
			{
				tridata->meshVertex[v0].z = estivalue(depthData_in, Vector2i(p0.x(), p0.y()), Vector2i(c2.x(), c2.y()));
				tridata->meshVertex[v0].x = tridata->meshVertex[v0].z * (p0.x() - intrinparam.z) / intrinparam.x;
				tridata->meshVertex[v0].y = tridata->meshVertex[v0].z * (p0.y() - intrinparam.w) / intrinparam.y;
				corr3Dnormal = interpolateBilinear_withHoles(normalsMap, Vector2f(p0.x(), p0.y()), imgDims);

				tridata->meshVertex[v0].nx = corr3Dnormal.x;
				tridata->meshVertex[v0].ny = corr3Dnormal.y;
				tridata->meshVertex[v0].nz = corr3Dnormal.z;

				tridata->meshVertex[v0].s0 = p0.x() / w;
				tridata->meshVertex[v0].t0 = p0.y() / h;
				tridata->meshVertex[v0].z = tridata->meshVertex[v0].z  -center;
			}
			if (tridata->meshVertex[v1].z < 0)
			{
				tridata->meshVertex[v1].z = estivalue(depthData_in, Vector2i(p1.x(), p1.y()), Vector2i(c2.x(), c2.y()));
				tridata->meshVertex[v1].x = tridata->meshVertex[v1].z * (p1.x() - intrinparam.z) / intrinparam.x;
				tridata->meshVertex[v1].y = tridata->meshVertex[v1].z * (p1.y() - intrinparam.w) / intrinparam.y;
				tridata->meshVertex[v1].s0 = p1.x() / w;
				tridata->meshVertex[v1].t0 = p1.y() / h;
				tridata->meshVertex[v1].z = tridata->meshVertex[v1].z -center;
				corr3Dnormal = interpolateBilinear_withHoles(normalsMap, Vector2f(p1.x(), p1.y()), imgDims);

				tridata->meshVertex[v1].nx = corr3Dnormal.x;
				tridata->meshVertex[v1].ny = corr3Dnormal.y;
				tridata->meshVertex[v1].nz = corr3Dnormal.z;

			}
			if (tridata->meshVertex[v2].z < 0)
			{
				tridata->meshVertex[v2].z = estivalue(depthData_in, Vector2i(p2.x(), p2.y()), Vector2i(c2.x(), c2.y()));
				tridata->meshVertex[v2].x = tridata->meshVertex[v2].z * (p2.x() - intrinparam.z) / intrinparam.x;
				tridata->meshVertex[v2].y = tridata->meshVertex[v2].z * (p2.y() - intrinparam.w) / intrinparam.y;
				tridata->meshVertex[v2].s0 = p2.x() / w;
				tridata->meshVertex[v2].t0 = p2.y() / h;
				tridata->meshVertex[v2].z = tridata->meshVertex[v2].z -center;
				corr3Dnormal = interpolateBilinear_withHoles(normalsMap, Vector2f(p2.x(), p2.y()), imgDims);

				tridata->meshVertex[v2].nx = corr3Dnormal.x;
				tridata->meshVertex[v2].ny = corr3Dnormal.y;
				tridata->meshVertex[v2].nz = corr3Dnormal.z;

			}

			tridata->meshTri[nface] = v0;
			tridata->meshTri[nface+1] = v1;
			tridata->meshTri[nface+2] = v2;// .push_back(cv::Point3i(v0, v1, v2));
			nface = nface + 3;
		}
	}
	tridata->totalFace = nface;
	fout.close();

	//Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	
	tridata->project(NULL, intrinparam);
	edgeRefine(tridata);


//	mesh->noTotalTriangles = ti;
//	bmesh = true;


}

#endif