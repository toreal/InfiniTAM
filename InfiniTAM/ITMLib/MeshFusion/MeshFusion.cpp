


#include "MeshFusion.h"
#include <Fade_2D.h>
#include <algorithm>
#include <vector> 
#include <iterator> 
# include <iostream> 
#include "../../ORUtils/psimpl.h"
//#include <math.h>
#include <opencv\cvwimage.h>

using namespace ITMLib::Objects;
using namespace std;
using namespace GEOM_FADE2D;


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
	
	std::cout <<result.size() << std::endl;

	if (draw == NULL)
		return;

	Vector4u* data = draw->GetData(MEMORYDEVICE_CPU);
	int i = 0; 
	for (vector<int>::iterator it = result.begin(); it != result.end(); ++it) {
		int x = *it;
		++it;
		int y = *it;
		//cout << x <<"," << y << endl;
		
		this->sellist[i].x = x;
		this->sellist[i].y = y;
		i++;
	}

	this->selp = i;

}

void MeshFusion::buildProjDepth()
{

	if (proDepth == NULL  )
	proDepth=	new ITMFloatImage(mainView->depth->noDims, true, false);


	float* dp = proDepth->GetData(MEMORYDEVICE_CPU);
	int lens = proDepth->noDims.x * proDepth->noDims.y;
	memset(dp, 0x00, sizeof(float) *lens);

	float* dd = mainView->depth->GetData(MEMORYDEVICE_CPU);
	int xlens = mainView->depth->noDims.x;
	int ylens = mainView->depth->noDims.y;
	for (int nx = 0; nx < xlens; nx++)
		for (int ny = 0; ny < ylens; ny++)
		{
			float val = dd[nx + ny*xlens];

			if (val > 0)
			{
				float x = nx*1.0 / _image.cols;
				float y = 1 - ny*1.0 / _image.rows;
//				glVertex2f(fstartx + x*fwidth, fstarty + y*fheight);
			}
		}

}
	 

void MeshFusion::constructMesh(ITMMesh * mesh )
{
	ITMFloatImage * depth_in = mainView->depth; 
	Vector4f  intrinparam = mainView->calib->intrinsics_d.projectionParamsSimple.all;


	Vector2i imgDims = depth_in->noDims;
	int w = imgDims.x;
	int maxlen = (imgDims.y - 2)*w;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *mask= segImage->GetData(MEMORYDEVICE_CPU);
	

	cv::Mat a;
	

	std::vector<Point2> vInputPoints;

	for (int i = 0; i < selp; i++)
	{
		vInputPoints.push_back(Point2(sellist[i].x,sellist[i].y));
	}

	for (int i = 0; i<(int)_corners.size(); i++)
	{
		vInputPoints.push_back(Point2(_corners[i].x, _corners[i].y));

	}


	Fade_2D dt;
	dt.insert(vInputPoints);
	std::vector<Segment2> vSegments;

	for (int i = 0; i < selp-1; i++)
	{
		vSegments.push_back(Segment2(vInputPoints[i], vInputPoints[i+1]));
		
	}

	
	ConstraintGraph2* pCG = dt.createConstraint(vSegments, CIS_CONFORMING_DELAUNAY);
	dt.applyConstraintsAndZones();

	Visualizer2 vis2("example3_withConstraints.ps");
	dt.show(&vis2);

	Color colorBlue(0.0, 0.0, 1.0, 0.01);
	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt.getTrianglePointers(vAllDelaunayTriangles);
	int ti = 0;
	ITMMesh::Triangle * trivec=  mesh->triangles->GetData(MEMORYDEVICE_CPU);

	for (std::vector<Triangle2*>::iterator it = vAllDelaunayTriangles.begin(); it != vAllDelaunayTriangles.end(); ++it)
	{
		Triangle2* pT(*it);
		Point2 c2 = pT->getBarycenter();
		
		int idx = (c2.x() + w* c2.y());
		Vector4u p = mask[idx];
		if (p.x >0 || p.y > 0 || p.z > 0 )
		{
			Circle2 cc(c2, 2);
			vis2.addObject(cc, colorBlue);


		// An alternative method (just to show how to access the vertices) would be:
			Point2* p0=pT->getCorner(0);
			Point2* p1=pT->getCorner(1);
			Point2* p2=pT->getCorner(2);

			int i1 = p0->x() + w*p0->y();
			int i2 = p1->x() + w*p1->y();
			int i3 = p2->x() + w*p2->y();
			trivec[ti].p0.z = depthData_in[i1];
			if (trivec[ti].p0.z > 0.5 || trivec[ti].p0.z < 0)
				trivec[ti].p0.z = 0.25;
			/*while (trivec[ti].p0.z < 0 && i1 < maxlen)
			{
				i1 += w;
				trivec[ti].p0.z = depthData_in[i1];
			}*/


			trivec[ti].p1.z = depthData_in[i2];
			if (trivec[ti].p1.z > 0.5 || trivec[ti].p1.z < 0)
				trivec[ti].p1.z = 0.25;

			trivec[ti].p2.z = depthData_in[i3];
			if (trivec[ti].p2.z > 0.5 || trivec[ti].p2.z < 0)
				trivec[ti].p2.z = 0.25;

			//if (trivec[ti].p0.z >= 0 && trivec[ti].p1.z >= 0 && trivec[ti].p2.z >= 0)
			{

				trivec[ti].p0.x = trivec[ti].p0.z * (p0->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p0.y = trivec[ti].p0.z * (p0->y() - intrinparam.w) / intrinparam.y;

				trivec[ti].p1.x = trivec[ti].p1.z * (p1->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p1.y = trivec[ti].p1.z * (p1->y() - intrinparam.w) / intrinparam.y;

				trivec[ti].p2.x = trivec[ti].p2.z * (p2->x() - intrinparam.z) / intrinparam.x;
				trivec[ti].p2.y = trivec[ti].p2.z * (p2->y() - intrinparam.w) / intrinparam.y;

				ti++;
			}
		}
	}

	mesh->noTotalTriangles = ti;


}