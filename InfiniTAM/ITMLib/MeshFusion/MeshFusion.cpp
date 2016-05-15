


#include "MeshFusion.h"


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


void addvextex(std::vector<Point2> &vInputPoints, Point2 p)
{
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

void MeshFusion::buildProjDepth()
{
	//ITMFloatImage * floatImage= new ITMFloatImage(mainView->depth->noDims, true, false);

	if (proDepth == NULL  )
	proDepth=	new ITMFloatImage(mainView->depth->noDims, true, false);

	Vector4f intrinD= mainView->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f intrinRGB= mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

	Matrix4f d2rgb = mainView->calib->trafo_rgb_to_depth.calib_inv;

	float* dp = proDepth->GetData(MEMORYDEVICE_CPU);
	int lens = proDepth->noDims.x * proDepth->noDims.y;
	memset(dp, 0x00, sizeof(float) *lens);

	float* dd = mainView->depth->GetData(MEMORYDEVICE_CPU);
	int xlens = mainView->depth->noDims.x;
	int ylens = mainView->depth->noDims.y;
	for (int nx = 0; nx < xlens; nx++)
		for (int ny = 0; ny < ylens; ny++)
		{
			float pz = dd[nx + ny*xlens]*1000;

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
					if (dp[ix + iy*xlens] > 0 && ABS(dp[ix + iy*xlens] - prgb.z) > 10)
						cout << dp[ix + iy*xlens] << "," << prgb.z << endl;
					dp[ix + iy*xlens] = prgb.z;
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

		float delx = abs(p2.x - p1.x);
		float dely = abs(p2.y - p1.y);
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
			cout << "wrong";
		}
	}
}

void MeshFusion::constructMesh(ITMMesh * mesh)
{

	ITMFloatImage * depth_in = proDepth;
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
		meshVertex[idx].z = -1;
		//meshVertex[idx].s0 = uv0.x;
		//meshVertex[idx].t0 = uv0.y;



		idx++;
	}
	totalVertex = idx;



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
			
			if (meshVertex[v0].z < 0)
			{
				meshVertex[v0].z = estivalue(depthData_in, Vector2i(p0.x(), p0.y()), Vector2i(c2.x(), c2.y()));
				meshVertex[v0].x = meshVertex[v0].z * (p0.x() - intrinparam.z) / intrinparam.x;
				meshVertex[v0].y = meshVertex[v0].z * (p0.y() - intrinparam.w) / intrinparam.y;
				meshVertex[v0].s0 = p0.x() / w;
				meshVertex[v0].t0 = p0.y() / h;
				meshVertex[v0].z = meshVertex[v0].z  -center;
			}
			if (meshVertex[v1].z < 0)
			{
				meshVertex[v1].z = estivalue(depthData_in, Vector2i(p1.x(), p1.y()), Vector2i(c2.x(), c2.y()));
				meshVertex[v1].x = meshVertex[v1].z * (p1.x() - intrinparam.z) / intrinparam.x;
				meshVertex[v1].y = meshVertex[v1].z * (p1.y() - intrinparam.w) / intrinparam.y;
				meshVertex[v1].s0 = p1.x() / w;
				meshVertex[v1].t0 = p1.y() / h;
				meshVertex[v1].z = meshVertex[v1].z -center;
			}
			if (meshVertex[v2].z < 0)
			{
				meshVertex[v2].z = estivalue(depthData_in, Vector2i(p2.x(), p2.y()), Vector2i(c2.x(), c2.y()));
				meshVertex[v2].x = meshVertex[v2].z * (p2.x() - intrinparam.z) / intrinparam.x;
				meshVertex[v2].y = meshVertex[v2].z * (p2.y() - intrinparam.w) / intrinparam.y;
				meshVertex[v2].s0 = p2.x() / w;
				meshVertex[v2].t0 = p2.y() / h;
				meshVertex[v2].z = meshVertex[v2].z -center;
			}

			meshTri[nface] = v0;
			meshTri[nface+1] = v1;
			meshTri[nface+2] = v2;// .push_back(cv::Point3i(v0, v1, v2));
			nface = nface + 3;
		}
	}
	totalFace = nface;
	fout.close();

//	mesh->noTotalTriangles = ti;
	bmesh = true;


}

#endif