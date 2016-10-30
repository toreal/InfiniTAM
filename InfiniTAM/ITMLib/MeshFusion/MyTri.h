#pragma once

#include "opencv2/core/core.hpp"
#include "../Utils/ITMLibDefines.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
# include <iostream> 
#include <fstream>

using namespace std;

typedef CGAL::Simple_cartesian<float> K;
typedef K::Point_2          Point2;



namespace ITMLib
{
	namespace Objects
	{

		

		class halfEdge
		{
		public :
			halfEdge() { pair = NULL; needflip = -1; }
			int v1;
			int v2;
			int belongface;
			halfEdge * pair;
			int       needflip;//-1: not set. 0: no , 1:yes
			float     cost;
			void checkPair(halfEdge * v)
			{
				if (pair != NULL)
					return;

				if ((v1 == v->v1 && v2 == v->v2) || (v1 == v->v2 && v2 == v->v1))
				{
					pair = v;
					v->pair = this;
				}
			}
		};

		struct MyVertex
		{
			float x, y, z;        //Vertex
			float s0, t0;         //Texcoord0
			float nx, ny, nz;     //Normal

		};

		class MyTri
		{
		public:
			MyVertex meshVertex[2048];
			ushort  meshTri[2048 * 3];
			Point2  meshProj[2048];
			int  boundary[2048];
			int  contour[1024];
			float meshDepth[2048];
			halfEdge meshEdge[2048];
			bool  stat[2048];
			int totalVertex = 0;
			int totalFace = 0;
			int totaledge=0;
			int nboundary;
			int ncontour;

			void copyFrom(MyTri * data);
			int opposite(halfEdge e);
			void output2d(char * fn);
			void project(Matrix4f * m, Vector4f intrinRGB);
			void buildHalfEdge(void * mfdata);

			float findError(MyTri &, std::vector<cv::Point3f> * diff,
				std::vector<cv::Point3f> * node = NULL, std::vector<cv::Point3f> * normal = NULL);


		};


	}

}