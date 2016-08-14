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
			float meshDepth[2048];
			halfEdge meshEdge[2048];
			bool  stat[2048];
			int totalVertex = 0;
			int totalFace = 0;
			int totaledge=0;
			void copyFrom(MyTri * data)
			{
				memcpy(this, data, sizeof(MyTri));
			}

			int opposite(halfEdge e)
			{
				int v[3];
				for (int i = 0; i < 3; i++)
				{
					v[i] = meshTri[3 * e.belongface + i];
				}
				for (int i = 0; i < 3; i++)
				{
					if (v[i] == e.v1)
					{
						if (v[(i + 1) % 3] == e.v2)
						{
							return v[(i + 2) % 3];
						}
						else if (v[(i + 2) % 3] == e.v2)
						{
							return v[(i + 1) % 3];
						}
	
					}
					else if (v[i] == e.v2)
					{
						if (v[(i + 1) % 3] == e.v1)
						{
							return v[(i + 2) % 3];
						}
						else if (v[(i + 2) % 3] == e.v1)
						{
							return v[(i + 1) % 3];
						}
					}

				}

				return -1;
			}

			void output2d(char * fn)
			{

				fstream fout;
				fout.open(fn, ios::out);
				int nface = totalFace / 3;
				for (int i = 0; i < nface; i++)
				{
					Point2 p0 = meshProj[meshTri[3*i]];
					Point2 p1 = meshProj[meshTri[3*i+1]];
					Point2 p2 = meshProj[meshTri[3*i+2]];

					fout << p0 << "  moveto" << endl;
					fout << p1 << "  lineto" << endl;
					fout << p1 << "  moveto" << endl;
					fout << p2 << "  lineto" << endl;
					fout << p2 << "  moveto" << endl;
					fout << p0 << "  lineto" << endl;
				}
				fout.close();

			}

			float findError(MyTri &, std::vector<cv::Point3f> * diff, 
				std::vector<cv::Point3f> * node=NULL, std::vector<cv::Point3f> * normal=NULL);
			void project(Matrix4f * m, Vector4f intrinRGB)
			{
				for (int i = 0; i < totalVertex; i++)
				{

					Vector3f vpos(meshVertex[i].x, meshVertex[i].y, meshVertex[i].z);
					Vector3f npos;
					if (m != NULL)
						npos = (*m)*vpos;
					else
						npos = vpos;


					float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
					float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;
					//meshVertex.push_back(cv::Point3f(pos.x(),pos.y(),pos.z()));
					meshProj[i] = Point2(ix, iy);
					meshDepth[i] = npos.z;
				}//end of for 


			}
		};


	}

}