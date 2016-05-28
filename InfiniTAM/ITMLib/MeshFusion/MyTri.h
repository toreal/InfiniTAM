#pragma once

#include "opencv2/core/core.hpp"
#include "../Utils/ITMLibDefines.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Simple_cartesian<float> K;
typedef K::Point_2          Point2;


namespace ITMLib
{
	namespace Objects
	{

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
			bool  stat[2048];
			int totalVertex = 0;
			int totalFace = 0;
			void copyFrom(MyTri * data)
			{
				memcpy(this, data, sizeof(MyTri));
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