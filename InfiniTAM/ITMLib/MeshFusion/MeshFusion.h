#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "opencv2/core/core.hpp"
#include "../Objects/ITMMesh.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMPose.h"
#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Simple_cartesian<float> K;
typedef K::Point_2          Point2;
typedef CGAL::Polyhedron_3<K >      Polyhedron;



namespace ITMLib
{
	namespace Objects
	{

		struct MyVertex
		{
			float x, y, z;        //Vertex
			//float nx, ny, nz;     //Normal
			float s0, t0;         //Texcoord0
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
		
			float findError(MyTri &, std::vector<float> &);
			void project(Matrix4f * m,Vector4f intrinRGB)
			{
				for (int i = 0; i < totalVertex; i++)
				{

					Vector3f vpos(meshVertex[i].x, meshVertex[i].y,meshVertex[i].z);
					Vector3f npos;
					if (m != NULL)
						npos = (*m)*vpos;
					else
						npos = vpos;


					float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
					float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;
					//meshVertex.push_back(cv::Point3f(pos.x(),pos.y(),pos.z()));
					meshProj[i]=Point2(ix, iy);
					meshDepth[i]=npos.z;
				}//end of for 


			}
		};

	/*	struct MyIndices
		{
			ushort pindices[3];
		}*/

		class MeshFusion
		{
		private :
			Polyhedron mymesh;
			//Polyhedron currMesh;
			//std::vector< cv::Point2f > uvlist;
			//std::vector<cv::Point3f> meshVertex;
		

			//std::vector<cv::Point3f> meshVertex;
			//std::vector<cv::Point3i> meshTri;

			// For Tracker
			std::vector< cv::Point2f > m_corners, m_pre_corners,m_base_corners,m_backup,m_backup2;
			std::vector<cv::Point3f> objectPoints;
			cv::Mat             m_image, m_pre_image;
			std::vector<uchar>  m_status;
			std::vector<float>  m_err;
			bool                m_bfirst = true;
			std::vector<Point2> vInputPoints;
				
			float estivalue(const float * data, Vector2i ,Vector2i);
			

		public:

			MyTri mytriData;
			MyTri currTri;
			bool bmesh = false;
			int      shift = 0;
			const int MAXNODE = 10000;
			ITMView     *mainView=NULL;
			//segmented image
			ITMUChar4Image *segImage;
			ITMFloatImage * proDepth=NULL;

			//silhouette features
			Vector2i *      pointlist;
			int             npoint;

			Vector2i *      sellist;
			int             selp;
			int             ncon;

			MeshFusion() {
				segImage = NULL;
				pointlist = new Vector2i[MAXNODE];
				sellist = new Vector2i[MAXNODE];
			};
			~MeshFusion() {  
				if (segImage != NULL)
					delete segImage;
				
				delete pointlist;
				delete sellist;
			};

			bool find3DPos(cv::Point2f p, cv::Point3f&);
			//processing silhouette point
			void sortpoint(ITMUChar4Image * draw);
			void constructMesh(ITMMesh *, MyTri * tridata);
			void buildProjDepth();
			void estimatePose(ITMPose * posd);
			void refinePose(ITMPose * posd);

			void meshUpdate(ITMMesh * mesh, ITMPose *, MyTri * tridata);
			void buildMesh( MyTri *);
            ////////////////////////////
            //  Image feature tracking
            void MeshFusion_InitTracking( void );
			int MeshFusion_Tracking(float & maxdis );
            //ui codes. opengl code inside for drawing motion vectors. Typically, call this func in glutDisplayFunction
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);
			void MeshFusion_Model(float fstartx, float fstarty, float fwidth, float fheight, bool getImageType, ITMPose *pose, ITMIntrinsics *intrinsics);
			void writeMesh(char *);
			void Generate3DPoints();

		};
	}
}
