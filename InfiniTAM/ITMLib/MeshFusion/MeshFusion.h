#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "opencv2/core/core.hpp"
#include "../Objects/ITMMesh.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMPose.h"
#include "../Engine/ITMTracker.h"

#include "../../ORUtils/MemoryBlock.h"
#include "MyTri.h"

typedef CGAL::Polyhedron_3<K >      Polyhedron;



namespace ITMLib
{
	namespace Objects
	{

		
	/*	struct MyIndices
		{
			ushort pindices[3];
		}*/

		class MeshFusion
		{
		private :
			Polyhedron mymesh;
			ITMLib::Engine::ITMTracker * tracker =NULL;
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
