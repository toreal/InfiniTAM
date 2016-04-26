#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "opencv2/core/core.hpp"
#include "../Objects/ITMMesh.h"
#include "../Objects/ITMView.h"
#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Polyhedron_3<K >      Polyhedron;


namespace ITMLib
{
	namespace Objects
	{
		class MeshFusion
		{
		private :
			Polyhedron mymesh;
			std::vector< cv::Point2f > uvlist;
            // For Tracker
			std::vector< cv::Point2f > m_corners, m_pre_corners,m_base_corners;
			cv::Mat             m_image, m_pre_image;
			std::vector<uchar>  m_status;
			std::vector<float>  m_err;
			bool                m_bfirst = true;
            
			float estivalue(const float * data, int index);
			std::vector<cv::Point3f> Generate3DPoints();

		public:
			
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

			//processing silhouette point
			void sortpoint(ITMUChar4Image * draw);
			void constructMesh(ITMMesh *);
			void buildProjDepth();
			void estimatePose();

			void meshUpdate(ITMMesh * mesh);
            
            ////////////////////////////
            //  Image feature tracking
            void MeshFusion_InitTracking( void );
			int MeshFusion_Tracking(float & maxdis );
            //ui codes. opengl code inside for drawing motion vectors. Typically, call this func in glutDisplayFunction
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);
			void MeshFusion_Model(float fstartx, float fstarty, float fwidth, float fheight);
			void writeMesh(char *);
		};
	}
}
