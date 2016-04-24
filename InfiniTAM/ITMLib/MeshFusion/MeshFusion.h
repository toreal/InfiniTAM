#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "opencv2/core/core.hpp"
#include "../Objects/ITMMesh.h"
#include "../Objects/ITMView.h"
#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"


namespace ITMLib
{
	namespace Objects
	{
		class MeshFusion
		{
		private :

			std::vector< cv::Point2f > _corners, _pre_corners,base_corners;
			cv::Mat _image, _pre_image;
			std::vector<uchar> _status;
			std::vector<float> _err;

			float estivalue(const float * data, int index);
			std::vector<cv::Point3f> Generate3DPoints();

		public:
			const int MAXNODE = 10000;
			ITMView     *mainView=NULL;
			//segmented image
			ITMUChar4Image *segImage;
			ITMFloatImage * proDepth=NULL;
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
			int MeshFusion_Tracking(float & mindis );
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);


		};
	}
}
