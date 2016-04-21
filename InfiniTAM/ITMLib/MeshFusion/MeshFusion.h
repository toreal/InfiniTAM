#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "opencv2/core/core.hpp"

#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"


namespace ITMLib
{
	namespace Objects
	{
		class MeshFusion
		{


			std::vector< cv::Point2f > _corners, _pre_corners;
			cv::Mat _image, _pre_image;
			std::vector<uchar> _status;
			std::vector<float> _err;



		public:
			const int MAXNODE = 10000;
			//segmented image
			ITMUChar4Image *segImage;
			Vector2i *      pointlist;
			int             npoint;

			Vector2i *      sellist;
			int             selp;

			MeshFusion() {
				segImage = NULL;
				pointlist = new Vector2i[MAXNODE];
			};
			~MeshFusion() {  
				if (segImage != NULL)
					delete segImage;
				delete pointlist;

			};

			//processing silhouette point
			void sortpoint(ITMUChar4Image * draw);

			int MeshFusion_Tracking(ITMUChar4Image * , ITMUChar4Image *);
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);


		};
	}
}
