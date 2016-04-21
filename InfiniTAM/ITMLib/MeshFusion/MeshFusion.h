#pragma once


#ifndef __METALC__
#include <stdlib.h>
#endif

#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"


namespace ITMLib
{
	namespace Objects
	{
		class MeshFusion
		{
		public:
			const int MAXNODE = 10000;
			//features list
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


			void sortpoint(ITMUChar4Image * draw);

			int MeshFusion_Tracking(ITMUChar4Image *);
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);


		};
	}
}
