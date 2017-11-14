#pragma once



#include "opencv2/core/core.hpp"
#include "MyTri.h"



namespace ITMLib
{
	namespace Objects
	{


		class DomainSurface
		{

		public:
			DomainSurface();
			~DomainSurface();

			//for surface
			cv::Point3f  centerbuf[1000];
			cv::Point3f  normaldbuf[1000];
			float    dbuf[1000];
			int      ndface = 0;
		//	int belongTo[1000];

			//for boundary line
			cv::Point3f rpoint[1000];
			cv::Point3f rnormal[1000];
			float       t1[1000];
			float       t2[1000];

			int         c1[1000];
			int         c2[1000];


			int     rline=0;


			//private:
			void mesh(MyTri & data);

			int findLine(int a, int b);
			void drawline(cv::Mat & buf, int i, Vector4f  intrinRGB);
			void makeTri(cv::Point3f * depth3d, cv::Mat& buf,
				Vector4f  intrinparam );
			
		};


	}
}