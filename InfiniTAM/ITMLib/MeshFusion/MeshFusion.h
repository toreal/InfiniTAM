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

//#define DEBUG_OUTPUT_STREAM std::cout
#define DEBUG_OUTPUT_STREAM (MeshFusion::GetDebugStream())
//#define DEBUG_ENDL          "\n"
#define DEBUG_ENDL          std::endl

#define CV_PYRAMID


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
			class DebugLog
			{
			public:

				static std::stringstream ssOut;

				template <class T>
				DebugLog &operator<<(const T &v)
				{
					ssOut << v;

					return *this;
				}
				// function that takes a custom stream, and returns it
				typedef DebugLog& (*DebugLogManipulator)(DebugLog&);

				// take in a function with the custom signature
				DebugLog& operator<<(DebugLogManipulator manip)
				{
					// call the function, and return it's value
					return manip(*this);
				}
				// define the custom endl for this stream.
				// note how it matches the `MyStreamManipulator`
				// function signature
				static DebugLog& endl(DebugLog& stream)
				{
					// print a new line
					ssOut << std::endl;

					// do other stuff with the stream
					// std::cout, for example, will flush the stream
					stream << "Called DebugLog::endl!" << std::endl;

					return stream;
				}

				// this is the type of std::cout
				typedef std::basic_ostream<char, std::char_traits<char> > CoutType;

				// this is the function signature of std::endl
				typedef CoutType& (*StandardEndLine)(CoutType&);

				// define an operator<< to take in std::endl
				DebugLog& operator<<(StandardEndLine manip)
				{
					// call the function, but we cannot return it's value
					manip(ssOut);

					OutputDebugText(ssOut.str().c_str());
					ssOut.str("");
					ssOut.clear();

					return *this;
				}
			};

		private:
			Polyhedron mymesh;
			ITMLib::Engine::ITMTracker * tracker = NULL;
			//Polyhedron currMesh;
			//std::vector< cv::Point2f > uvlist;
			//std::vector<cv::Point3f> meshVertex;


			//std::vector<cv::Point3f> meshVertex;
			//std::vector<cv::Point3i> meshTri;

			// For Tracker
			std::vector< cv::Point2f > m_latest_paired_corners_base, m_latest_paired_corners_prev, m_latest_paired_corners_curr;  //M record lastest paired corners for visualize only
			std::vector<float>  m_latest_paired_corners_err;
			cv::Mat             m_latest_paired_curr_image, m_latest_paired_prev_image, m_latest_paired_base_image;

			cv::Mat             m_image, m_pre_image, m_base_image, d_image;
#ifdef CV_PYRAMID
			std::vector<cv::Mat> m_prevPyr, m_currPyr;
#endif

			std::vector<uchar>  m_status;
			std::vector<float>  m_err;
			bool                m_bfirst = true;
			std::vector<Point2> vInputPoints;
			std::vector<Point2> constrainbeg;
			std::vector<Point2> constrainend;
			static int          m_nDebugX, m_nDebugY;
			static int          m_nDebugVectorIdx;

			float estivalue(const float * data, Vector2i, Vector2i);
			void edgeRefine(MyTri * tridata);


		public:

			std::vector< cv::Point2f > m_corners, m_pre_corners, m_base_corners, m_backup, d_corners;

			std::vector<cv::Point3f> objectPoints;
			std::vector<cv::Point3f> newPoints;
			MyTri mytriData;
			MyTri currTri;
			bool bmesh = false;
			int      shift = 0;
			const int MAXNODE = 10000;
			ITMView     *mainView = NULL;
			//segmented image
			ITMUChar4Image *segImage;
			ITMFloatImage * proDepth = NULL;

			//silhouette features
			Vector2i *      pointlist;
			int             npoint;

			Vector2i *      sellist;
			int             selp;
			int             ncon;

			ITMPose        posd;
			MeshFusion() {
				segImage = NULL;
				pointlist = new Vector2i[MAXNODE];
				sellist = new Vector2i[MAXNODE];
				MeshFusion_InitTracking();
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
			void NormalAndCurvature(ITMView **view_ptr, bool modelSensorNoise);
			void rotateAngle(ITMPose * posd);
			void ReCoordinateSystem(ITMPose * posd);
			cv::Mat rigid_transformPose(cv::Mat A, cv::Mat B, ITMPose * posd);
			void goodFeature(ITMPose * posd);
			void estimatePose(ITMPose * posd);
			void refinePose(ITMPose * posd);
			void genContour(char * str);

			void meshUpdate(ITMMesh * mesh, ITMPose *, MyTri * tridata);
			void meshMerge(ITMMesh * mesh, ITMPose *, MyTri * tridata);
			void intoMesh(Matrix4f invm, MyTri *scanned);
			void buildMesh(MyTri *);
			////////////////////////////
			//  Image feature tracking
			void MeshFusion_InitTracking(void);
			int MeshFusion_Tracking(float & maxdis, int);
			//ui codes. opengl code inside for drawing motion vectors. Typically, call this func in glutDisplayFunction
			void MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight);
			void MeshFusion_DebugTracking(void);

			void MeshFusion_Model(float fstartx, float fstarty, float fwidth, float fheight, bool getImageType, ITMPose *pose, ITMIntrinsics *intrinsics);
			void writeMesh(char *);
			void Generate3DPoints(std::vector<cv::Point2f> &imp, std::vector<cv::Point3f> &d3p);

			void DebugVectorIdxAll(void) { m_nDebugVectorIdx = -1; }
			void DebugVectorIdxInc(void) { if (m_latest_paired_corners_base.size()>0) m_nDebugVectorIdx = (m_nDebugVectorIdx + 1) % m_latest_paired_corners_base.size(); }
			void DebugVectorIdxDec(void) { if (m_latest_paired_corners_base.size()>0) m_nDebugVectorIdx = (m_nDebugVectorIdx + m_latest_paired_corners_base.size() - 1) % m_latest_paired_corners_base.size(); }

			//            static std::stringstream _ssDebug;
			static DebugLog _ssDebug;
			static cv::Mat      m_matDebugVector;
			static cv::Mat      m_matDebugConsole;
			static cv::Mat      m_normal;
			//static inline std::stringstream & GetDebugStream() { return _ssDebug; }

			static inline DebugLog& GetDebugStream() { return _ssDebug; }

			static void ClearDebugWindow(void);
			static void ShowDebugWindow(void);
			static void OutputDebugText(const char *str);
			static void OutputDebugPlot(const int nX, const int nY, const std::vector<float> & plotdata);

		};
	}
}