//
//  Tracker.cpp
//  InfiniTAM
//
//  Created by Chien-Chang Ho on 4/16/16.
//
//

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#include "MeshFusion.h"

#include <cstdlib>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#define TEST_CGAL
#ifdef TEST_CGAL
#include <CGAL/Simple_cartesian.h>
#endif

//using namespace std;
using namespace ITMLib::Objects;
using namespace cv;

#define SQR(x) ((x)*(x))

void MeshFusion::MeshFusion_InitTracking( void )
{
    m_bfirst = true;
    m_pre_corners.clear();
    m_corners.clear();
    m_base_corners.clear();
}


int MeshFusion::MeshFusion_Tracking( float & maxdis)//
{
	ITMUChar4Image * draw = mainView->rgb;

	maxdis = 0;

	int w = draw->noDims.x;
	int h = draw->noDims.y;
	Vector4u* img=draw->GetData(MEMORYDEVICE_CPU);
	Vector4u* segimg = segImage->GetData(MEMORYDEVICE_CPU);
    cv::Mat input(h,w,CV_8UC4,img);
	cv::Mat seginput(h, w, CV_8UC4, segimg);

    //cv::namedWindow( "input", CV_WINDOW_NORMAL );
    //cv::imshow( "input", input );
    
    if (!m_bfirst && !m_image.empty())
        m_pre_image = m_image.clone();
	
    
	// mask – The optional region of interest. If the image is not empty (then it
	// needs to have the type CV_8UC1 and the same size as image ), it will specify
	// the region in which the corners are detected
	cv::Mat mask;

    cvtColor(input,m_image,COLOR_BGR2GRAY);
	cvtColor(seginput, mask, COLOR_BGR2GRAY);
    
    // maxCorners – The maximum number of corners to return. If there are more corners
    // than that will be found, the strongest of them will be returned
    int maxCorners = 500;
    
    // qualityLevel – Characterizes the minimal accepted quality of image corners;
    // the value of the parameter is multiplied by the by the best corner quality
    // measure (which is the min eigenvalue, see cornerMinEigenVal() ,
    // or the Harris function response, see cornerHarris() ).
    // The corners, which quality measure is less than the product, will be rejected.
    // For example, if the best corner has the quality measure = 1500,
    // and the qualityLevel=0.01 , then all the corners which quality measure is
    // less than 15 will be rejected.
    double qualityLevel = 0.01;
    
    // minDistance – The minimum possible Euclidean distance between the returned corners
    double minDistance = 20.;
    
    // blockSize – Size of the averaging block for computing derivative covariation
    // matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
    int blockSize = 3;
    
    // useHarrisDetector – Indicates, whether to use operator or cornerMinEigenVal()
    bool useHarrisDetector = false;
    
    // k – Free parameter of Harris detector
    double k = 0.04;
    
    if (m_bfirst)
	{
		m_bfirst = false;
		cv::goodFeaturesToTrack(m_image, m_corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k);
        m_base_corners = m_corners;
	}
      else
    {
  
        m_pre_corners = m_corners;

        calcOpticalFlowPyrLK(m_pre_image, m_image, m_pre_corners, m_corners, m_status, m_err);

        //Delete fail to track features & calculate SQR of distance
        std::vector< cv::Point2f > __corners, __pre_corners,_base_corners;
        
        maxdis = 0;
        float fdistsqr = 0;
        int k=0;
        for (int i=0;i<(int)m_status.size();i++)
        {
            if (m_status[i]==1)
            {
                __corners.push_back(m_corners[i]);
                __pre_corners.push_back(m_pre_corners[i]);
                _base_corners.push_back(m_base_corners[i]);
                
                fdistsqr = SQR(m_base_corners[i].x - m_corners[i].x) + SQR(m_base_corners[i].y - m_corners[i].y);
                
                if (fdistsqr>maxdis)
                    maxdis = fdistsqr;
                k++;
            }
        }
        
        m_corners     = __corners;
        m_pre_corners = __pre_corners;
        m_base_corners = _base_corners;

    }
    
    return EXIT_SUCCESS;
}

#ifdef TEST_CGAL
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;

void testcgal()
{
    {
        Point_2 p(1,1), q(10,10);
        std::cout << "p = " << p << std::endl;
        std::cout << "q = " << q.x() << " " << q.y() << std::endl;
        std::cout << "sqdist(p,q) = "
        << CGAL::squared_distance(p,q) << std::endl;
        
        Segment_2 s(p,q);
        Point_2 m(5, 9);
        
        std::cout << "m = " << m << std::endl;
        std::cout << "sqdist(Segment_2(p,q), m) = "
        << CGAL::squared_distance(s,m) << std::endl;
        std::cout << "p, q, and m ";
        switch (CGAL::orientation(p,q,m)){
            case CGAL::COLLINEAR:
                std::cout << "are collinear\n";
                break;
            case CGAL::LEFT_TURN:
                std::cout << "make a left turn\n";
                break;
            case CGAL::RIGHT_TURN:
                std::cout << "make a right turn\n";
                break;
        }
        std::cout << " midpoint(p,q) = " << CGAL::midpoint(p,q) << std::endl;
        
    }
}
#endif

void MeshFusion::MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight)
{
    glColor3f(1.0f, 1.0f, 0.0f);
    
    glPointSize(3.4f);
    glBegin(GL_POINTS);
    for (int i=0;i<(int)m_corners.size();i++)
    {
        if (m_pre_corners.size()!=0 && m_status[i]==0)
            glColor3f(1.0f, 0.0f, 0.0f);
        else
            glColor3f(1.0f, 1.0f, 0.0f);

        float x = m_corners[i].x/m_image.cols;
        float y = 1-m_corners[i].y/m_image.rows;
        glVertex2f(fstartx+x*fwidth, fstarty+y*fheight);
    }
    glEnd();

    if (m_pre_corners.size()!=0)
    {
        // Draw yellow vector from pre_corner to _corner
       glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINES); {
            for (int i=0;i<(int)m_corners.size();i++)
            {
                if (m_status[i]==1)
                {
                    float x1 = m_corners[i].x/m_image.cols;
                    float y1 = 1-m_corners[i].y/m_image.rows;
                    float x2 = m_pre_corners[i].x/m_image.cols;
                    float y2 = 1-m_pre_corners[i].y/m_image.rows;
                    glVertex2f(fstartx+x1*fwidth, fstarty+y1*fheight);
                    glVertex2f(fstartx+x2*fwidth, fstarty+y2*fheight);

                }
            }
        }
        glEnd();

        // Draw green vector from base_corner to _corner
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES); {
            for (int i=0;i<(int)m_corners.size();i++)
            {
                if (m_status[i]==1)
                {
                    float x1 = m_corners[i].x/m_image.cols;
                    float y1 = 1-m_corners[i].y/m_image.rows;
                    float x2 = m_base_corners[i].x/m_image.cols;
                    float y2 = 1-m_base_corners[i].y/m_image.rows;
                    glVertex2f(fstartx+x1*fwidth, fstarty+y1*fheight);
                    glVertex2f(fstartx+x2*fwidth, fstarty+y2*fheight);
                    
                }
            }
        }
        glEnd();
        
    }

	if ( false)// proDepth != NULL)
	{
		glBegin(GL_POINTS);
		glColor3f(0, 1.0f, 0.0f);
		float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
		int xlens = proDepth->noDims.x;
		int ylens = proDepth->noDims.y;
		for (int nx = 0; nx < xlens; nx++)
			for (int ny = 0; ny < ylens; ny++)
			{
				float val = dd[nx + ny*xlens];

				if (val > 0)
				{
					float x = nx*1.0 / m_image.cols;
					float y =1- ny*1.0 / m_image.rows;
					glVertex2f(fstartx + x*fwidth, fstarty + y*fheight);
				}
			}
		glEnd();
	}
    
#ifdef TEST_CGAL
    //test cgal
    testcgal();
#endif
}