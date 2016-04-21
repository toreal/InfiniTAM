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

//using namespace std;
using namespace ITMLib::Objects;
using namespace cv;

#define SQR(x) ((x)*(x))

bool bfirst = true;

int MeshFusion::MeshFusion_Tracking(ITMUChar4Image * draw)//
{


	int w = draw->noDims.x;
	int h = draw->noDims.y;
	Vector4u* img=(draw->GetData(MEMORYDEVICE_CPU));
	
    cv::Mat input(h,w,CV_8UC4,img);
    
//    cv::namedWindow( "input", CV_WINDOW_NORMAL );
//    cv::imshow( "input", input );
    
	if (bfirst)
	{
		
	//	_image = input.clone();
	}
	else
	{
		_pre_image = _image.clone();

	}


    
	if (_status.size() == 0) {

		_pre_corners = _corners;

	}
         else
    {
        _pre_corners.clear();
        for(int i=0,k=0;i<_corners.size();i++)
        {
            if (_status[i]==1)
            {
                _pre_corners.push_back(_corners[i]);
                k++;
            }
        }
    }
    
    cvtColor(input,_image,COLOR_BGR2GRAY);

    //_image2 = input.clone();
    //_image2.convertTo(_image,CV_8UC1,1);
    
    //_corners.clear();
    
    
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
    
    // mask – The optional region of interest. If the image is not empty (then it
    // needs to have the type CV_8UC1 and the same size as image ), it will specify
    // the region in which the corners are detected
    cv::Mat mask;
    
    // blockSize – Size of the averaging block for computing derivative covariation
    // matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
    int blockSize = 3;
    
    // useHarrisDetector – Indicates, whether to use operator or cornerMinEigenVal()
    bool useHarrisDetector = false;
    
    // k – Free parameter of Harris detector
    double k = 0.04;
    
    if (bfirst)
	{

		//_corners.resize(maxCorners);
		bfirst = false;
		cv::goodFeaturesToTrack(_image, _corners, maxCorners, qualityLevel, minDistance, noArray(), blockSize, useHarrisDetector, k);

	}
      else
    {
        cv::goodFeaturesToTrack( _image, _corners, maxCorners, qualityLevel, minDistance, noArray(), blockSize, useHarrisDetector, k );
        int ci=0;
        while(_pre_corners.size()<(unsigned int)maxCorners && (int)_corners.size()>ci)
        {
            
            int i;
            for (i=0;i<(int)_pre_corners.size();i++)
            {
                float fdist = SQR(_pre_corners[i].x - _corners[ci].x) + SQR(_pre_corners[i].y - _corners[ci].y);
                if (fdist<SQR(minDistance))
                    break;
            }
            if (i==(int)_pre_corners.size())
                _pre_corners.push_back(_corners[ci]);
            ci++;
        }
            
        calcOpticalFlowPyrLK(_pre_image, _image, _pre_corners, _corners, _status, _err);
    }
//    for( size_t i = 0; i < _corners.size(); i++ )
//    {
//        if (i==0)
//            cv::circle( _image2, _corners[i], 5, CV_RGB(255,0,0), -1 );
//        else
//            cv::circle( _image2, _corners[i], 5, CV_RGB(255,255,0), -1 );
//    }
//    
//    cv::namedWindow( "output", CV_WINDOW_NORMAL );
//    cv::imshow( "output", _image2 );
    
    return EXIT_SUCCESS;   return 0;
}

void MeshFusion::MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight)
{
    glColor3f(1.0f, 1.0f, 0.0f);
    
    glPointSize(3.4f);
    glBegin(GL_POINTS);
    for (int i=0;i<(int)_corners.size();i++)
    {
        if (_pre_corners.size()!=0 && _status[i]==0)
            glColor3f(1.0f, 0.0f, 0.0f);
        else
            glColor3f(1.0f, 1.0f, 0.0f);

        float x = _corners[i].x/_image.cols;
        float y = 1-_corners[i].y/_image.rows;
        glVertex2f(fstartx+x*fwidth, fstarty+y*fheight);
    }
    glEnd();

    if (_pre_corners.size()!=0)
    {
        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINES); {
            for (int i=0;i<(int)_corners.size();i++)
            {
                if (_status[i]==1)
                {
                    float x1 = _corners[i].x/_image.cols;
                    float y1 = 1-_corners[i].y/_image.rows;
                    float x2 = _pre_corners[i].x/_image.cols;
                    float y2 = 1-_pre_corners[i].y/_image.rows;
                    glVertex2f(fstartx+x1*fwidth, fstarty+y1*fheight);
                    glVertex2f(fstartx+x2*fwidth, fstarty+y2*fheight);

                }
            }
        }
    }
    glEnd();
}