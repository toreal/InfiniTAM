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
#include <boost/assign/list_of.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

using namespace boost;
using namespace boost::accumulators;

typedef accumulator_set<double, features<tag::density> > acc;
typedef iterator_range<std::vector<std::pair<double, double> >::iterator > histogram_type;

//using namespace std;
using namespace ITMLib::Objects;
using namespace cv;

cv::Mat MeshFusion::m_Debug;
cv::Mat MeshFusion::m_matDebugVector;

int     MeshFusion::m_nDebugX = 0;
int     MeshFusion::m_nDebugY = 0;
int     MeshFusion::m_nDebugVectorIdx = -1;
//std::stringstream MeshFusion::_ssDebug;
MeshFusion::DebugLog MeshFusion::_ssDebug;
std::stringstream    MeshFusion::DebugLog::ssOut;


#define SQR(x) ((x)*(x))

#define CULL_ENABLE
#define CULL_MAX_SQRDIST 3.0f
#define CULL_MIN_SQRDIST 0.01f

void MeshFusion::MeshFusion_InitTracking( void )
{
	m_backup = m_corners;
	m_backup2 = m_base_corners;

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
    
    const int cOriginalPara = 1; //1: for Original Parameters. 0: for new/dense Parameters.
    double qualityLevel = cOriginalPara? 0.01 : 0.03;
    
    // minDistance – The minimum possible Euclidean distance between the returned corners
    double minDistance = cOriginalPara? 20.: 10.;
    
    // blockSize – Size of the averaging block for computing derivative covariation
    // matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
    int blockSize = cOriginalPara? 3 : 7 ;
    
    // useHarrisDetector – Indicates, whether to use operator or cornerMinEigenVal()
    bool useHarrisDetector = false;
    
    // k – Free parameter of Harris detector
    double k = 0.04;
    
    cv::Size winSize=cv::Size(21,21);
    
    int maxLevel=3;
    
    if (m_bfirst)
	{
		m_bfirst = false;
		cv::goodFeaturesToTrack(m_image, m_corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k);

#ifdef CV_PYRAMID
        cv::buildOpticalFlowPyramid(m_image, m_prevPyr, winSize, maxLevel, true);
#endif
        
        m_base_corners = m_corners;
        m_base_image = m_image.clone();
	}
      else
    {
        m_pre_corners = m_corners;

#ifdef CV_PYRAMID
        cv::buildOpticalFlowPyramid(m_image, m_currPyr, winSize, maxLevel, true);
        calcOpticalFlowPyrLK(m_prevPyr, m_currPyr, m_pre_corners, m_corners, m_status, m_err, winSize, maxLevel);
        
        // Reverse Check
        std::vector< cv::Point2f > _rpre_corners;
        std::vector< uchar > _rstatus;
        std::vector<float> _rerr;
        calcOpticalFlowPyrLK(m_currPyr, m_prevPyr, m_corners, _rpre_corners, _rstatus, _rerr, winSize, maxLevel);
        
        assert(_rpre_corners.size()==m_pre_corners.size());
        
        bool bRCheckFirst=true;
        for (size_t i=0;i<m_pre_corners.size();i++)
        {
            if (m_status[i] == 1)
            {
                if (_rstatus[i]==1)
                {
                    float fdistsqr = SQR(m_pre_corners[i].x - _rpre_corners[i].x) + SQR(m_pre_corners[i].y - _rpre_corners[i].y);;
                    
                    if (fdistsqr>1)
                    {
                        m_status[i]=0;
                        if (bRCheckFirst)
                            DEBUG_OUTPUT_STREAM << "Corner "<< i , bRCheckFirst=false;
                        else
                            DEBUG_OUTPUT_STREAM << ", " << i;
                    }
                }
                else
                {
                    m_status[i]=_rstatus[i];
                    if (bRCheckFirst)
                        DEBUG_OUTPUT_STREAM << "Corner *"<< i , bRCheckFirst=false;
                    else
                        DEBUG_OUTPUT_STREAM << ", *" << i;
                }
            }
        }
        if (!bRCheckFirst)
            DEBUG_OUTPUT_STREAM << " Removed!" << DEBUG_ENDL;
        
        m_prevPyr.swap(m_currPyr);
#else
        calcOpticalFlowPyrLK(m_pre_image, m_image, m_pre_corners, m_corners, m_status, m_err, winSize, maxLevel);
#endif

        // Calculate max and average SQR of motion vector distance
        maxdis = 0;
        float fdistsqr = 0;
        float fAverageDistSqr = 0;
        int k=0;
        
        for (int i=0;i<(int)m_status.size();i++)
        {
            if (m_status[i]==1)
            {
                fdistsqr = SQR(m_base_corners[i].x - m_corners[i].x) + SQR(m_base_corners[i].y - m_corners[i].y);
                
                fAverageDistSqr += fdistsqr;
                
                if (fdistsqr>maxdis)
                    maxdis = fdistsqr;
                
                k++;
            }
            
        }

        
        //Delete fail to track features & distance too large or too small
        std::vector< cv::Point2f > __corners, __pre_corners,_base_corners;
        std::vector<cv::Point3f> objpos;
        std::vector<float> _err;

        
        if (k>0)
        {

            fAverageDistSqr /= k;

            k=0;
            for (int i=0;i<(int)m_status.size();i++)
            {
                if (m_status[i]==1)
                {
#ifdef CULL_ENABLE
                    fdistsqr = SQR(m_base_corners[i].x - m_corners[i].x) + SQR(m_base_corners[i].y - m_corners[i].y);
                    if (fdistsqr <= fAverageDistSqr*CULL_MAX_SQRDIST && fdistsqr >= fAverageDistSqr*CULL_MIN_SQRDIST)
#endif
                    {
                        __corners.push_back(m_corners[i]);
                        __pre_corners.push_back(m_pre_corners[i]);
                        _base_corners.push_back(m_base_corners[i]);
                        _err.push_back(m_err[i]);
                        objpos.push_back(objectPoints[i]);
                        k++;
                    }
                }
                
            }
            
        }

        m_err               = _err;
        m_corners           = __corners;
        m_pre_corners       = __pre_corners;
        m_base_corners      = _base_corners;
        m_latest_paired_corners_base    = _base_corners;
        m_latest_paired_corners_prev    = __pre_corners;
        m_latest_paired_corners_curr    = __corners;
        m_latest_paired_corners_err     = _err;
        m_latest_paired_base_image      =   m_base_image.clone();
        m_latest_paired_prev_image      =   m_pre_image.clone();
        m_latest_paired_curr_image      =   m_image.clone();
        
        objectPoints    = objpos;
		
    }
    
    return EXIT_SUCCESS;
}

void MeshFusion::ClearDebugWindow(void)
{
    m_Debug = cv::Mat::zeros(800,600,CV_8UC3);

    m_nDebugX=15;
    m_nDebugY=15;
}

void MeshFusion::ShowDebugWindow(void)
{
    OutputDebugText(MeshFusion::DebugLog::ssOut.str().c_str());
    MeshFusion::DebugLog::ssOut.str("");
    MeshFusion::DebugLog::ssOut.clear();
    
    cv::namedWindow( "Debug", CV_WINDOW_NORMAL );
    cv::imshow( "Debug", m_Debug );
}

void MeshFusion::OutputDebugPlot(const int nX, const int nY, const std::vector<float> & plotdata)
{

    cv::rectangle(m_Debug, cv::Point2f(m_nDebugX,m_nDebugY), cv::Point2f(m_nDebugX + nX, m_nDebugY + nY), Scalar(64,64,64));

    if (plotdata.size()<2)
        return;
    
    std::vector< cv::Point> cvplot;
    
    for (size_t i=0;i<plotdata.size();i++)
    {
        cvplot.push_back(cv::Point2f(m_nDebugX+nX*i/(plotdata.size()-1),m_nDebugY+nY*(1-plotdata[i])));
    }
    
    auto current = cvplot.begin();
    auto next = std::next(current, 1);
    for (; next != cvplot.end(); current++, next++)
    {
        cv::line(m_Debug, *current, *next, cv::Scalar(0,0,255),1,cv::LINE_AA);
    }
    m_nDebugY += nY+15+5;
}

void MeshFusion::OutputDebugText(const char *str)
{
    
    if (str==NULL) return;
    
    cv::String cvStr(str);
    
    size_t nS  = 0;
    size_t nNewL = cvStr.find('\n',0);

    
    while (nNewL!=cv::String::npos)
    {
        cv::String strout = cvStr.substr(nS,nNewL-nS);
        cv::putText(m_Debug, strout,cv::Point(m_nDebugX,m_nDebugY), 0?CV_FONT_HERSHEY_PLAIN:CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255),1,LINE_AA,false);
        m_nDebugY+=15;
        
        nS = nNewL+1;
        nNewL = cvStr.find('\n',nS);
    }

    if (nS<cvStr.size())
    {
        cv::String strout = cvStr.substr(nS);
        cv::putText(m_Debug, strout,cv::Point(m_nDebugX,m_nDebugY), 0?CV_FONT_HERSHEY_PLAIN:CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255),1,LINE_AA,false);
        m_nDebugY+=15;
    }
}

void MeshFusion::MeshFusion_DebugTracking( void )
{
#define CurrCorner m_latest_paired_corners_curr
#define PrevCorner m_latest_paired_corners_base
#define CurrImage m_latest_paired_curr_image
#define PrevImage m_latest_paired_base_image
    
    if (CurrCorner.size()!=0)
    {
        std::vector <cv::Scalar> colors = boost::assign::list_of
            (cv::Scalar(0,0,255,200))(cv::Scalar(0,255,0,200)) (cv::Scalar(255,0,0,200))
            (cv::Scalar(0,255,255,200))(cv::Scalar(255,0,255,200))(cv::Scalar(255,255,0,200));
        

#if 1
#define NDIV    1
        Mat img_now, img_pre;
        cvtColor(CurrImage,img_now,COLOR_GRAY2BGR);
        cvtColor(PrevImage,img_pre,COLOR_GRAY2BGR);
#else
#define NDIV    2
        Mat img_now, img_pre, img_tmp;

        cvtColor(CurrImage,img_tmp,COLOR_GRAY2BGRA);
        cv::resize(img_tmp, img_now,cv::Size(CurrImage.cols/2,CurrImage.rows));
        cvtColor(PrevImage,img_tmp,COLOR_GRAY2BGR);
        cv::resize(img_tmp, img_pre,cv::Size(CurrImage.cols/2,CurrImage.rows));
#endif
        m_matDebugVector = cv::Mat::zeros(img_now.rows,img_now.cols*2,CV_8UC3);
        
        img_pre.copyTo(m_matDebugVector(Rect(0,0,img_pre.cols,img_pre.rows)));
        img_now.copyTo(m_matDebugVector(Rect(img_pre.cols,0,img_now.cols,img_now.rows)));

        acc myAccumulator( tag::density::num_bins = 20, tag::density::cache_size = 10);
        for (size_t i=0;i<m_latest_paired_corners_err.size();i++)
            myAccumulator(m_latest_paired_corners_err[i]);
        histogram_type hist = density(myAccumulator);
        
        const int cBlockSize = 7;
        int nColorIdx = 0;
        for (int i=0;i<(int)CurrCorner.size();i++)
        {
            cv::Point pt_txt1(               PrevCorner[i].x/NDIV - cBlockSize, PrevCorner[i].y - cBlockSize - 5);
            cv::Point pt_txt2(img_now.cols + CurrCorner[i].x/NDIV - cBlockSize, CurrCorner[i].y - cBlockSize - 5);
            cv::Point pt_pre (               PrevCorner[i].x/NDIV , PrevCorner[i].y);
            cv::Point pt_now (img_now.cols + CurrCorner[i].x/NDIV , CurrCorner[i].y);
            if (m_nDebugVectorIdx<0 || m_nDebugVectorIdx==i)
            {

                size_t j=0;
                for (;j<hist.size();j++)
                {
                    if (hist[j].first>m_latest_paired_corners_err[i])
                        break;
                }
                cv::Scalar _color1 = colors[nColorIdx];
                cv::Scalar _color2 = _color1 * ((40-j)/40.0);

                std::stringstream ssout1,ssout2;
                ssout1 << i;
                ssout2 << "[" << j << "/22]" << m_latest_paired_corners_err[i];
                cv::putText(m_matDebugVector,ssout1.str(),pt_txt1, CV_FONT_HERSHEY_SIMPLEX, 0.3, _color1,1,LINE_AA,false);
                cv::putText(m_matDebugVector,ssout2.str(),pt_txt2, CV_FONT_HERSHEY_SIMPLEX, 0.3, _color1,1,LINE_AA,false);
                
                cv::line(m_matDebugVector, cv::Point(pt_pre.x+cBlockSize,pt_pre.y), cv::Point(pt_now.x-cBlockSize,pt_now.y), _color2,1,cv::LINE_AA);

                cv::rectangle(m_matDebugVector, cv::Rect(pt_pre.x - cBlockSize , pt_pre.y - cBlockSize,cBlockSize*2+1,cBlockSize*2+1), _color2);
                cv::rectangle(m_matDebugVector, cv::Rect(pt_now.x - cBlockSize , pt_now.y - cBlockSize,cBlockSize*2+1,cBlockSize*2+1), _color2);
            }
            nColorIdx = (nColorIdx+1)%colors.size();
        }
        
        //cv::namedWindow( "DebugTracking", CV_WINDOW_NORMAL );
        //imshow("DebugTracking", m_matDebugVector);
        
    }
    
}

void MeshFusion::MeshFusion_DrawVector(float fstartx, float fstarty, float fwidth, float fheight)
{
    
    glColor3f(1.0f, 1.0f, 0.0f);
    
    glPointSize(0.4f);
    glBegin(GL_POINTS);
    for (int i=0;i<(int)m_corners.size();i++)
    {
        if (m_pre_corners.size()!=0)
            if (m_status[i]==0)
                glColor3f(1.0f, 0.0f, 0.0f);
            else
                glColor3f(0.0f, 1.0f, 0.0f);
        else
            glColor3f(1.0f, 1.0f, 0.0f);

        float x = m_corners[i].x/m_image.cols;
        float y = 1-m_corners[i].y/m_image.rows;
        glVertex2f(fstartx+x*fwidth, fstarty+y*fheight);
    }
    glEnd();

    // Draw green vector from m_latest_paired_corners_curr to m_latest_paired_corners_base
    if (m_latest_paired_corners_base.size()!=0)
    {
        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES); {
            for (int i=0;i<(int)m_latest_paired_corners_base.size();i++)
            {
                float x1 = m_latest_paired_corners_base[i].x/m_image.cols;
                float y1 = 1-m_latest_paired_corners_base[i].y/m_image.rows;
                float x2 = m_latest_paired_corners_curr[i].x/m_image.cols;
                float y2 = 1-m_latest_paired_corners_curr[i].y/m_image.rows;
                glVertex2f(fstartx+x1*fwidth, fstarty+y1*fheight);
                glVertex2f(fstartx+x2*fwidth, fstarty+y2*fheight);
            }
        }
        glEnd();
    }

//    // Draw green vector from m_base_corner to m_corner
//    if (m_pre_corners.size()!=0)
//    {
//        glColor3f(0.0f, 1.0f, 0.0f);
//        glBegin(GL_LINES); {
//            for (int i=0;i<(int)m_corners.size();i++)
//            {
//                if (m_status[i]==1)
//                {
//                    float x1 = m_corners[i].x/m_image.cols;
//                    float y1 = 1-m_corners[i].y/m_image.rows;
//                    float x2 = m_base_corners[i].x/m_image.cols;
//                    float y2 = 1-m_base_corners[i].y/m_image.rows;
//                    glVertex2f(fstartx+x1*fwidth, fstarty+y1*fheight);
//                    glVertex2f(fstartx+x2*fwidth, fstarty+y2*fheight);
//                    
//                }
//            }
//        }
//        glEnd();
//    }

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
}