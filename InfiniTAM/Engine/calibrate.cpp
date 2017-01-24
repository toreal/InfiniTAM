

#include "calibrate.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

//#include <boost/timer/timer.hpp>
//#include <boost/format.hpp>


void readPara()
{

	const string outputFileName = "images\\out_camera_data.xml";

	Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;
	Size imageSize;

	Mat qview;
	

	qview = imread("images\\test0.jpg");


	if (!readCameraParams(outputFileName, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs))
	{


	}

	cout << tvecs[0] << endl;

	vector<Point3f>  newpoints;
	
	newpoints.push_back(Point3f(0, 0, 0));

	Mat newMatrix;
	newMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
	


	Mat distorted_points2d;
	//projectPoints(Mat(newpoints), cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), cameraMatrix, distCoeffs, distorted_points2d);
	projectPoints(Mat(newpoints), rvecs[0], tvecs[0], newMatrix,
		distCoeffs, distorted_points2d);

	
	Scalar cc3 = cv::Scalar(0, 0, 255, 255); //BGRA
	int cBlockSize = 5;
	Point2f pt = distorted_points2d.at<Point2f>(0);

	rectangle(qview, cv::Rect(pt.x - cBlockSize, pt.y - cBlockSize, cBlockSize * 2 + 1, cBlockSize * 2 + 1), cc3);

	imshow("", qview);
	cvWaitKey();
	imwrite("debug.png", qview);


}



static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}


static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };



int runcalib()
{

	 Settings s;
	  const string inputSettingsFile =  "images\\default.xml";


	  
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
	  clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
	const char ESC_KEY = 27;
    for(int i = 0;;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints,rvecs, tvecs))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }
      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 && mode != CALIBRATED )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints,rvecs, tvecs);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);


		char key;
		if ( i ==0 ) 
			key =waitKey();
		else 
			key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }


	 return 1;
}


int mymain(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "images\\default.xml";



    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

	if (! readCameraParams(  s.outputFileName, imageSize,  cameraMatrix, distCoeffs,  rvecs,   tvecs ))
	{


	}

	//boost::timer::cpu_timer timer;

	//timer.start();



    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
		Mat  nview, nmap1, nmap2;
		Mat newMatrix;
		newMatrix= getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
           newMatrix, imageSize, CV_16SC2, map1, map2);

	   float ss = s.squareSize ;

		vector<Point3f>  newpoints;		
		vector<Point2f> imagePoints2;

		float bw=169;//195;//169;//ss*5;
		float bh=230;//265;//228;//ss*7;
		float bt=31.5;//19;//32;//ss;

		float newx = ss*7; 
		float newy =ss*5;

		newpoints.push_back(Point3f(0,newy -bw,0));   //oldbase (0,0,0)  viewed from the newbase (0,newy,0) 
		newpoints.push_back(Point3f(bh,newy-bw,0));   // x 
		newpoints.push_back(Point3f(0,newy,0)); //y
		newpoints.push_back(Point3f(bh,newy ,0)); //x,y
		newpoints.push_back(Point3f(bh,newy,-bt));  //x,y
		newpoints.push_back(Point3f(0,newy-bw,-bt));  //oldbase
		newpoints.push_back(Point3f(bh,newy-bw,-bt));  //x
		newpoints.push_back(Point3f(0,newy,-bt));
		newpoints.push_back(Point3f(0,0,0));

        //for(int i = 0; i < (int)s.imageList.size(); i++ )
		int i=0;
        {
            view = imread("images\\captured.png", 1);
			//view = imread(s.imageList[i], 1);
           // if(view.empty())
            //    continue;
            remap(view, rview, map1, map2, INTER_LINEAR);

		

			/*distCoeffs.at<double>(0) = 0;
			distCoeffs.at<double>(1) = 0;
			distCoeffs.at<double>(2) = 0;
			distCoeffs.at<double>(3) = 0;
			distCoeffs.at<double>(4) = 0;*/
		
			 projectPoints( Mat(newpoints), rvecs[i], tvecs[i], newMatrix,
                       distCoeffs, imagePoints2);


			  nmap1.create( view.size(), CV_32FC1 );  
			  nmap2.create( view.size(), CV_32FC1 );  

			  float scale=4.0f;

			vector<Point3f>  map3d;		
		    vector<Point2f> mapp;


			//boost::timer::cpu_times elapsed = timer.elapsed(); // nano seconds...

			//float t = (elapsed.wall / 1000.0f / 1000.0f / 1000.0f) / (float)100;

			//float fps = 1.0f / t ;

			//if (t > 0.001) {

			//	std::cout << "t=" << t << ", fps=" << fps << std::endl;

			//}

			//timer.start();



			  for( int j = 0; j < nmap1.rows; j++ )  
			  { 
					for( int i = 0; i < nmap1.cols; i++ )     
					{
						  if ( (i < scale*bw +50) && (j < scale*bh +50)) 
						  {
			
						map3d.push_back(Point3d(bh-j/scale ,newy -bw+i/scale,-bt));
					

						}
						 
					}
	            }

		 //elapsed = timer.elapsed(); // nano seconds...

			// t = (elapsed.wall / 1000.0f / 1000.0f / 1000.0f) / (float)100;

			// fps = 1.0f / t ;

			//if (t > 0.001) {

			//	std::cout << "t=" << t << ", fps=" << fps << std::endl;

			//}

		//	timer.start();


			  	
						 projectPoints( Mat(map3d), rvecs[0], tvecs[0], newMatrix,
								distCoeffs, mapp);

// elapsed = timer.elapsed(); // nano seconds...

	//		 t = (elapsed.wall / 1000.0f / 1000.0f / 1000.0f) / (float)100;

		/*	 fps = 1.0f / t ;

			if (t > 0.001) {

				std::cout << "t=" << t << ", fps=" << fps << std::endl;

			}

			timer.start();
*/


                int k = 0 ; 
			    for( int j = 0; j < nmap1.rows; j++ )  
			  { 
					for( int i = 0; i < nmap1.cols; i++ )     
					{
						  if ( (i < scale*bw +50) && (j < scale*bh +50)) 
						  {
				
						  nmap1.at<float>(j,i) = mapp[k].x ;  
						  nmap2.at<float>(j,i) = mapp[k].y ;
						  k++;

						}else
						{
							 nmap1.at<float>(j,i) = 0;  
							  nmap2.at<float>(j,i) = 0 ;  
						}
						 
					}
	            }

		//	 elapsed = timer.elapsed(); // nano seconds...

		/*	 t = (elapsed.wall / 1000.0f / 1000.0f / 1000.0f) / (float)100;

			 fps = 1.0f / t ;

			if (t > 0.001) {

				std::cout << "t=" << t << ", fps=" << fps << std::endl;

			}

			timer.start();*/


			// updatmap( nmap1,nmap2, imagePoints2 ,bw,bh,scale);
			  remap(rview, nview, nmap1, nmap2, INTER_LINEAR);


			  //在變正的封面上劃線
			   line(nview,Point2f(0,0*scale),Point2f(bw*scale,0*scale), Scalar( 0, 0, 255 ),2,8);
			  line(nview,Point2f(0,60*scale),Point2f(bw*scale,60*scale), Scalar( 0, 255, 0 ),2,8);
			  line(nview,Point2f(0,77*scale),Point2f(bw*scale,77*scale), Scalar( 0, 0, 255 ),2,8);
			  line(nview,Point2f(0,bh*scale),Point2f(bw*scale,bh*scale), Scalar( 0, 0, 255 ),2,8);

			 ellipse( rview, imagePoints2[8],Size( 5, 5 ),0,0,360,Scalar( 255, 0, 0 ),2,8 );
			 line(rview,imagePoints2[0],imagePoints2[1], Scalar( 0, 255, 0 ),2,8);
			 line(rview,imagePoints2[0],imagePoints2[2], Scalar( 0, 0, 255 ),2,8);
			 line(rview,imagePoints2[1],imagePoints2[3], Scalar( 0, 0, 255 ),2,8);
			 line(rview,imagePoints2[2],imagePoints2[3], Scalar( 0, 0, 255 ),2,8);
			 line(rview,imagePoints2[3],imagePoints2[4], Scalar( 0, 255, 255 ),2,8);
			 line(rview,imagePoints2[0],imagePoints2[5], Scalar( 0, 255, 255 ),2,8);
			 line(rview,imagePoints2[5],imagePoints2[4], Scalar( 0, 255, 255 ),2,8);
			 line(rview,imagePoints2[1],imagePoints2[6], Scalar( 0, 255, 255 ),2,8);
			 line(rview,imagePoints2[2],imagePoints2[7], Scalar( 0, 255, 255 ),2,8);

			 
			   imshow("map View", nview);
			   imwrite("map.png",nview);
            imshow("Image View", rview);
			imwrite("result.png",rview);
//
		 //elapsed = timer.elapsed(); // nano seconds...

			// t = (elapsed.wall / 1000.0f / 1000.0f / 1000.0f) / (float)100;

			// fps = 1.0f / t ;

			//if (t > 0.001) {

			//	std::cout << "t=" << t << ", fps=" << fps << std::endl;

			//}

			//timer.start();



            char c = (char)waitKey();
           /* if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;*/
        }
    }


    return 0;
}

void Barycentric(Point2f p, Point2f a, Point2f b, Point2f c, float &u, float &v, float &w)
{
    Point2f v0 = b - a;
	Point2f v1 = c - a;
	Point2f v2 = p - a;
    float d00 = v0.dot( v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot( v1);
    float d20 = v2.dot( v0);
    float d21 = v2.dot( v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

Point2f  cartesian( Point2f p0, Point2f p1, Point2f p2, float u, float v, float w) 
{
      return u * p0 + v * p1 + w * p2;
}

//7,4,5,6
void  updatmap(Mat& nmap1, Mat& nmap2,vector<Point2f> & imagePoints2 , int bw,int bh, float scale )
{
	

	float u,v,w;
	Point2f ret ;

	

	for( int j = 0; j < nmap1.rows; j++ )  
    { 
		for( int i = 0; i < nmap1.cols; i++ )     
		{
			 ret= Point2f(0,0);
			  if ( (j < scale*bw) || (i < scale*bh)) 
			  {
				    Barycentric(Point2f(i,j),Point2f(0,0),Point2f(scale*bw,0),Point2f(0,scale*bh),u,v,w);
					if (( u+v+w)<=1.001 && ( u>=0 && v>=0 && w>=0)) 
					{
						ret = cartesian(imagePoints2[6],imagePoints2[4],imagePoints2[5],u,v,w);
				   
					}else
					{
					   
					 Barycentric(Point2f(i,j),Point2f(scale*bw,scale*bh),Point2f(scale*bw,0),Point2f(0,scale*bh),u,v,w);
					 if (( u+v+w)<=1.001 && ( u>=0 && v>=0 && w>=0)) 
						  ret = cartesian(imagePoints2[7],imagePoints2[4],imagePoints2[5],u,v,w);
				     
					}
				  

				  

			  }
			  nmap1.at<float>(j,i) = ret.x ;  
              nmap2.at<float>(j,i) = ret.y ;  
		}
	}
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}




// Print camera parameters to the output file
 bool readCameraParams( string  s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                               vector<Mat>& rvecs,  vector<Mat>& tvecs
                               )
{
    FileStorage fs( s, FileStorage::READ );


	if ( !fs.isOpened())
		return false;

	fs["image_Width"]>>imageSize.width;
	fs["image_Height"]>>imageSize.height;

    fs["Camera_Matrix"]>> cameraMatrix ;
    fs["Distortion_Coefficients"] >> distCoeffs;

	 Mat bigmat;
	 fs["Extrinsic_Parameters"]>>bigmat;
	 int nrows=bigmat.rows;
	 
	
	 for( int i = 0 ; i < nrows; i++)
	 {
		    Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));
			rvecs.push_back(r);
            tvecs.push_back(t);
	 }

	 return true;
}



bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints,vector<Mat>& rvecs, vector<Mat> &tvecs )
{
   // vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}
