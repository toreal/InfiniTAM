
#include "MeshFusion.h"


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "../../ORUtils/psimpl.h"

using namespace cv;



Point3f raw3d[640 * 480];

void MeshFusion::build3DPoint()
{
	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
	int w = proDepth->noDims.x;
	int h = proDepth->noDims.y;

	
	for (int j = 0;j <h; j++)
		for (int i = 0; i < w; i++)
		{
			int ind = j*w + i;
			float z = estivalue(dd, Vector2i(i, j), Vector2i(-1, 1 - 1));
		

			float x = z * (i - intrinparam.z) / intrinparam.x;
			float y = z * (j - intrinparam.w) / intrinparam.y;

			raw3d[ind] = cv::Point3f(x, y, z);


	}


	int ks = 7;
	int lens = (ks + 1)*(ks + 1);
	//the depth also need to unalias

	for (int j = 0; j <h; j++)
		for (int i = 0; i < w; i++)
		{
			int ind = j*w + i;


			float sx = 0.0f;
			float sy = 0.0f;
			float sz = 0.0f;
			for (int kj = -ks; kj <= ks; kj++)
				for (int ki = -ks; ki <= ks; ki++)
				{
					if (i + ki < 0 || i + ki >= w)
						continue;

					if (j + kj < 0 || j + kj >= h)
						continue;


					int idx = (kj + j)*w + (i + ki);
					Point3f nomald = raw3d[idx];
					sx += nomald.x;
					sy += nomald.y;
					sz += nomald.z;

				}

			sx = sx / lens;
			sy = sy / lens;
			sz = sz / lens;

			depth3d[ind] = Point3f(sx,sy,sz);

		}

}


float  MeshFusion::plane_from_points(std::vector<cv::Point3f> & points ,int pcolor)
{
	int n = points.size();
	//assert(n >= 3, "At least three points required");

	
	 Point3f sum = Point3f(0.0, 0.0, 0.0 );

	 for each (Point3f p in points)
	 {
		 sum = sum + p;
	 }

	Point3f centroid = sum * (1.0 / n) ;

	// Calc full 3x3 covariance matrix, excluding symmetries:
	float xx = 0.0,  xy = 0.0, xz = 0.0;
	float yy = 0.0,  yz = 0.0, zz = 0.0;

	for each (Point3f p in points)
	{
		Point3f r = p - centroid;
	xx += r.x * r.x;
	xy += r.x * r.y;
	xz += r.x * r.z;
	yy += r.y * r.y;
	yz += r.y * r.z;
	zz += r.z * r.z;
	}

	float det_x = yy*zz - yz*yz;
	float det_y = xx*zz - xz*xz;
	float det_z = xx*yy - xy*xy;

	float det_max= det_z;// 
	
	if (det_x >= det_y && det_x >= det_z)
		det_max = det_x;
	else if (det_y >= det_z)
		det_max = det_y;

	//assert(det_max > 0.0, "The points don't span a plane");

	// Pick path with best conditioning:
	float a, b;
	Point3f dir;
	if(	 det_max == det_x) {
			 a = (xz*yz - xy*zz) / det_x;
			 b = (xy*yz - xz*yy) / det_x;
			dir = Point3f(1.0, a, b);
		}
	else if (det_max == det_y) {
		a = (yz*xz - xy*zz) / det_y;
		b = (xy*xz - yz*xx) / det_y;
		dir = Point3f(a, 1.0, b);

	}
		else {
		 a = (yz*xy - xz*yy) / det_z;
		 b = (xz*xy - yz*xx) / det_z;
		 dir = Point3f( a, b,1.0);
		
		}

		double  normal = sqrt(dir.x * dir.x + dir.y*dir.y + dir.z*dir.z);
		dir = dir *(1.0/ normal);


		
		float d = centroid.x*dir.x + centroid.y*dir.y + centroid.z*dir.z;

		float sumerr = 0;

		for each (Point3f p in points)
		{
			float err = p.x*dir.x + p.y*dir.y + p.z*dir.z + d;
			sumerr += err*err;
		}

		sumerr = sumerr / n;
		

		ds.centerbuf[ds.ndface] = centroid;
		ds.normaldbuf[ds.ndface] = dir;
		ds.dbuf[ds.ndface] = -d;
	//	ds.belongTo[ds.ndface] = pcolor;
		ds.ndface++;


	
		int ndo = ds.ndface - 1;

		FILE * fp;
		Vector4f  intrinRGB = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
		Point3f cpos = centroid;

		int sx = (int)(cpos.x *intrinRGB.x / cpos.z + intrinRGB.z);
		int sy = (int)(cpos.y *intrinRGB.y / cpos.z + intrinRGB.w);

		char fn[1024];
		sprintf(fn, "3d\\3d%d_%d_%d.txt", ndo,sx,sy);


		fp = fopen(fn, "w+");



		for ( float x = -30 ; x<30; x++)
			for (float y = -30; y < 30; y++)
			{

				float z = (d - ((cpos.x+x)*dir.x + (cpos.y+y)*dir.y)) / dir.z;
				fprintf(fp, "%f %f %f\n", cpos.x+x, cpos.y+y, z);


		}



		for each (Point3f p in points)
		{

			fprintf(fp, "%f %f %f\n", p.x, p.y, p.z);
		}


		fclose(fp);


		return sumerr;

}

void MeshFusion::genpoint(vector<Point>  contours , std::vector<cv::Point> & sellist)
{
	int count = contours.size();

	std::vector <int> generatedPoints;
	std::vector <int> result;

	for (int i = 0; i < count; i++)
	{
		generatedPoints.push_back(contours[i].x);
		generatedPoints.push_back(contours[i].y);

	}

	// simplify
	std::vector <int>::const_iterator begin = generatedPoints.begin();
	std::vector <int>::const_iterator end = generatedPoints.end();

	psimpl::simplify_douglas_peucker_n<2>(begin, end, count, std::back_inserter(result));

	DEBUG_OUTPUT_STREAM << result.size() << std::endl;

	sellist.clear();

	int i = 0;
	for (vector<int>::iterator it = result.begin(); it != result.end(); ++it) {
		int x = *it;
		++it;
		int y = *it;
		//DEBUG_OUTPUT_STREAM << x <<"," << y << endl;

		Point p(x, y);
		sellist.push_back(p);
		//this->sellist[i].x = x;
		//this->sellist[i].y = y;
		i++;
	}

	//this->selp = i;


}


void MeshFusion::patchSurface( int ind, Mat& gray2 ,int pcolor)
{
	int h = 480;
	int w = 640;

	Mat canny_output;
	Mat src_gray;
	int thresh = 100;

	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat gray1 =Mat(480, 640, CV_8UC1);
	Mat mask = Mat(480+2, 640+2, CV_8UC1);

	try
	{


		for (int j = 0; j < h; j++)
		{
			for (int i = 0; i < w; i++)
			{	
				int k = j*w + i;
				if (faceIdx[k] == ind) {
			
					gray1.at<uchar>(j, i) = 255;
					mask.at<uchar>(j+1, i+1) = 0;

				}
				else
				{
					gray1.at<uchar>(j, i) = 0;
					mask.at<uchar>(j+1, i+1) = 255;
				}

				//				c = gray1.at<Vec3b>(j, i);

			}
		}
//		cv::imwrite("d.png", gray1);
		//Mat mask = gray1.clone();
		
		Mat dst1,dst2;
		Mat ele=getStructuringElement(cv::MORPH_DILATE, Size(10, 10));
		Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(10, 10));
//		cvtColor(gray1, src_gray, CV_BGR2GRAY);
	
		threshold(gray1, dst1, 127, 255 , cv::THRESH_BINARY);

	//	cv::imshow("canny_output", dst1);
	//	waitKey(-1);
		cv::erode(dst1, dst2, ele2);

	cv:dilate(dst2, dst1, ele);
	
/// Detect edges using canny
		Canny(dst1, canny_output, thresh, thresh * 2, 3);
	//	cv::imshow("canny_output", canny_output);
		/// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

		//vector<Point> contour = contours[0];
		//npoint = contour.size();

		RNG rng(12345);

		mytriData.totalVertex = 0;
		mytriData.totalFace = 0;


		/*cv::imshow("gray", gray1);
		cv::imshow("mask", mask);

		cvWaitKey(-1);
*/

		for (int i = 0; i<contours.size(); i++) {
			Scalar color = Scalar(100+ds.ndface);// , rng.uniform(0, 255), 255);

			if (contours[i].size() > 10) {

				cout << contours[i].size() << endl;
				std::vector<cv::Point>       sellist;
				//int             selp;


				genpoint(contours[i], sellist);

				//Point p = contour[i];
				//cv::drawContours(mask, back, 0, color, CV_FILLED);// 2, 8, hierarchy);var.y

				Point pp = sellist[0];

				bool bfind = false;
				for each (Point var in sellist)
				{
					uchar cc = gray1.at<uchar>(var.y, var.x);
					if (cc == 255)
					{
						pp = var;
						bfind = true;
						break;

					}
				}
				if (!bfind)//the same contour, already fill.
				{


				/*		vector<vector<Point>> back;
						back.push_back(sellist);

						cv::drawContours(gray1, back, 0, color);

						cv::imshow("gray", gray1);
				
						cvWaitKey(-1);
*/

					continue;
				}
				cv::floodFill(gray1, mask, pp, color, 0, Scalar(1), Scalar(1), 8);

				vector<Point2f> list2;
				vector<Point3f> list3;
				
		        for(int xi =0 ; xi < w; xi++)
					for ( int xj = 0 ; xj < h ; xj++)
					{
						uchar xc = gray1.at<uchar>(xj, xi);
						if (xc == (100 + ds.ndface))
						{
							//list2.push_back(Point2f(xi, xj));
							int ind = xj*w + xi;
							if (abs(depth3d[ind].z - 300) > 0.1) {
								list3.push_back(depth3d[ind]);

								gray2.at<cv::Vec3b>(xj, xi) = cv::Vec3b(0, 128, ds.ndface);
							}
						}

					}

				//Generate3DPoints(list2, list3, 0,&gray2);


				float err=plane_from_points(list3 ,pcolor );
				

		//		continue;

			
				//pointlist[i].x = p.x;

				//pointlist[i].y = p.y;

			//	cv::imshow("gray", gray1);
			
			//	cvWaitKey(-1);
			}
		//	fprintf(fp, "%d %d\n", pointlist[i].x, pointlist[i].y);
		}

		//當所有面都找到之後,
		//相鄰的面有相鄰的edge,封閉的多邊形將形成一個vertex
		//三個edge 可找到一個vertex,但多個edge 如何形成一個vertex

		
	//	cv::imshow("gray", gray2);

	//	cvWaitKey(-1);

	}
	catch (exception em)
	{
		cout << em.what() << endl;
	}
}