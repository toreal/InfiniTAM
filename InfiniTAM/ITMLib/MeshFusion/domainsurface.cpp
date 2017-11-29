
#include "domainsurface.h"


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <algorithm>
#include <vector>



using namespace cv;
using namespace std;



using namespace ITMLib::Objects;


std::pair<int, int> my_make_pair(int a, int b)
{
	if (a < b) return std::pair<int, int>(a, b);
	else return std::pair<int, int>(b, a);
}



DomainSurface::DomainSurface()
{
}

DomainSurface::~DomainSurface()
{
}


vector<pair<int, int>> pairlist;

bool contains(int a, int b)
{
	pair<int, int>p = my_make_pair(a, b);

	return  (find(pairlist.begin(), pairlist.end(), p) != pairlist.end());

}

void addpair(int a, int b)
{
	pair<int,int>p=	my_make_pair(a, b);

	if (find(pairlist.begin(), pairlist.end(), p) == pairlist.end())
	{
		pairlist.push_back(p);
	}

}




int DomainSurface::findLine( int a, int b)
{
	int ret = -1;

	Point3f p1 = normaldbuf[a];
	Point3f p2 = normaldbuf[b];
	float d1 = dbuf[a];
	float d2 = dbuf[b];
	//Point3f r_point, r_normal;


	
	// logically the 3rd plane, but we only use the normal component.
	 Point3f p3_normal = p1.cross(p2);
	 float det =sqrt( p3_normal.dot(p3_normal));

	// If the determinant is 0, that means parallel planes, no intersection.
	// note: you may want to check against an epsilon value here.
	if (det != 0.0) {

		p3_normal = (1.0 / det)*p3_normal;

		float Denominator = p1.x * p2.y - p1.y * p2.x;
		rpoint[rline] = Vec3f((d2 * p1.y - d1 * p2.y) / Denominator, (d1 * p2.x - d2 * p1.x) / Denominator, 0.0f);


		// calculate the final (point, normal)
	//Point3f testp = ((p3_normal.cross(p2) * d1) +
	//		(p1.cross(p3_normal) * d2));


	//float testd1 = rpoint[rline].dot(p1) + d1;
	//float testd2 = rpoint[rline].dot(p2) + d2;

	//float testd3 = testp.dot(p1) + d1;
	//float testd4 = testp.dot(p2) + d2;



		rnormal[rline] = p3_normal;


		Point3f v1 =  rpoint[rline]- centerbuf[a] ;
		t1[rline] =- v1.dot(rnormal[rline]);

		//Point3f testi1 = rpoint[rline] + t1[rline] * rnormal[rline];

		//Point3f delta = testi1 - centerbuf[a];
		//float distance = sqrt(delta.dot(delta));

		Point3f v2 =  rpoint[rline]- centerbuf[b] ;
		t2[rline] =- v2.dot(rnormal[rline]);

		c1[rline] = a;
		c2[rline] = b;
		ret = rline;
		rline++;
		return ret;
	}
	else {
		return ret;
	}
}

void DomainSurface::drawline(Mat & buf, int i, Vector4f  intrinRGB)
{
	Scalar c = Scalar(100, 100, 0);

	float ct = (t1[i] + t2[i]) / 2;


	Point3f p1 = rpoint[i] + rnormal[i] * (ct - 10);

	Point3f p2 = rpoint[i] + rnormal[i] * (ct + 10);

	float sx = p1.x * intrinRGB.x / p1.z + intrinRGB.z;
	float sy = p1.y * intrinRGB.y / p1.z + intrinRGB.w;

	float ex = p2.x * intrinRGB.x / p2.z + intrinRGB.z;
	float ey = p2.y * intrinRGB.y / p2.z + intrinRGB.w;

	line(buf, Point(sx, sy), Point(ex, ey), c, 1);



}


void DomainSurface::makeTri(  Point3f *depth3d , Mat& nbuf,
	Vector4f  intrinRGB)
{
	float T = 0;
	int  near[1000];
	float nearval[1000];
	int ind = 0;

	rline = 0;


	//imshow("label", nbuf);
	//cvWaitKey(-1);

	pairlist.clear();

	vector<float>  list;
//	float *normalData_out = depth->GetData(MEMORYDEVICE_CPU);

	int h = nbuf.rows;
	int w = nbuf.cols;

	Mat buf = Mat(h, w, CV_8SC3);

	buf.setTo(Scalar(-127, -127, -127));


	int ks = 1;


	float vmin = 10000;
	float vmax = -1;
	//find the neighborhood relationship
	for ( int i = 0 ; i < w; i++)
		for (int j = 0; j < h; j++)
		{
			int kc = -1;


			for (int kj = -ks; kj <= ks; kj++)
				for (int ki = -ks; ki <= ks; ki++)
				{
					if (i + ki < 0 || i + ki >= w)
						continue;

					if (j + kj < 0 || j + kj >= h)
						continue;


					int ikx = (kj + j)*w + (i + ki);
					Vec3b c= nbuf.at<Vec3b>(kj + j, i + ki);

					if (kc <= 0)
						kc = c.val[2];

					if (c.val[2] != kc)
					{
						addpair(kc, c.val[2]);
					}

				}


			int idx = ( j)*w + (i  );
			Point3f val = depth3d[idx] ;
			Vec3b belong = nbuf.at<Vec3b>( j, i );
			int cd = belong.val[2];

		 float error =	abs(val.dot(normaldbuf[cd]) + dbuf[cd]);

			//check the error between the belong surface and closest surface

		 int rval, bval, gval;
		 rval = bval = gval = -127;

		 if (error < vmin)
			 vmin = error;
		 if (error > vmax  && error < 100)
		 {
			 vmax = error;
		 }


		 if ( error < 3 )
			 rval =( ((int)(255 * error / 3)) % 255)-127;
		 else if (error < 10)
		 {
			 rval = 127;
			 bval = (((int)(255 * (error-3) / 10)) % 255)-127;

		 }
		 else if ( error < 50)
		 {
			 rval = 127;
			 bval = 127;
			 gval= (((int)(255 * (error-10) / 50)) % 255)-127 ;

		 }


		//	int bval = ((int)(255*error/20  ))%255;

		//291,151

		// if (i == 291 && j == 151)
		//	 cout << "here" << endl;

			buf.at<Vec3b>(j, i) = Vec3b(rval, bval, gval);


		}



	cout << "min error: " << vmin << endl;

	cout << "max error: " << vmax << endl;


	for (int i = 0; i < ndface; i++)
	{

		for each (pair<int,int> var in pairlist)
		{
			int nline = -1;
			if (var.first == i)
			nline=	findLine(i, var.second);
			else if (var.second == i)
			nline=	findLine(i, var.first);

			drawline(buf, nline, intrinRGB);

		}


	//	cv::imshow("", buf);
	//	cvWaitKey(0);

	}




	
	for (int i = 0; i < ndface; i++)
	{
		list.clear();
		for (int j = 0; j < ndface; j++)
		{
			if (i != j)
			{
				Point3f diffv = centerbuf[i] - centerbuf[j];
				float dis = sqrt(diffv.dot(diffv));
				list.push_back(dis);

			}
		}

		sort(list.begin(), list.end());

		if (list[2] > T)
			T = list[2];
	}





	for (int i = 0; i < ndface; i++)
	{
		ind = 0;
		for (int j = 0; j < ndface; j++)
		{
			if (i != j)
			{
				Point3f diffv = centerbuf[i] - centerbuf[j];
				float dis=   sqrt(diffv.dot(diffv));
				if (dis <= T && contains(i,j))
				{
					findLine(i, j);

					near[ind] = j;
					nearval[ind] = dis;
					ind++;
				
				}

			}
			
		}

		Point3f cpos = centerbuf[i];
	
		float sx = cpos.x * intrinRGB.x / cpos.z + intrinRGB.z;
		float sy = cpos.y * intrinRGB.y / cpos.z + intrinRGB.w;

		int b = rand() % 255;
		int g = rand() % 255;

		int r = rand() % 255;


		Scalar c = Scalar(b,g,r);

		for (int k = 0; k < ind; k++)
		{

			if (contains(  i,    near[k] ))
			{

				Point3f npos = centerbuf[near[k]];
				float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
				float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;

				findLine(i, near[k]);

			//	line(buf, Point(sx, sy), Point(ix, iy), c, 1);
			}

		}

	}

	Scalar c = Scalar(100,-127,100);
	Scalar nc = Scalar(100, 127, -127);

	//cv::QtFont qt = cv::fontQt("arial");

	for (int i = 0; i < rline; i++)
	{
		float ct = (t1[i] + t2[i]) / 2;
		

		Point3f p1 = rpoint[i] + rnormal[i] *(ct-10);

		Point3f p2 = rpoint[i] + rnormal[i] * (ct+10);

		float sx = p1.x * intrinRGB.x / p1.z + intrinRGB.z;
		float sy = p1.y * intrinRGB.y / p1.z + intrinRGB.w;

		float ex = p2.x * intrinRGB.x / p2.z + intrinRGB.z;
		float ey = p2.y * intrinRGB.y / p2.z + intrinRGB.w;

		line(buf, Point(sx, sy), Point(ex, ey), c, 1);

		float c1x = centerbuf[c1[i]].x * intrinRGB.x / centerbuf[c1[i]].z + intrinRGB.z;
		float c1y = centerbuf[c1[i]].y * intrinRGB.y / centerbuf[c1[i]].z + intrinRGB.w;

		float c2x = centerbuf[c2[i]].x * intrinRGB.x / centerbuf[c2[i]].z + intrinRGB.z;
		float c2y = centerbuf[c2[i]].y * intrinRGB.y / centerbuf[c2[i]].z + intrinRGB.w;


		cv::circle(buf, Point(c1x, c1y), 5, c, 1);
		cv::circle(buf, Point(c2x, c2y), 5, c, 1);
		
		
		Point3f cc = centerbuf[c1[i]];
		cc = cc + 30*normaldbuf[c1[i]];

		float n1x = cc.x * intrinRGB.x / cc.z + intrinRGB.z;
		float n1y = cc.y * intrinRGB.y / cc.z + intrinRGB.w;

		line(buf, Point(c1x, c1y), Point(n1x, n1y), nc, 1);


		cc = centerbuf[c2[i]];
		cc = cc + 30*normaldbuf[c2[i]];

		 n1x = cc.x * intrinRGB.x / cc.z + intrinRGB.z;
		 n1y = cc.y * intrinRGB.y / cc.z + intrinRGB.w;

		line(buf, Point(c2x, c2y), Point(n1x, n1y), nc, 1);




		//291,151



		//char txtbuf[1024];

		//sprintf(txtbuf, "%d", c1[i]);

	//	cv::addText(buf, txtbuf, Point(c1x, c1y), qt);


		



	}





/*
	for each (pair<int,int> var in pairlist)
	{
		if (var.second != 205)
		{
			Point3f cpos = centerbuf[var.first];

			float sx = cpos.x * intrinRGB.x / cpos.z + intrinRGB.z;
			float sy = cpos.y * intrinRGB.y / cpos.z + intrinRGB.w;

			Point3f npos = centerbuf[var.second];
			float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
			float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;


			line(buf, Point(sx, sy), Point(ix, iy), Scalar(0,255,0), 2);


		}

	}
*/

	imwrite("segmentds.png", buf);
	//imwrite("seg.png", buf);

	cv::imshow("seg",buf );
	cvWaitKey(1);



}

 
void DomainSurface::mesh(MyTri& mytriData)
{
/*
	if (false)
	{
		int nstart = mytriData.totalVertex;

		for each (Point p2d in sellist)
		{
			ind = p2d.y*w + p2d.x;
			if (abs(depth3d[ind].z - 300) > 0.1) {

				mytriData.meshVertex[mytriData.totalVertex].x = depth3d[ind].x;
				mytriData.meshVertex[mytriData.totalVertex].y = depth3d[ind].y;
				mytriData.meshVertex[mytriData.totalVertex].z = depth3d[ind].z;
				mytriData.totalVertex++;
			}

		}
		int nend = mytriData.totalVertex - 1;
		mytriData.meshVertex[mytriData.totalVertex].x = centerbuf[ndface - 1].x;
		mytriData.meshVertex[mytriData.totalVertex].y = centerbuf[ndface - 1].y;
		mytriData.meshVertex[mytriData.totalVertex].z = centerbuf[ndface - 1].z;
		mytriData.totalVertex++;

		for (int k = nstart; k < nend; k++)
		{
			mytriData.meshTri[mytriData.totalFace++] = nend + 1;
			mytriData.meshTri[mytriData.totalFace++] = k;
			mytriData.meshTri[mytriData.totalFace++] = k + 1;
		}

		mytriData.meshTri[mytriData.totalFace++] = nend + 1;
		mytriData.meshTri[mytriData.totalFace++] = nend;
		mytriData.meshTri[mytriData.totalFace++] = nstart;
	}
*/

}