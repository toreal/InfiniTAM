
#include "MeshFusion.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/container/slist.hpp>

//建立facebuf. 一個新的region 有一face buffer.
//每個facebuf 有其region, boundary, 


using namespace cv;

Vec3f nbuf[100];
//Vec3f nowbuf[100];
Vec3b cbuf[100];
int ndx = 0;
int countIdx[100];

bool bfirst = true;

void MeshFusion::buildnormal()
{

	int h = (mainView)->depthNormal->noDims.height;
	int w = (mainView)->depthNormal->noDims.width;

	int ks = 1;
	int lens = (ks + 1)*(ks + 1);

	

	for (int j = 0; j < h; j++)
		for (int i = 0; i < w; i++)		
		{
			int idx = ( j)*w + (i );
			if (abs(depth3d[idx].z - 300) < 0.1)
				continue;

			std::vector<Point3f> list;
			for (int kj = -ks; kj <= ks; kj++)
				for (int ki = -ks; ki <= ks; ki++)
				{
					if (i + ki < 0 || i + ki >= w)
						continue;

					if (j + kj < 0 || j + kj >= h)
						continue;


					 idx = (kj + j)*w + (i + ki);
					 if (abs(depth3d[idx].z - 300) > 0.1) {
						 list.push_back(depth3d[idx]);
					 }

				}
					float err = plane_from_points(list, -1);
					ds.ndface--;

					normal3d[idx] = ds.normaldbuf[ds.ndface];


		}

			

}

Point3f barycentric(Point3f p1, Point3f p2, Point3f p3, Point3f t)
{
Point3f v2_1, v2_3, v2_t;

Point3f bary;
v2_1 =p1- p2;
v2_3 = p3 - p2;
v2_t = t - p2;
	
float d00 = v2_1.dot( v2_1);
float d01 = v2_1.dot( v2_3);
float d11 = v2_3.dot( v2_3);
float denom = d00 * d11 - d01 * d01;

float d20 = v2_t.dot( v2_1);
float d21 = v2_t.dot( v2_3);
bary.x = (d11 * d20 - d01 * d21) / denom;
bary.y = (d00 * d21 - d01 * d20) / denom;
bary.z = 1.0f - bary.x - bary.y;

return bary;
}




void MeshFusion::label(ITMView **view_ptr, Mat & buf3, int currentFrameNo)
{


	build3DPoint();
	downSample((*view_ptr)->depthNormal);
	//buildnormal();
	
	if (bfirst)
	{
		//prepare data
		srand(time(NULL));
		FILE *fp = fopen("files\\out.txt", "r");
		float nx, ny, nz;


		int bstat = 1;
		

		while (bstat >= 0)
		{
			bstat = fscanf(fp, "%f,%f,%f\n", &nx, &ny, &nz);
			nbuf[ndx].val[0] = nx;
			nbuf[ndx].val[1] = ny;
			nbuf[ndx].val[2] = nz;

			cbuf[ndx].val[0] = rand() % 128;
			cbuf[ndx].val[1] = ny >= 0 ? 100 : 0;
			cbuf[ndx].val[2] =ndx;


			ndx++;
		}

		fclose(fp);

		bfirst = false;
	}
	//


	memset(countIdx, 0x00, 100*sizeof(int));
	Vector4f *normalData_out = (*view_ptr)->depthNormal
		->GetData(MEMORYDEVICE_CPU);

int h=	(*view_ptr)->depthNormal->noDims.height;
int w = (*view_ptr)->depthNormal->noDims.width;

Mat buf = Mat(h, w, CV_8SC3);

Mat buf2 = Mat(h, w, CV_8UC3);

//Mat buf3 = Mat(h, w, CV_8SC3);


int ks = 3;
int lks = 1;
int lens = (ks + 1)*(ks + 1);

Point3f c,pc;

for ( int j = 0 ; j < h; j++)
	for (int i = 0; i < w; i++)
	{

		float val1 = 999;
		int kdx1 = 0;
		float val2 = 999;
		int kdx2 = 0;
		float val3 = 999;
		int kdx3 = 0;


		Point3f centroid = depth3d[j*w+i];
		vector<Point3f> points;

		//if (i == 0)
		//	continue;

		//c = normal3d[j*w + i]; 
		//pc = normal3d[j*w + i-1];

		//Vector4f oldc=normalData_out[j*w + i];
		//c = Point3f(oldc.x, oldc.y, oldc.z);


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

				points.push_back(depth3d[idx]);

				Vector4f nomald = normalData_out[idx];

				sx += nomald.x;

				sy += nomald.y;

				sz += nomald.z;



			}



		sx = sx / lens;

		sy = sy / lens;

		sz = sz / lens;



		float norm = sqrt(sx*sx + sy*sy + sz*sz);



		if (norm <= 0)

			continue;



		c.x = sx / norm;// (sx + 1) * 127;

		c.y = sy / norm;// (sy + 1) * 127;



		c.z = sz / norm;// (sz + 1) * 127;



		for (int k = 0; k < ndx; k++)
		{
			Point3f dir = nbuf[k];

			float d = centroid.x*dir.x + centroid.y*dir.y + centroid.z*dir.z;

			float sumerr = 0;

			int n = 0;

			for each (Point3f p in points)
			{
				float err = p.x*dir.x + p.y*dir.y + p.z*dir.z - d;
				sumerr += err*err;
				n++;
			}

			sumerr = sumerr / n;
			



			float v = sumerr;// abs(acos(c.dot(nbuf[k])));
			if (v < val1 && v < val2 && v < val3)
			{
				if (val1 < val2 && val1 < val3)
				{
					val2 = val1;
					kdx2 = kdx1;

				}
				else if (val1 < val3)
				{
					val3 = val1;
					kdx3 = kdx1;

				}

				val1 = v;
				kdx1 = k;

			}
			else if (v < val2 && v < val3)
			{
				if (val2 < val3)
				{
					val3 = val2;
					kdx3 = kdx2;
				}

				val2 = v;
				kdx2 = k;
			}
			else if (v < val3)
			{
				val3 = v;
				kdx3 = k;
			}

		}
		//float val = 254 * 9 * val1 / 3.14159;
		int kdx = kdx1*10 %255;
		buf3.at<Vec3b>(j, i) = Vec3b(kdx1, kdx2,kdx3);

		buf.at<Vec3b>(j, i) = cbuf[kdx];
		buf2.at<Vec3b>(j, i) = Vec3b(kdx, kdx, kdx);
	//	buf2.at<Vec3b>(j, i) = Vec3b(cr, cg, cb);  //Vec3b((c.x+1)*127 ,(c.y+1)*127,(c.z+1)*127);
												   //buf2.at<Vec3b>(j, i) = Vec3b((c.x+1)*127 ,(c.y+1)*127,(c.z+1)*127);

		faceIdx[j*w + i] = kdx;
		countIdx[kdx]++;


	}


	for (int j = 0; j < h; j++)
		for (int i = 0; i < w; i++)

	{

			int localCount[100];

			memset(localCount, 0x00, 100*sizeof (int));

			for (int kj = -lks; kj <= lks; kj++)

				for (int ki = -lks; ki <= lks; ki++)

				{

					if (i + ki < 0 || i + ki >= w)

						continue;



					if (j + kj < 0 || j + kj >= h)

						continue;





					int idx = (kj + j)*w + (i + ki);

					Vec3b hh = buf3.at<Vec3b>((kj + j), (i + ki));

					localCount[hh.val[0]]++;
					localCount[hh.val[1]]++;
					localCount[hh.val[2]]++;


				}

			int nmax = -1;
			int nmaxv = -1;
			for (int tt = 0; tt < 100; tt++)
			{
				if (localCount[tt] > nmaxv)
				{
					nmaxv = localCount[tt];
					nmax = tt;
				}

			}


			if (j > 5 && nmaxv > 0)
			{
				nmaxv = nmaxv;

			}

	//Point3f ret = barycentric(nbuf[kdx1], nbuf[kdx2], nbuf[kdx3], c);

	//int finx = kdx1;

	//if (kdx2 < kdx1 && kdx2 < kdx3)
	//	finx = kdx2;

	//if (kdx3 < kdx1 && kdx3 < kdx2)
	//	finx = kdx3;


	//if (j > 10 && finx > 0)
	//	finx = finx;


//	float  ifloat = finx;// ret.x *(kdx1) + ret.y* kdx2 + ret.z*kdx3;

	int ii = nmax*10 ;

	uchar cr, cg, cb;
	cr = cg = cb = 0;
	if (ii < 255)
	{
		cr = ii;
	}
	else if (ii < 512)
	{
		cr = 255;
		cg = ii - 256;
	}
	else
	{
		cr = 255;
		cg = 255;
		cb = (ii - 512) % 255;


	}

	//430,230

	
	//float v = abs(acos(c.dot(pc)));

	//uchar ccc = (int)v % 255;
	//v = sqrt(c.dot(c));

	//if (abs(v - 1) > 0.01 && v > 0.001)
	//	c = c / v;

	//buf2.at<Vec3b>(j, i) = Vec3b(cr, cg, cb);  //Vec3b((c.x+1)*127 ,(c.y+1)*127,(c.z+1)*127);
	//buf2.at<Vec3b>(j, i) = Vec3b((c.x+1)*127 ,(c.y+1)*127,(c.z+1)*127);

}

imshow("ttt", buf2);
cvWaitKey(-1);


class  myint
{
public:
	int a;
	int b;

	bool operator<(const myint & b  ) const { return a< b.a; }

};

myint ma,mb;
//ma.a = 100;

//bool mc = ma < mb;

boost::container::slist<myint> mlist;

for (int i = 0; i < 100; i++)
{
	ma.a = countIdx[i];
	ma.b = i;
	if (countIdx[i] > 0)
		mlist.push_front(ma);
}

mlist.sort();
mlist.reverse();

ds.ndface = 0;

boost::container::slist<myint>::iterator  it= mlist.begin();
for (int i = 0; i < mlist.size(); i++)
{
	myint a = it.get()->get_data();
//	std::cout << a.a <<"," << a.b <<  ":" << nbuf[a.b] <<  "==>" << nowbuf[a.b] << std::endl;
	if (a.a > 500)
	patchSurface(a.b,buf3,i);
	
	it++;
}



for ( int j = 0 ; j < h;j++)
	for (int i = 0; i < w; i++)
	{
		Vec3b cc=buf.at<Vec3b>(j, i);
		if (cc.val[0] != 205 && cc.val[1] != 205 && cc.val[2] != 205)
		{
			int ind = i + j*w;
			Point3f p = depth3d[ind];

			if (abs(p.z - 300) > 0.01)
			{
				float minerr = 9999;
				int mink = -1;
				for (int k = 0; k < ds.ndface; k++)
				{
					float err1=abs(ds.normaldbuf[k].x * p.x + ds.normaldbuf[k].y *p.y + ds.normaldbuf[k].z *p.z - ds.dbuf[k]);
					Point3f diff = p - ds.centerbuf[k];
					float err2 = sqrt(diff.dot(diff));
					float err = err1 + err2/30;
					if (err < minerr)
					{
						minerr = err;
						mink = k;
					}
					
				}
				if (mink > 0)
				{
					buf2.at<Vec3b>(j, i) = cbuf[mink];
				}

			}
			else
			{
				buf2.at<Vec3b>(j, i) = Vec3b(128,0,128);
			}

		
		}//end if 

}//end of for loop 

Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

for (int i = 0; i < ds.ndface; i++)
{
	int x = intrinparam.x*ds.centerbuf[i].x / ds.centerbuf[i].z + intrinparam.z;
	int y = intrinparam.y*ds.centerbuf[i].y / ds.centerbuf[i].z + intrinparam.w;
	cv::circle(buf2, Point(x, y), 3, cv::Scalar(128, 128, 128));
}



this->writeMesh("ttt.obj");

char fn[1024];

sprintf(fn, "output\\d%d.png", currentFrameNo);

imwrite(fn, buf2);

cv::imshow("segmentation", buf);
cv::imshow("error", buf2);
cv::imshow("tmp", buf3);

//cvWaitKey(1);

}


