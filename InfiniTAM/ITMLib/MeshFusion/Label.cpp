
#include "MeshFusion.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/container/slist.hpp>

//建立facebuf. 一個新的region 有一face buffer.
//每個facebuf 有其region, boundary, 
struct facebuf
{


};


using namespace cv;

Vec3f nbuf[100];
Vec3f nowbuf[100];
Vec3b cbuf[100];
int ndx = 0;
int countIdx[100];

bool bfirst = true;

void MeshFusion::label(ITMView **view_ptr, Mat & buf3, int currentFrameNo)
{

	
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

Mat buf2 = Mat(h, w, CV_8SC3);

//Mat buf3 = Mat(h, w, CV_8SC3);


int ks = 7;
int lens = (ks + 1)*(ks + 1);

Vec3f c;

for ( int j = 0 ; j < h; j++)
for (int i = 0; i < w; i++)
{
	float sx = 0.0f;
	float sy = 0.0f;
	float sz = 0.0f;
	for (int kj = -ks; kj <= ks; kj++)
	for (int ki = -ks; ki <= ks ;ki++)
	{
		if (i + ki < 0 || i + ki >= w)
			continue;

		if (j + kj < 0 || j + kj >= h)
			continue;


		int idx = (kj+j)*w + (i+ki);
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

	c.val[0] = sx/norm;// (sx + 1) * 127;
	c.val[1] = sy/norm;// (sy + 1) * 127;
	c.val[2] = sz/norm;// (sz + 1) * 127;

	float val = 999;
	int kdx = 0;
	for (int k = 0; k < ndx; k++)
	{

		float v =abs(acos( c.dot(nbuf[k])));
		if (v < val)
		{
			val = v;
			kdx = k;

		}

	}
	val = 254 * 9 * val / 3.14159;

	buf.at<Vec3b>( j,i) = cbuf[kdx] ;
	//buf2.at<Vec3b>(j, i) = Vec3b(val, val,val);
	faceIdx[j*w + i] = kdx;
	countIdx[kdx]++;
	nowbuf[kdx] = c;
}

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

build3DPoint();
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


