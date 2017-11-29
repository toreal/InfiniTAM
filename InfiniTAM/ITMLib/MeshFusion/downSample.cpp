

#include "MeshFusion.h"


void MeshFusion::downSample(ITMFloat4Image * depth, ITMFloat4Image *& subNormal)
{
	const int downsize = 4;

	int sw = depth->noDims.width;
	int sh = depth->noDims.height;
	int w = sw / downsize;
	int h = sh / downsize;


	if (subNormal == NULL)
	{
		
		subNormal = new ITMFloat4Image( ORUtils::Vector2<int>(w, h), true, false);

	}

	Vector4f * dd = depth->GetData(MEMORYDEVICE_CPU);
	Vector4f* downd = subNormal->GetData(MEMORYDEVICE_CPU);

	int shift[downsize*downsize];

	int k = 0; 
	for ( int j = 0 ; j < downsize; j++)
		for (int i = 0; i < downsize; i++)
		{
			shift[k++] = j * sw + i;

	}

	int lens = downsize*downsize;

	double x, y, z;

	for ( int j = 0 ; j < h; j++)
		for (int i = 0; i < w; i++)
		{
			int sx = i * downsize;
			int sy = j * downsize;

			x = y = z = 0;
			Vector4f pixel_out, pixels_in;
			for (int k = 0; k < lens; k++)
			{
				pixels_in = dd[sy*sw + sx + shift[k]];
				x += pixels_in.x;
				y += pixels_in.y;
				z += pixels_in.z;
				
			}

			pixel_out.x = x / downsize;
			pixel_out.y = y / downsize;
			pixel_out.z = z / downsize;

			double norm = sqrt(pixel_out.x*pixel_out.x + pixel_out.y*pixel_out.y + pixel_out.z*pixel_out.z);

			if (norm > 0){
				downd[j*w + i] = pixel_out / norm;
				downd[j*w + i].w = 1;

			}
			else{
				downd[j*w + i] = pixel_out;

				downd[j*w + i].w = -1;
			}


	}
	





}