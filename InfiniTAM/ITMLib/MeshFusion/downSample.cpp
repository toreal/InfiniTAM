

#include "MeshFusion.h"


void MeshFusion::downSample(ITMFloat4Image * depth)
{

	int sw = depth->noDims.width;
	int sh = depth->noDims.height;
	int w = sw / 4;
	int h = sh / 4;


	if (downNormal == NULL)
	{
		
		downNormal = new ITMFloat4Image(new ORUtils::Vector2<int>(w, h), true, false);

	}

	Vector4f * dd = depth->GetData(MEMORYDEVICE_CPU);
	Vector4f* downd = downNormal->GetData(MEMORYDEVICE_CPU);

	int shift[16];

	int k = 0; 
	for ( int j = 0 ; j < 4; j++)
		for (int i = 0; i < 4; i++)
		{
			shift[k++] = j * sw + i;

	}


	for ( int j = 0 ; j < h; j++)
		for (int i = 0; i < w; i++)
		{
			int sx = i * 4;
			int sy = j * 4;

			Vector4f pixel_out, pixels_in;
			for (int k = 0; k < 16; k++)
			{
				pixels_in = dd[sy*sw + sx + shift[k]];
				pixel_out.x += pixels_in.x;
				pixel_out.y += pixels_in.y;
				pixel_out.z += pixels_in.z;
				
			}

			pixel_out.x = pixel_out.x / 16;
			pixel_out.y = pixel_out.y / 16;
			pixel_out.z = pixel_out.z / 16;




	}
	





}