
#include "MeshFusion.h"

//For each edge, try to find out where has the larger error, 
//with edge flipping to minimize the error. 
//1.find edge list 
void MeshFusion::edgeRefine(MyTri * tridata)
{

	tridata->output2d("before.txt");

	//construct half edge
	int nface = currTri.totalFace / 3;
	int  nedge = 0;
	for (int i = 0; i < nface; i++)
	{
		int n0 = tridata->meshTri[3 * i];
		int n1 = tridata->meshTri[3 * i + 1];
		int n2 = tridata->meshTri[3 * i + 2];
		tridata->meshEdge[nedge].v1 = n0;
		tridata->meshEdge[nedge].v2 = n1;
		tridata->meshEdge[nedge].belongface = i;
		tridata->meshEdge[nedge].pair = NULL;

		nedge++;

		tridata->meshEdge[nedge].v1 = n1;
		tridata->meshEdge[nedge].v2 = n2;
		tridata->meshEdge[nedge].belongface = i;
		tridata->meshEdge[nedge].pair = NULL;
		nedge++;

		tridata->meshEdge[nedge].v1 = n2;
		tridata->meshEdge[nedge].v2 = n0;
		tridata->meshEdge[nedge].belongface = i;
		tridata->meshEdge[nedge].pair = NULL;
		nedge++;

	}
	tridata->totaledge = nedge;

	for (int i = 0; i < nedge; i++)
	{
		tridata->meshEdge[i].needflip = -1;
		for (int j = i + 1; j < nedge; j++)
		{
			tridata->meshEdge[i].checkPair(tridata->meshEdge + j);

		}
	}

	float* dd = proDepth->GetData(MEMORYDEVICE_CPU);
	int xlens = proDepth->noDims.x;
	int ylens = proDepth->noDims.y;
	
	for (int i = 0; i < nedge; i++)
	{
		if (tridata->meshEdge[i].needflip < 0)
		{
			int n0 = tridata->meshEdge[i].v1;
			int n1 = tridata->meshEdge[i].v2;
			Point2 p1 = tridata->meshProj[n0];
			Point2 p2 = tridata->meshProj[n1];
			Vector3f pos0(currTri.meshVertex[n0].x, currTri.meshVertex[n0].y, currTri.meshVertex[n0].z);
			Vector3f pos1(currTri.meshVertex[n1].x, currTri.meshVertex[n1].y, currTri.meshVertex[n1].z);
     		int x=	(p1.x() + p2.x() )/ 2;
			int y = (p1.y() + p2.y()) / 2;
			if ((x < 0 || x >= 640) || (y < 0 || y >= 480))
				continue;
			Vector3f c=(pos0 + pos1) / 2;
			float z=this->estivalue(dd,Vector2i(x,y), NULL);
			float del = fabsf(z - c.z);
			if (del > 2)
			{
				if (tridata->meshEdge[i].pair != NULL)
				{
					
					int np1 = tridata->opposite(tridata->meshEdge[i]);
					int np2 = tridata->opposite(*(tridata->meshEdge[i].pair));

					if (np1 > 0 && np2 > 0)
					{
						Point2 vp1 = tridata->meshProj[np1];
						Point2 vp2 = tridata->meshProj[np2];

						int xc = (vp1.x() + vp2.x()) / 2;
						int yc = (vp1.y() + vp2.y()) / 2;

						if ((xc < 0 || xc >= 640) || (yc < 0 || yc >= 480))
							continue;
						float zc = this->estivalue(dd, Vector2i(xc, yc), NULL);
						float delc = fabsf(zc - c.z);
						if (delc > del)
							continue;


						tridata->meshEdge[i].needflip = 1;
						tridata->meshEdge[i].cost = del;

						tridata->meshEdge[i].pair->needflip = 1;
						tridata->meshEdge[i].pair->cost = del;
					}
				}
			}
			else
			{
				tridata->meshEdge[i].needflip = 0;
				if (tridata->meshEdge[i].pair != NULL)
				tridata->meshEdge[i].pair->needflip = 0;

			}


		}
       }//end of for loop 

	for (int i = 0; i < nedge; i++)
	{
		if (tridata->meshEdge[i].needflip > 0 && tridata->meshEdge[i].pair->needflip > 0)
		{  
			int f1 = tridata->meshEdge[i].belongface;

			bool bc = false;
			for (int k = -2; k <= 2; k++)
			{
				if ((k + i) >= 0 && k != 0 && tridata->meshEdge[k + i].belongface == f1 && tridata->meshEdge[k + i].needflip == -2)
				{
					tridata->meshEdge[i].needflip = 0;

					bc = true;
					tridata->meshEdge[i].pair->needflip = 0;
					break;

				}
					
			}
			if (bc)
				continue;


			if (tridata->meshEdge[i + 1].belongface == f1 && tridata->meshEdge[i + 1].needflip > 0)//下一個edge
			{
				if (tridata->meshEdge[i + 1].cost > tridata->meshEdge[i].cost)
				{
					tridata->meshEdge[i].needflip = 0;
					tridata->meshEdge[i].pair->needflip = 0;
					continue;
				}
				else
				{
					tridata->meshEdge[i+1].needflip = 0;
					tridata->meshEdge[i+1].pair->needflip = 0;

				}

			}

			if (tridata->meshEdge[i + 2].belongface == f1 && tridata->meshEdge[i + 2].needflip > 0)//下一個edge
			{
				if (tridata->meshEdge[i + 2].cost > tridata->meshEdge[i].cost)
				{
					tridata->meshEdge[i].needflip = 0;
					tridata->meshEdge[i].pair->needflip = 0;
					continue;
				}
				else
				{
					tridata->meshEdge[i + 2].needflip = 0;
					tridata->meshEdge[i + 2].pair->needflip = 0;

				}

			}


			int f2 = tridata->meshEdge[i].pair->belongface;
			int n1 = tridata->meshEdge[i].v1;
			int n2 = tridata->meshEdge[i].v2;
			int f1n[3];
			int f2n[3];
			int t1, t2;
			for (int k = 0; k < 3; k++)
			{
				f1n[k] = tridata->meshTri[3*f1 + k];
				f2n[k] = tridata->meshTri[3*f2 + k];
				if (f1n[k] != n1 && f1n[k] != n2)
					t1 = f1n[k];
				if (f2n[k] != n1 && f2n[k] != n2)
					t2 = f2n[k];
				
			}
			tridata->meshTri[3*f1] = t1;
			tridata->meshTri[3*f1+1] = t2;
			tridata->meshTri[3 * f1+2] = n1;
			tridata->meshTri[3 * f2] = t2;
			tridata->meshTri[3 * f2 + 1] = t1;
			tridata->meshTri[3 * f2 + 2] = n2;
			tridata->meshEdge[i].needflip = -2;
			tridata->meshEdge[i].v1 = t1;
			tridata->meshEdge[i].v2 = t2;
			tridata->meshEdge[i].pair->needflip = -2;
			tridata->meshEdge[i].pair->v1 = t2;
			tridata->meshEdge[i].pair->v2 = t1;

		}
	}

	tridata->output2d("after.txt");


	}