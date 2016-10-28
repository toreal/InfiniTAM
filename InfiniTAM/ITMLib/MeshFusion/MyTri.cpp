
#include "MyTri.h"

using namespace ITMLib::Objects;

void MyTri::buildHalfEdge()
{

	//construct half edge
	int nface = totalFace / 3;
	int  nedge = 0;
	for (int i = 0; i < nface; i++)
	{
		int n0 = meshTri[3 * i];
		int n1 = meshTri[3 * i + 1];
		int n2 = meshTri[3 * i + 2];
		meshEdge[nedge].v1 = n0;
		meshEdge[nedge].v2 = n1;
		meshEdge[nedge].belongface = i;
		meshEdge[nedge].pair = NULL;

		nedge++;

		meshEdge[nedge].v1 = n1;
		meshEdge[nedge].v2 = n2;
		meshEdge[nedge].belongface = i;
		meshEdge[nedge].pair = NULL;
		nedge++;

		meshEdge[nedge].v1 = n2;
		meshEdge[nedge].v2 = n0;
		meshEdge[nedge].belongface = i;
		meshEdge[nedge].pair = NULL;
		nedge++;

	}
	totaledge = nedge;

	for (int i = 0; i < nedge; i++)
	{
		//meshEdge[i].needflip = -1;
		for (int j = 0; j < nedge; j++)
		{
			if(i!=j)
			meshEdge[i].checkPair(meshEdge + j);

		}
	}

	cout << "print out boundary points" << endl;
	for (int i = 0; i < nedge; i++)
	{
		if (meshEdge[i].pair == NULL)
		{
			cout << meshEdge[i].v1 << "," << meshEdge[i].v2 << endl;

		}

	}




}


void MyTri::copyFrom(MyTri * data)
{
	memcpy(this, data, sizeof(MyTri));
}

int MyTri::opposite(halfEdge e)
{
	int v[3];
	for (int i = 0; i < 3; i++)
	{
		v[i] = meshTri[3 * e.belongface + i];
	}
	for (int i = 0; i < 3; i++)
	{
		if (v[i] == e.v1)
		{
			if (v[(i + 1) % 3] == e.v2)
			{
				return v[(i + 2) % 3];
			}
			else if (v[(i + 2) % 3] == e.v2)
			{
				return v[(i + 1) % 3];
			}

		}
		else if (v[i] == e.v2)
		{
			if (v[(i + 1) % 3] == e.v1)
			{
				return v[(i + 2) % 3];
			}
			else if (v[(i + 2) % 3] == e.v1)
			{
				return v[(i + 1) % 3];
			}
		}

	}

	return -1;
}

void MyTri::output2d(char * fn)
{

	fstream fout;
	fout.open(fn, ios::out);
	int nface = totalFace / 3;
	for (int i = 0; i < nface; i++)
	{
		Point2 p0 = meshProj[meshTri[3 * i]];
		Point2 p1 = meshProj[meshTri[3 * i + 1]];
		Point2 p2 = meshProj[meshTri[3 * i + 2]];

		fout << p0 << "  moveto" << endl;
		fout << p1 << "  lineto" << endl;
		fout << p1 << "  moveto" << endl;
		fout << p2 << "  lineto" << endl;
		fout << p2 << "  moveto" << endl;
		fout << p0 << "  lineto" << endl;
	}
	fout.close();

}

void MyTri::project(Matrix4f * m, Vector4f intrinRGB)
{
	for (int i = 0; i < totalVertex; i++)
	{

		Vector3f vpos(meshVertex[i].x, meshVertex[i].y, meshVertex[i].z);
		Vector3f npos;
		if (m != NULL)
			npos = (*m)*vpos;
		else
			npos = vpos;


		float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
		float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;
		//meshVertex.push_back(cv::Point3f(pos.x(),pos.y(),pos.z()));
		meshProj[i] = Point2(ix, iy);
		meshDepth[i] = npos.z;
	}//end of for 


}
