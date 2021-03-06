
#include "MeshFusion.h"

#include <vector>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>

// Some convenient typedefs.
//typedef CGAL::Simple_cartesian<float> Kernel;

typedef K::FT      Scalar;
//typedef K::Point_2 Point2;

typedef std::vector<Scalar> Scalar_vector;

typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<K> Triangle_coordinates;

void MeshFusion::intoMesh(Matrix4f invm, MyTri *scanned)
{

	int nnode[2048];

	int ocase = -1;
	int	nface = scanned->totalFace / 3;
	for (int i = 0; i < nface; i++)
	{
		int n0 = scanned->meshTri[3 * i];
		int n1 = scanned->meshTri[3 * i + 1];
		int n2 = scanned->meshTri[3 * i + 2];
		Point2  first_vertex = scanned->meshProj[n0];
		Point2  second_vertex = scanned->meshProj[n1];
		Point2  third_vertex = scanned->meshProj[n2];

		bool s0 = scanned->stat[n0];
		bool s1 = scanned->stat[n1];
		bool s2 = scanned->stat[n2];

		if (!s0 && !s1 && !s2)
		{
			Vector3f pos0(scanned->meshVertex[n0].x, scanned->meshVertex[n0].y, scanned->meshVertex[n0].z);
			Vector3f pos1(scanned->meshVertex[n1].x, scanned->meshVertex[n1].y, scanned->meshVertex[n1].z);
			Vector3f pos2(scanned->meshVertex[n2].x, scanned->meshVertex[n2].y, scanned->meshVertex[n2].z);

			Vector3f ipos0 = invm *pos0;
			Vector3f ipos1 = invm *pos1;
			Vector3f ipos2 = invm *pos2;

			// if (nnode[n0] <= 0)
			{
				mytriData.meshVertex[mytriData.totalVertex].x = ipos0.x;
				mytriData.meshVertex[mytriData.totalVertex].y = ipos0.y;
				mytriData.meshVertex[mytriData.totalVertex].z = ipos0.z;
				nnode[n0] = mytriData.totalVertex;
				mytriData.totalVertex++;
			}

			//if (nnode[n1] <= 0)
			{
				mytriData.meshVertex[mytriData.totalVertex].x = ipos1.x;
				mytriData.meshVertex[mytriData.totalVertex].y = ipos1.y;
				mytriData.meshVertex[mytriData.totalVertex].z = ipos1.z;
				nnode[n1] = mytriData.totalVertex;
				mytriData.totalVertex++;
			}

			//if (nnode[n2] <= 0)
			{
				mytriData.meshVertex[mytriData.totalVertex].x = ipos2.x;
				mytriData.meshVertex[mytriData.totalVertex].y = ipos2.y;
				mytriData.meshVertex[mytriData.totalVertex].z = ipos2.z;
				nnode[n2] = mytriData.totalVertex;
				mytriData.totalVertex++;
			}

			mytriData.meshTri[mytriData.totalFace++] = nnode[n0];
			mytriData.meshTri[mytriData.totalFace++] = nnode[n1];
			mytriData.meshTri[mytriData.totalFace++] = nnode[n2];

		}


	}

}

/// pose: current transform
/// scanned : scanned model
///
void MeshFusion::meshMerge(ITMMesh * mesh, ITMPose * pose, MyTri * scanned)
{

	Vector4f intrinRGB = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

	Matrix4f m = pose->GetM();
	Matrix4f invm = pose->GetInvM();

	//meshVertex.clear();

	scanned->project(&m, intrinRGB);
	currTri.project(NULL, intrinRGB);


	float* dp = proDepth->GetData(MEMORYDEVICE_CPU);
	int w = proDepth->noDims.x;

	//currTri.stat 判斷目前mesh 上的vertex 是否在scanned mesh 上,如果是的話state 為true
	memset(currTri.stat, 0x00, sizeof(bool) * 2048);
		for (int j = 0; j < currTri.totalVertex; j++)
		{
			float dz= currTri.meshDepth[j];
			float  ed = scanned->locateAt(currTri.meshProj[j]);
			if ( ed >=0  && fabs(dz-ed) <5)
			currTri.stat[j] = true;
			
		};//end of for loop to check vertex



	//project merged boundary as constrain from scanned model into current model
	// 

	int nstart = 0;
	int nend = scanned->nboundary;
	
	if (scanned->ncontour > 1)
	{
		int maxnv = 0;
		int currnv;
		for (int i = 0; i < scanned->ncontour; i++)
		{
			currnv = scanned->contour[i + 1] - scanned->contour[i];
			if (currnv > maxnv)
			{
				maxnv = currnv;
				nstart = scanned->contour[i];
				nend = scanned->contour[i + 1]-1; // i+1 是下一個countour 的起點, 該點-1 是這個countour 的終點,但也是起點(繞一圈). -2 正好才是終點
			}

		}
	}

//scanned model的vertex 保持不變,
//1.要新增新model 的點
//2.舊點的位置,要根據新的depth作調整

	
	int begi = -1;
	int lasti = -1;
	for (int i = nstart; i < nend; i++)
	{
		int bnode = scanned->boundary[i];// vertex index
		float dz = scanned->meshDepth[bnode];
		float  ed=	currTri.locateAt(scanned->meshProj[bnode]);
		if (ed >= 0 && fabs(ed-dz) < 5)
		{

			if (lasti != -1 && lasti == (i - 1))
			{
				constrainbeg.push_back(scanned->meshProj[scanned->boundary[lasti]]);
				constrainend.push_back(scanned->meshProj[bnode]);
			}

			vInputPoints.push_back(scanned->meshProj[bnode]);
			lasti = i;
			if (begi < 0)
				begi = i;

		}
	}

	if (lasti == (nend - 1) && begi == nstart)
	{
		constrainbeg.push_back(scanned->meshProj[scanned->boundary[lasti]]);
		constrainend.push_back(scanned->meshProj[scanned->boundary[begi]]);

	}

	//新scanned 的data,建新的mesh
	constructMesh(NULL, &currTri);
	
	//將新mesh 加到原有的mesh data 上
	intoMesh(invm, &currTri);

	

	//build new triangle
	// for each new corner check whether it is belong to a triangle or not 

	//for (int i = 0; i < _corners.size(); i++)
	//{
	//	cv::Point2f p = _corners[i];

	//	for (int j = 0; j < meshold->noTotalTriangles; j++)
	//	{
	//		 
	//	}

	//}





}
