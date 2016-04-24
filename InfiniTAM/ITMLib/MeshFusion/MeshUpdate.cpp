
#include "MeshFusion.h"
#include "../Objects/ITMPose.h"

using namespace ITMLib::Objects;

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
// ----------------------------------------------------------------------------
typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;



void MeshFusion::meshUpdate(ITMMesh * meshold)
{
	ITMPose pose;

	MyMesh mesh;

	// generate vertices

	MyMesh::VertexHandle vhandle[8];

	vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));


	//mesh transform

	//projection 
	
	Matrix4f  m= pose.GetM();
	ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);
	
	for (int i = 0; i < meshold->noTotalTriangles; i++)
	{
		Vector4f p0= trivec[i].p0;
		p0.w = 1;
		Vector4f pp0 = m*p0;
		Vector4f p1 = trivec[i].p1;
		p1.w = 1;
		Vector4f pp1 = m*p1;


	}

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