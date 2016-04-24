
#include "MeshFusion.h"
#include "../Objects/ITMPose.h"


<<<<<<< HEAD
#include <vector>

#include <boost/foreach.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;



using namespace ITMLib::Objects;
//
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
//// ----------------------------------------------------------------------------
=======
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
// ----------------------------------------------------------------------------
>>>>>>> origin/master
//typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;



void MeshFusion::meshUpdate(ITMMesh * meshold)
{
	ITMPose pose;

<<<<<<< HEAD
=======
#ifdef OPENMESH
	MyMesh mesh;
>>>>>>> origin/master


	//MyMesh mesh;

	//// generate vertices

	//MyMesh::VertexHandle vhandle[8];

	//vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));

	Mesh m;

	// u            x
	// +------------+
	// |            |
	// |            |
	// |      f     |
	// |            |
	// |            |
	// +------------+
	// v            w

	// Add the points as vertices
	vertex_descriptor u = m.add_vertex(K::Point_3(0, 1, 0));
	vertex_descriptor v = m.add_vertex(K::Point_3(0, 0, 0));
	vertex_descriptor w = m.add_vertex(K::Point_3(1, 0, 0));
	vertex_descriptor x = m.add_vertex(K::Point_3(1, 1, 0));

	/* face_descriptor f = */ m.add_face(u, v, w, x);

<<<<<<< HEAD

=======
	vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));
#endif
>>>>>>> origin/master

	//mesh transform

	//projection 
	



	Matrix4f  mm= pose.GetM();
	ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);
	
	for (int i = 0; i < meshold->noTotalTriangles; i++)
	{
<<<<<<< HEAD
		Vector4f p0;// = trivec[i].p0;
=======
/*
		Vector4f p0= trivec[i].p0;
>>>>>>> origin/master
		p0.w = 1;
		Vector4f pp0 = mm*p0;
		Vector4f p1;// = trivec[i].p1;
		p1.w = 1;
		Vector4f pp1 = mm*p1;

*/
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