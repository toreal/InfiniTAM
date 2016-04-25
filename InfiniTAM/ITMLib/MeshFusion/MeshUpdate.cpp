
#include "MeshFusion.h"
#include "../Objects/ITMPose.h"



#include <vector>

#include <boost/foreach.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Parameterization_polyhedron_adaptor_3.h>


#include <iostream>
#include <string.h>
#include <ctype.h>
#include <fstream>
#include <cassert>



// ----------------------------------------------------------------------------
// Private types
// ----------------------------------------------------------------------------
typedef CGAL::Simple_cartesian<double> K;

typedef CGAL::Polyhedron_3<K >      Polyhedron;

typedef CGAL::Parameterization_polyhedron_adaptor_3<Polyhedron>    Parameterization_polyhedron_adaptor;




// Type describing a border or seam as a vertex list
typedef std::list<Parameterization_polyhedron_adaptor::Vertex_handle> Seam;




typedef           Parameterization_polyhedron_adaptor::Vertex_handle  SeamHandle;
typedef typename Polyhedron::Vertex_iterator Vertex_iterator;



using namespace ITMLib::Objects;

template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
	std::vector<double> &coords;
	std::vector<int>    &tris;
	polyhedron_builder(std::vector<double> &_coords, std::vector<int> &_tris) : coords(_coords), tris(_tris) {}
	void operator()(HDS& hds) {
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;

		// create a cgal incremental builder
		CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
		B.begin_surface(coords.size() / 3, tris.size() / 3);

		// add the polyhedron vertices
		for (int i = 0; i<(int)coords.size(); i += 3) {
			B.add_vertex(Point(coords[i + 0], coords[i + 1], coords[i + 2]));
		}

		// add the polyhedron triangles
		for (int i = 0; i<(int)tris.size(); i += 3) {
			B.begin_facet();
			B.add_vertex_to_facet(tris[i + 0]);
			B.add_vertex_to_facet(tris[i + 1]);
			B.add_vertex_to_facet(tris[i + 2]);
			B.end_facet();
		}

		// finish up the surface
		B.end_surface();
	}
};



void MeshFusion::meshUpdate(ITMMesh * meshold)
{
	ITMPose pose;

	std::vector<double> coords;
	std::vector<int>    tris;


	typedef Polyhedron::HalfedgeDS             HDS;

	Polyhedron m;
	coords.push_back(0);
	coords.push_back(0);
	coords.push_back(0);
	coords.push_back(0);
	coords.push_back(1);
	coords.push_back(0);
	coords.push_back(1);
	coords.push_back(1);
	coords.push_back(0);
	tris.push_back(0);
	tris.push_back(1);
	tris.push_back(2);
	
	polyhedron_builder<HDS> builder(coords, tris);
	m.delegate(builder);

	Parameterization_polyhedron_adaptor       mesh_adaptor(m);

	
	int nv=mesh_adaptor.count_mesh_vertices();
	int n=mesh_adaptor.count_mesh_facets();
	Vertex_iterator  it=mesh_adaptor.mesh_vertices_begin();
	Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
	Polyhedron::Halfedge_handle seam_halfedges[10];
	Polyhedron::Halfedge_handle seam_half[10];
	float v = 0.001;
	while (it!=ie)
	{ 	K::Point_3 pp=  mesh_adaptor.get_vertex_position(it);
	
	// map vertex on unit circle
	        CGAL::Point_2<K> uv;
	             uv = CGAL::Point_2<K>(0.5+v , 0.5 );
				 v += 0.002;
             mesh_adaptor.set_vertex_uv(it, uv);
			 uv=mesh_adaptor.get_vertex_uv(it);
			 it++;
	}

	 


	//mesh transform

	//projection 
	



	Matrix4f  mm= pose.GetM();
	ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);
	
	for (int i = 0; i < meshold->noTotalTriangles; i++)
	{

		Vector4f p0;// = trivec[i].p0;

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