

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif


#include "MeshFusion.h"
#include "../Objects/ITMPose.h"



#include <vector>

#include <boost/foreach.hpp>

#include <CGAL/Parameterization_polyhedron_adaptor_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Polyhedron_VRML_2_ostream.h>
#include <fstream>

#include <iostream>
#include <string.h>
#include <ctype.h>
#include <fstream>
#include <cassert>



// ----------------------------------------------------------------------------
// Private types
// ----------------------------------------------------------------------------


typedef CGAL::Parameterization_polyhedron_adaptor_3<Polyhedron>    Parameterization_polyhedron_adaptor;

//
//// Type describing a border or seam as a vertex list
//typedef std::list<Parameterization_polyhedron_adaptor::Vertex_handle> Seam;
//
//
//
//
//typedef           Parameterization_polyhedron_adaptor::Vertex_handle  SeamHandle;
typedef typename Polyhedron::Vertex_iterator Vertex_iterator;



using namespace ITMLib::Objects;

template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
	std::vector<float> &coords;
	std::vector<int>    &tris;
	polyhedron_builder(std::vector<float> &_coords, std::vector<int> &_tris) : coords(_coords), tris(_tris) {}
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

void maintainList(Vector3f vec, std::vector<float> & coords, std::vector<int>   & tris)
{
	
	coords.push_back(vec.x);
	coords.push_back(vec.y);
	coords.push_back(vec.z);
	int n = coords.size()/3-1;
	tris.push_back(n);

}

void MeshFusion::meshUpdate(ITMMesh * meshold)
{
	ITMPose pose;

	std::vector<float> coords;
	std::vector<int>    tris;
	ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < meshold->noTotalTriangles; i++)
	{
		maintainList(trivec[i].p0, coords, tris);
		maintainList(trivec[i].p1, coords, tris);
		maintainList(trivec[i].p2, coords, tris);
	}	
	
	mymesh.clear();

	polyhedron_builder<Polyhedron::HalfedgeDS > builder(coords, tris);
	mymesh.delegate(builder);

	Parameterization_polyhedron_adaptor       mesh_adaptor(mymesh);	
	int nv=mesh_adaptor.count_mesh_vertices();
	int n=mesh_adaptor.count_mesh_facets();
	Vertex_iterator  it=mesh_adaptor.mesh_vertices_begin();
	Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
	int idx = 0;
	while (it!=ie)
	{ 
		CGAL::Point_2<K> uv(uvlist[idx].x, uvlist[idx].y);	  
        mesh_adaptor.set_vertex_uv(it, uv);		
		idx++;
		if (idx >= uvlist.size())
			break;
	   it++;
	}

	meshold->noTotalTriangles = 0;

	//mesh transform

	//projection 
	



	Matrix4f  mm= pose.GetM();
	//ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);
	
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


void MeshFusion::MeshFusion_Model(float fstartx, float fstarty, float fwidth, float fheight, bool busepose, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	GLint viewport[4];
	GLdouble mvmatrix[16];
	float projmatrix[16]= { 614,0.0000,-308,0.0000,
		                    0.0000,614,-234,0.0000,
		                    0.0000,0.0000,700,100000,
		                    0.0000,0.0000,-1,0.0000 };

	GLuint textureID;

	

	glPushMatrix();
	{
		glGetIntegerv(GL_VIEWPORT, viewport);
		glViewport(viewport[2]*fstartx, viewport[3] * fstarty, viewport[2] * fwidth, viewport[3] * fheight);
		glColor3f(1.0f, 1.0f, 1.0f);
		glEnable(GL_TEXTURE_2D); // Enable texturing	

		if (mainView != NULL)
		{
			glGenTextures(1, &textureID); // Obtain an id for the texture
			glBindTexture(GL_TEXTURE_2D, textureID); // Set as the current texture

			Vector4u* buf = this->mainView->rgb->GetData(MEMORYDEVICE_CPU);

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 640, 480, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		float aspect = viewport[2] * fwidth/ (viewport[3] * fheight);
		gluPerspective(60.0f, aspect, 0.1, 10000.0f);
		//glLoadMatrixf(projmatrix);

		//glOrtho(-640 / 2, 640 / 2, -480 / 2, 480 / 2, 200, 500);
		//glOrtho(0, 640, 480, 0, 200, 500);
		//glMultMatrixf(projmatrix);



		glMatrixMode(GL_MODELVIEW);
		
		glLoadIdentity();
		//glRotatef(shift, 0, 0, 1);
		//glTranslatef(-60, 0, -600);
	
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

		/*glGetIntegerv(GL_VIEWPORT, viewport);
		glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
		glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
/*
		/*glCullFace(GL_CW);
		glEnable(GL_CULL_FACE);
*/
		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glPushMatrix();
		{
			

			Parameterization_polyhedron_adaptor       mesh_adaptor(mymesh);
			CGAL::Point_3<K> center = CGAL::Point_3<K>(0, 0, 300);
			glTranslated(center.x(), center.y(),center.z());
			if (busepose)
				glMultMatrixf(pose->GetM().m);

			int nv = mesh_adaptor.count_mesh_vertices();
			int n = mesh_adaptor.count_mesh_facets();
			Vertex_iterator  it = mesh_adaptor.mesh_vertices_begin();
			Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
			//glScalef(1.f, -1.f, 1.f);
			glBegin(GL_TRIANGLES);
			int idx=0;
			while (it != ie)
			{
				CGAL::Point_3<K> pos= mesh_adaptor.get_vertex_position(it);
				CGAL::Point_2<K> uv=mesh_adaptor.get_vertex_uv(it);

				CGAL::Vector_3<K> spos=  pos - center;
				glTexCoord2f(uvlist[idx].x , uvlist[idx].y);
				glVertex3f(spos.x(), spos.y(), spos.z());

				idx++;
				it++;
			}
			glEnd();



			
		}
		glPopMatrix();
	}
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}


void MeshFusion::writeMesh(char * fn)
{
	std::fstream fs;
	fs.open(fn, std::fstream::out);

	//CGAL::VRML_2_ostream out(fs);
	fs << mymesh;
	fs.close();


}