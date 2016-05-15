
#include <GL/glew.h>


#ifdef FREEGLUT

#include <GL/freeglut.h>

#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif


#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
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
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>

// Some convenient typedefs.
//typedef CGAL::Simple_cartesian<float> Kernel;

typedef K::FT      Scalar;
//typedef K::Point_2 Point2;

typedef std::vector<Scalar> Scalar_vector;

typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<K> Triangle_coordinates;

using std::cout; using std::endl; using std::string;
using std::fstream; using std::ios;



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
typedef typename Polyhedron::Facet_iterator Facet_iterator;
typedef CGAL::Triangle_2<K >      Tri2;



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

bool MeshFusion::find3DPos(cv::Point2f p , cv::Point3f &ret)
{
	//bool ret = false;

	Parameterization_polyhedron_adaptor       mesh_adaptor(mymesh);
	Vertex_iterator  it = mesh_adaptor.mesh_vertices_begin();
	Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
	//int idx = 0;
	Vector4f intrinRGB =mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float mdis = 10000;


	while (it != ie)
	//for(int i = 0 ; i < meshVertex.size();i++)	
	{
		//cv::Point3f posf = meshVertex[i];
		CGAL::Point_3<K> pos = mesh_adaptor.get_vertex_position(it);
		//Vector3f p = Vector3f(pos.x(), pos.y(), pos.z());
		int ix = pos.x() * intrinRGB.x / pos.z() + intrinRGB.z;
		int iy = pos.y() * intrinRGB.y / pos.z() + intrinRGB.w;

		//int ix = posf.x * intrinRGB.x / posf.z + intrinRGB.z;
		//int iy = posf.y * intrinRGB.y / posf.z + intrinRGB.w;

		float dis = (ix - p.x)*(ix - p.x) + (iy - p.y)*(iy - p.y);
		if (dis <= 1)
		{
			ret=cv::Point3f(pos.x(), pos.y(), pos.z());
			return true;
		}
		else if (dis < mdis)
		{
			mdis = dis;
			ret= cv::Point3f(pos.x(), pos.y(), pos.z());
		}
		it++;

	}


	return false;
}

//for all m_base_corners to determine that whether it is a new vertex or not
//if it is a new vertex, contruct the new triangle



void MeshFusion::meshUpdate(ITMMesh * meshold ,ITMPose *pose)
{
	

	if (meshold->noTotalTriangles > 0)
	{

		std::vector<float> coords;
		std::vector<int>    tris;
		ITMMesh::Triangle * trivec = meshold->triangles->GetData(MEMORYDEVICE_CPU);

		//if (mymesh.size_of_vertices() > 0)
		//	return;

		for (int i = 0; i < meshold->noTotalTriangles; i++)
		{
			maintainList(trivec[i].p0, coords, tris);
			maintainList(trivec[i].p1, coords, tris);
			maintainList(trivec[i].p2, coords, tris);
		
		/*	meshVertex.push_back(cv::Point3f(trivec[i].p0.x, trivec[i].p0.y, trivec[i].p0.z));
			meshVertex.push_back(cv::Point3f(trivec[i].p1.x, trivec[i].p1.y, trivec[i].p1.z));
			meshVertex.push_back(cv::Point3f(trivec[i].p2.x, trivec[i].p2.y, trivec[i].p2.z));*/
		}

		mymesh.clear();

		polyhedron_builder<Polyhedron::HalfedgeDS > builder(coords, tris);
		mymesh.delegate(builder);
		return;


	}

	Vector4f intrinRGB = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	//meshVertex.clear();
	std::vector<Point2> meshProj;



	for ( int i = 0 ; i < totalVertex; i++)	
	{
		
		Vector3f vpos(meshVertex[i].x, meshVertex[i].y, meshVertex[i].z);
		Vector3f npos=pose->GetM()*vpos;

		float ix = npos.x * intrinRGB.x / npos.z + intrinRGB.z;
		float iy = npos.y * intrinRGB.y / npos.z + intrinRGB.w;
		//meshVertex.push_back(cv::Point3f(pos.x(),pos.y(),pos.z()));
		meshProj.push_back(Point2(ix,iy));
	}//end of for 

	 // Instantiate some interior, boundary, and exterior query points for which we compute coordinates.
//	 int number_of_query_points = vInputPoints.size();
	//const Point2 query_points[number_of_query_points];// = { Point(0.5f , 0.5f), Point(1.0f, 0.5f), Point(1.0f , 0.75f), Point(1.0f , 1.0f),                     // interior query points
		//Point(1.0f , 1.25f), Point(1.0f, 1.5f), Point(0.75f, 1.0f), Point(1.25f, 1.0f), Point(1.5f, 0.75f),
		//Point(1.0f , 0.25f), Point(0.5f, 1.0f), Point(1.5f , 1.25f), Point(1.0f , 2.0f), Point(2.0f, 0.5f), // boundary query points
		//Point(0.25f, 1.0f), Point(0.5f, 1.75f), Point(1.5f , 1.75f), Point(1.75f, 1.5f)                      // exterior query points
	//};

	fstream fout;
	fout.open("projdebug.txt", ios::out);

	fout << "0	0	1	setrgbcolor" << endl;
	int i = 0;
	std::vector <Point2>::const_iterator   it = vInputPoints.begin();
	while (it != vInputPoints.end()) {
		fout << *it << "  moveto" << endl;
		fout << ((*it).x() - 2) << " " << (*it).y() << " 2	0	360	arc" << endl;
		it++;
		i++;
		if (i == ncon)
		{
			fout << "stroke " << endl;
			fout << "0	1	0	setrgbcolor" << endl;

		}

	}
	Scalar_vector coordinates;
	coordinates.reserve(3);

	// Reserve memory to store triangle coordinates for 18 query points.
	

	// Construct a triangle
	int nface = totalFace / 3;
	for (int i = 0; i < nface; i++)
	{

	  Point2  first_vertex=meshProj[meshTri[3*i]];
	  Point2  second_vertex= meshProj[meshTri[3 * i+1]];
	  Point2  third_vertex= meshProj[meshTri[3 * i+2]];


	  fout << first_vertex << "  moveto" << endl;
	  fout << second_vertex << "  lineto" << endl;
	  fout << second_vertex << "  moveto" << endl;
	  fout << third_vertex << "  lineto" << endl;
	  fout << third_vertex << "  moveto" << endl;
	  fout << first_vertex << "  lineto" << endl;

		// Create an std::vector to store coordinates.

		// Instantiate the class Triangle_coordinates_2 for the triangle defined above.
		Triangle_coordinates triangle_coordinates(first_vertex, second_vertex, third_vertex);

		
		// Compute triangle coordinates for these points.
		//cout << endl << "Computed triangle coordinates: " << endl << endl;
		std::vector <Point2>::const_iterator   it = vInputPoints.begin();
		while (it !=vInputPoints.end()) {
			coordinates.clear();

			triangle_coordinates(*it, coordinates);
			if (coordinates[0] > 0 && coordinates[1] > 0 && coordinates[ 2] > 0)
			{
				it = vInputPoints.erase(it);
				
			}
			else
				it++;
		};

	}
	
	cout << vInputPoints.size() << endl;
	fout << "stroke" << endl;
	fout << "1	0	0	setrgbcolor" << endl;

   it = vInputPoints.begin();
	while (it != vInputPoints.end()) {
		fout << *it << "  moveto" << endl;
		fout << ((*it).x() - 1.5) << " " << (*it).y() << " 1.5	0	360	arc" << endl;
		it++;
			
	}


	fout.close();

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
			

	//		Parameterization_polyhedron_adaptor       mesh_adaptor(mymesh);
//			CGAL::Point_3<K> center = CGAL::Point_3<K>(0, 0, 300);
			cv::Point3f center = cv::Point3f(0, 0, 300);

	//	glTranslated(center.x, center.y,center.z);
			if (busepose)
				glMultMatrixf(pose->GetM().m);

			//int nv = meshVertex.size();//  mesh_adaptor.count_mesh_vertices();
			//int n = meshTri.size();// mesh_adaptor.count_mesh_facets();
			glewInit();

			GLuint VertexVBOID,IndexVBOID;
			glGenBuffers(1, &VertexVBOID);
			glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
			glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * totalVertex, &meshVertex[0].x, GL_STATIC_DRAW);

			glGenBuffers(1, &IndexVBOID);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ushort) * totalFace, meshTri, GL_STATIC_DRAW);

			//Define this somewhere in your header file
#define BUFFER_OFFSET(i) ((void*)(i))

			glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), BUFFER_OFFSET(0));   //The starting point of the VBO, for the vertices
			//glEnableClientState(GL_NORMAL_ARRAY);
			//glNormalPointer(GL_FLOAT, sizeof(MyVertex), BUFFER_OFFSET(12));   //The starting point of normals, 12 bytes away
			glClientActiveTexture(GL_TEXTURE0);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glTexCoordPointer(2, GL_FLOAT, sizeof(MyVertex), BUFFER_OFFSET(12));   //The starting point of texcoords, 24 bytes away

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
			//To render, we can either use glDrawElements or glDrawRangeElements
			//The is the number of indices. 3 indices needed to make a single triangle
			glDrawElements(GL_TRIANGLES, totalFace, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));   //The starting point of the IBO
																					//0 and 3 are the first and last vertices
																					//glDrawRangeElements(GL_TRIANGLES, 0, 3, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));   //The starting point of the IBO
																					//glDrawRangeElements may or may not give a performance advantage over glDrawElements



								   //Vertex_iterator  it = mesh_adaptor.mesh_vertices_begin();
			//Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
			//glScalef(1.f, -1.f, 1.f);
			//glBegin(GL);
			//int idx=0;
			//while (it != ie)
			//for(int i = 0 ; i < nv; i++)
			//{
			//	cv::Point3f pos( meshVertex[i].x, meshVertex[i].y, meshVertex[i].z);// mesh_adaptor.get_vertex_position(it);
			//	cv::Point2f uv(meshVertex[i].s0, meshVertex[i].t0);// //mesh_adaptor.get_vertex_uv(it);

			//	cv::Point3f spos=  pos - center;
			//	//if ( i < uvlist.size())
			//	//glTexCoord2f(uvlist[i].x , uvlist[i].y);
			//	//glVertex3f(spos.x, spos.y, spos.z);

			//	//idx++;
			//	//it++;
			//}
			//glEnd();



			
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