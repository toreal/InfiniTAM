
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

//#include <CGAL/Parameterization_polyhedron_adaptor_3.h>
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


//typedef CGAL::Parameterization_polyhedron_adaptor_3<Polyhedron>    Parameterization_polyhedron_adaptor;


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
//typedef CGAL::Triangle_2<K >      Tri2;



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
			B.add_vertex_to_facet(tris[i + 2]);
			B.add_vertex_to_facet(tris[i + 1]);
			
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

//	Parameterization_polyhedron_adaptor       mesh_adaptor(mymesh);
//	Vertex_iterator  it = mesh_adaptor.mesh_vertices_begin();
//	Vertex_iterator  ie = mesh_adaptor.mesh_vertices_end();
	//int idx = 0;
	Vector4f intrinRGB =mainView->calib->intrinsics_rgb.projectionParamsSimple.all;
	float mdis = 10000;


	//while (it != ie)
	////for(int i = 0 ; i < meshVertex.size();i++)	
	//{
	//	//cv::Point3f posf = meshVertex[i];
	//	CGAL::Point_3<K> pos = mesh_adaptor.get_vertex_position(it);
	//	//Vector3f p = Vector3f(pos.x(), pos.y(), pos.z());
	//	int ix = pos.x() * intrinRGB.x / pos.z() + intrinRGB.z;
	//	int iy = pos.y() * intrinRGB.y / pos.z() + intrinRGB.w;

	//	//int ix = posf.x * intrinRGB.x / posf.z + intrinRGB.z;
	//	//int iy = posf.y * intrinRGB.y / posf.z + intrinRGB.w;

	//	float dis = (ix - p.x)*(ix - p.x) + (iy - p.y)*(iy - p.y);
	//	if (dis <= 1)
	//	{
	//		ret=cv::Point3f(pos.x(), pos.y(), pos.z());
	//		return true;
	//	}
	//	else if (dis < mdis)
	//	{
	//		mdis = dis;
	//		ret= cv::Point3f(pos.x(), pos.y(), pos.z());
	//	}
	//	it++;

	//}


	return false;
}

//for all m_base_corners to determine that whether it is a new vertex or not
//if it is a new vertex, contruct the new triangle

void MeshFusion::buildMesh( MyTri * data  )
{


		std::vector<float> coords;
		std::vector<int>    tris;


		for (int i = 0; i < data->totalVertex; i++)
		{
			coords.push_back(data->meshVertex[i].x);
			coords.push_back(data->meshVertex[i].y);
			coords.push_back(data->meshVertex[i].z);
		}

		for (int i = 0; i < data->totalFace; i++)
		{
			tris.push_back(data->meshTri[i]);
		}

		mymesh.clear();

		polyhedron_builder<Polyhedron::HalfedgeDS > builder(coords, tris);
		mymesh.delegate(builder);
		return;


}

//tridata 即mytridata 即目前scan  的成果
//先將目前已scan 的data 轉到估計的current view
// 檢查current mesh 中的點是否為新點
// 將current mesh 依所估計的transform 轉回model view, 並加入新的點
//目前為完全copy 但實際上要依情況merge

void MeshFusion::meshUpdate(ITMMesh * mesho ,ITMPose *pose ,MyTri * tridata)
{
	
	Vector4f intrinRGB = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

	Matrix4f m=pose->GetM();
	Matrix4f invm = pose->GetInvM();

	//meshVertex.clear();
	
	tridata->project(&m, intrinRGB);
	currTri.project(NULL, intrinRGB);
	

		float* dp = proDepth->GetData(MEMORYDEVICE_CPU);
		int w = proDepth->noDims.x;
		memset(currTri.stat, 0x00, sizeof(bool) * 2048);
	fstream fout;
	fout.open("projdebug.txt", ios::out);

	fout << "0	1	1	setrgbcolor" << endl;
	int i = 0;
	//currTri.meshProj
	//std::vector <Point2>::const_iterator   it = vInputPoints.begin();
	//while (it != vInputPoints.end())
	/*for ( int i =0 ; i < currTri.totalVertex; i++)
	{
		fout << currTri.meshProj[i] << "  moveto" << endl;
		fout << ((currTri.meshProj[i]).x() - 2) << " " << (currTri.meshProj[i]).y() << " 2	0	360	arc" << endl;
	
	}*/
	Scalar_vector coordinates;
	coordinates.reserve(3);

	// Construct a triangle
	int nface = tridata->totalFace / 3;
	for (int i = 0; i < nface; i++)
	{

	  Point2  first_vertex=tridata->meshProj[tridata->meshTri[3*i]];
	  Point2  second_vertex= tridata->meshProj[tridata->meshTri[3 * i+1]];
	  Point2  third_vertex= tridata->meshProj[tridata->meshTri[3 * i+2]];

	 float d1 = tridata->meshDepth[tridata->meshTri[3 * i]];
	 float d2 = tridata->meshDepth[tridata->meshTri[3 * i+1]];
	 float d3 = tridata->meshDepth[tridata->meshTri[3 * i+2]];


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
		for (int j = 0; j < currTri.totalVertex; j++)
		//std::vector <Point2>::const_iterator   it = vInputPoints.begin();
		//while (it !=vInputPoints.end()) 
		{
			if (currTri.stat[j])
				continue;

			coordinates.clear();

			triangle_coordinates(currTri.meshProj[j], coordinates);
			if (coordinates[0] > 0 && coordinates[1] > 0 && coordinates[ 2] > 0)
			{
				int nx = (currTri.meshProj[j]).x();
				int ny = (currTri.meshProj[j]).y();

				float dz = currTri.meshDepth[j];
			//	float dd = estivalue(dp, Vector2i(nx, ny), NULL);
			//	float dd2 = dp[nx + ny*w];
				float ed = d1*coordinates[0] + d2*coordinates[1] + d3*coordinates[2];
				float err = fabs(dz - ed);
				if (err < 5)
					currTri.stat[j] = true;
					//it = vInputPoints.erase(it);
				else
				{
		//			currTri.stat[j] = true;
					fout << "stroke" << endl;
					float evalue = err / 50;
					if (evalue > 1)
						evalue = 1;
					fout << nx <<" " << ny  << "  moveto" << endl;
					fout << nx-2 << " " << ny << " 2	0	360	arc" << endl;
					fout << "stroke" << endl;
					fout << nx << " " << ny << "  moveto" << endl;
					fout << "(" << (int)(dz-ed) << ") show" << endl;

			//		cout << "depth err" << err << endl;
					//it++;
				}
				
			}
			
		};

	}
	
//	cout << vInputPoints.size() << endl;
	//fout << "stroke" << endl;
	//fout << "1	0	0	setrgbcolor" << endl;

 //  //it = vInputPoints.begin();
	////while (it != vInputPoints.end()) 
	//for (int j = 0; j < currTri.totalVertex; j++)
	//{
	//	if (currTri.stat[j])
	//	{
	//		fout << currTri.meshProj[j] << "  moveto" << endl;
	//		fout << ((currTri.meshProj[j]).x() - 1.5) << " " << (currTri.meshProj[j]).y() << " 1.5	0	360	arc" << endl;
	//	//	it++;
	//	}
	//}
	
	int nnode[2048];

	int ocase = -1;
	 nface = currTri.totalFace / 3;
	 for (int i = 0; i < nface; i++)
	 {
		 int n0 = currTri.meshTri[3 * i];
		 int n1 = currTri.meshTri[3 * i + 1];
		 int n2 = currTri.meshTri[3 * i + 2];
		 Point2  first_vertex = currTri.meshProj[n0];
		 Point2  second_vertex = currTri.meshProj[n1];
		 Point2  third_vertex = currTri.meshProj[n2];

		 bool s0 = currTri.stat[n0];
		 bool s1 = currTri.stat[n1];
		 bool s2 = currTri.stat[n2];
	 
		 if (!s0 && !s1 && !s2)
		 {
			 if (ocase != 0)
			 {
				 fout << "stroke" << endl;
				 fout << "1	0	0	setrgbcolor" << endl;
			 }
			 ocase = 0;
			 Vector3f pos0(currTri.meshVertex[n0].x, currTri.meshVertex[n0].y, currTri.meshVertex[n0].z);
			 Vector3f pos1(currTri.meshVertex[n1].x, currTri.meshVertex[n1].y, currTri.meshVertex[n1].z);
			 Vector3f pos2(currTri.meshVertex[n2].x, currTri.meshVertex[n2].y, currTri.meshVertex[n2].z);

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
		 else if (s0 && s1 && s2)
		 {
			 if (ocase != 1)
			 {
				 fout << "stroke" << endl;
				 fout << "1	0.7	0	setrgbcolor" << endl;
			 }
			 ocase = 1;
		 }
		 else
		 {
			 if (ocase != 2)
			 {
				 fout << "stroke" << endl;
				 fout << "0	1	0	setrgbcolor" << endl;
			 }
			 ocase = 2;
		 }
	

		fout << first_vertex << "  moveto" << endl;
		fout << second_vertex << "  lineto" << endl;
		fout << second_vertex << "  moveto" << endl;
		fout << third_vertex << "  lineto" << endl;
		fout << third_vertex << "  moveto" << endl;
		fout << first_vertex << "  lineto" << endl;

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
		gluPerspective(60.0f, aspect, 100, 10000.0f);
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
			glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * mytriData.totalVertex, &mytriData.meshVertex[0].x, GL_STATIC_DRAW);

			glGenBuffers(1, &IndexVBOID);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ushort) * mytriData.totalFace, mytriData.meshTri, GL_STATIC_DRAW);

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
			glDrawElements(GL_TRIANGLES, mytriData.totalFace, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));   //The starting point of the IBO
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
	buildMesh(&mytriData);

	std::fstream fs;
	fs.open(fn, std::fstream::out);

	//CGAL::VRML_2_ostream out(fs);
	fs << mymesh;
	fs.close();


}