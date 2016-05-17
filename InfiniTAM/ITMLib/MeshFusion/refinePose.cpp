
#include "MeshFusion.h"
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>
using namespace ITMLib::Objects;
typedef K::FT      Scalar;

typedef std::vector<Scalar> Scalar_vector;

typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<K> Triangle_coordinates;


float  MyTri::findError(MyTri & tridata, std::vector<float> & errList)
{
	Scalar_vector coordinates;
	coordinates.reserve(3);

	memset(stat, 0x00, sizeof(bool) * 2048);
	float ret = 0;
	int nerr = 0;
	// Construct a triangle
	int nface = tridata.totalFace / 3;
	for (int i = 0; i < nface; i++)
	{

		Point2  first_vertex = tridata.meshProj[tridata.meshTri[3 * i]];
		Point2  second_vertex = tridata.meshProj[tridata.meshTri[3 * i + 1]];
		Point2  third_vertex = tridata.meshProj[tridata.meshTri[3 * i + 2]];

		float d1 = tridata.meshDepth[tridata.meshTri[3 * i]];
		float d2 = tridata.meshDepth[tridata.meshTri[3 * i + 1]];
		float d3 = tridata.meshDepth[tridata.meshTri[3 * i + 2]];

		Triangle_coordinates triangle_coordinates(first_vertex, second_vertex, third_vertex);

		for (int j = 0; j < totalVertex; j++)
		{
			if (stat[j])
				continue;

			coordinates.clear();

			triangle_coordinates( meshProj[j], coordinates);
			if (coordinates[0] > 0 && coordinates[1] > 0 && coordinates[2] > 0)
			{
				int nx = (meshProj[j]).x();
				int ny = (meshProj[j]).y();

				float dz = meshDepth[j];
				float ed = d1*coordinates[0] + d2*coordinates[1] + d3*coordinates[2];
				float err = fabs(dz - ed);
				if (err < 5)
					stat[j] = true;
				else
				{
					float eval = dz - ed;
					errList.push_back(eval);
				
					ret += eval;
					nerr++;
				}

			}

		}//end for j

	}//end for i 

	if ( nerr > 0 )
	ret = ret / nerr;
	return ret;


}

void  MeshFusion::refinePose(ITMPose * pose)
{
	Vector4f intrinRGB = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;

	Matrix4f m = pose->GetM();
	Matrix4f invm = pose->GetInvM();

	//meshVertex.clear();

	mytriData.project(&m, intrinRGB);
	currTri.project(NULL, intrinRGB);

	std::vector<float> ret;
	float err=   currTri.findError(mytriData ,ret );
	Vector3f t=pose->GetT() + Vector3f(0, 0, err);
	pose->SetT(t);
	pose->Coerce();
	m = pose->GetM();
	mytriData.project(&m, intrinRGB);

	 err = currTri.findError(mytriData, ret);


	


}