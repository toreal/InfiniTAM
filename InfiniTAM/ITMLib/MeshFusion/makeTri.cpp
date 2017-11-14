

#include "MeshFusion.h"


void MeshFusion::makeTri(ITMMesh * mesh, MyTri * tridata)
{

	std::vector<Point2> vInDSPoints;
	std::vector<Point2> conDSbeg;
	std::vector<Point2> conDSend;

	Vector4f  intrinparam = mainView->calib->intrinsics_rgb.projectionParamsSimple.all;



	for (int i = 0; i < ds.ndface; i++)
	{
		int x = intrinparam.x*ds.centerbuf[i].x / ds.centerbuf[i].z + intrinparam.z;
		int y = intrinparam.y*ds.centerbuf[i].y / ds.centerbuf[i].z + intrinparam.w;

		vInDSPoints.push_back( Point2(x,y));
	}


	

	this->constructMesh(mesh, tridata, vInDSPoints, conDSbeg, conDSend);


	


}

