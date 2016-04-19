


#include "MeshFusion.h"
#include <algorithm>
#include <vector> 
#include <iterator> 
# include <iostream> 
#include "../../ORUtils/psimpl.h"
//#include <math.h>

using namespace ITMLib::Objects;

void MeshFusion::sortpoint()
{
	std::vector<std::pair<float, int>> baryProc;

	for (int i = 0; i < npoint; i++)
	{
		Vector2i  pp = pointlist[(i + npoint - 1) % npoint];
		Vector2i  np = pointlist[(i +  1) % npoint];
		Vector2i  p = pointlist[i ];

		Vector2i pd = pp - p;
		Vector2i nd = np - p;

		pd = pd / length(pd);
		nd = nd / length(nd);
		
		float fdot = pd.x*nd.x + pd.y*nd.y;   // # dot product
		float fdet = pd.x*nd.y - pd.y*nd.x;     //# determinant
		float angle = atan2(fdet, fdot);   //# atan2(y, x) or atan2(sin, cos)
		
		baryProc.push_back(std::make_pair(angle, i));
		
	}
	std::sort(baryProc.begin(), baryProc.end());

	for (std::vector<std::pair<float, int>>::const_iterator itr = baryProc.begin();
		itr != baryProc.end(); ++itr)
	{

		std::cout << itr->second << " - $" << itr->first << " values\n";

	}

	std::vector<double > mGeneratedCoords, mSimplifiedCoords;

	//QVector <qreal> mGeneratedCoords;
	//QVector <qreal> mSimplifiedCoords;
	mSimplifiedCoords.clear();
	int count = 50;
	//SignalConvertingPolyline();
	std::vector <double> generatedPoints ;
	std::vector <double> result;
	// simplify
	//SignalSimplifyingPolyline();
	std::vector <double>::const_iterator begin = generatedPoints.begin();
	std::vector <double>::const_iterator end = generatedPoints.end();

	std::back_inserter(generatedPoints);
	//t.start();
	psimpl::simplify_douglas_peucker_n<2>(begin, end, count,std::back_inserter(result) );
	//SignalCleaningConvertedPolyline();
	//DoSignalSimplifiedPolyline(duration);


}
	 