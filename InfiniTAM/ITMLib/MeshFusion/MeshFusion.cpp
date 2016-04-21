


#include "MeshFusion.h"
#include <algorithm>
#include <vector> 
#include <iterator> 
# include <iostream> 
#include "../../ORUtils/psimpl.h"
//#include <math.h>

using namespace ITMLib::Objects;
using namespace std;


void MeshFusion::sortpoint(ITMUChar4Image * draw)
{
	
	int count = npoint;
	
	std::vector <int> generatedPoints ;
	std::vector <int> result;
	
	for (int i = 0; i < npoint; i++)
	{
		generatedPoints.push_back(pointlist[i].x);
		generatedPoints.push_back(pointlist[i].y);

	}
		
	// simplify
	std::vector <int>::const_iterator begin = generatedPoints.begin();
	std::vector <int>::const_iterator end = generatedPoints.end();
		
	psimpl::simplify_douglas_peucker_n<2>(begin, end, count,std::back_inserter(result) );
	
	std::cout <<result.size() << std::endl;

	if (draw == NULL)
		return;

	Vector4u* data = draw->GetData(MEMORYDEVICE_CPU);
	for (vector<int>::iterator it = result.begin(); it != result.end(); ++it) {
		int x = *it;
		++it;
		int y = *it;
		cout << x <<"," << y << endl;
		
		
		if (draw != NULL)
		{
			int idx = x + y*draw->noDims.x;
			data[idx].x = 255;
			data[idx].y = 255;
			
			data[idx+1].x = 255;
			data[idx+1].y = 255;

			data[idx-1].x = 255;
			data[idx-1].y = 255;


		}
	}

	

}
	 