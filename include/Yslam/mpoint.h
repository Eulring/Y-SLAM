#ifndef MAPPOINT_H
#define MAPPOINT_H

namespace Yslam
{

class MapPoint
{
public:
	Point3d pos_;

	set<int> observed_frames_;

	int ob_num_;

	MapPoint()
	{
		ob_num_=0;
	};
};

}

#endif
