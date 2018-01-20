#ifndef TOOL_H
#define TOOL_H
#include "Yslam/global.h"

namespace Yslam
{

class Tool
{
public:
	cv::Point2d pixel2cam(cv::Point2d p, Mat& K);
	double vnorm(Vector3d a);
	double getScale(Vector3d a,Vector3d b);
	Mat p3d2mat(Point3d p);
	Point3d mat2p3d(Mat mat);
	//Point3d scalePoint(Point3d x,double scale);
	Tool();
};

}

#endif
