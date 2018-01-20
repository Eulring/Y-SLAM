#include "Yslam/tool.h"

namespace Yslam
{

Tool::Tool(){}

cv::Point2d Tool::pixel2cam(cv::Point2d p, Mat& K)
{
	//cout<<p<<endl;
	//cout<<( p.x - K.at<float>(0,2) ) <<endl;
	//cout<<K.at<double>(0,0)<<endl;
	//cout<<K.at<double>(0,0)<<endl;
	return cv::Point2d
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
    );
}

double Tool::vnorm(Vector3d a)
{
	return a(0,0)*a(0,0)+a(1,0)*a(1,0)+a(2,0)*a(2,0);
}

double Tool::getScale(Vector3d a,Vector3d b)
{
	double na=vnorm(a);
	double nb=vnorm(b);
	return sqrt(na/nb);
}

Mat Tool::p3d2mat(Point3d p)
{
	Mat mat(3,1,CV_64F);
	mat.at<double>(0,0)=p.x;
	mat.at<double>(1,0)=p.y;
	mat.at<double>(2,0)=p.z;
	return mat;
}

Point3d Tool::mat2p3d(Mat mat)
{
	return Point3d(
		mat.at<double>(0,0),
		mat.at<double>(1,0),
		mat.at<double>(2,0)
	);
}

}
