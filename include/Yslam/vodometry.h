#ifndef VODOMETRY_H
#define VODOMETRY_H

#include "Yslam/camera.h"
#include "Yslam/frame.h"
#include "Yslam/global.h"
#include "Yslam/tool.h"
#include "Yslam/mpoint.h"
#include "Yslam/g2opose.h"

namespace Yslam
{

class VO
{
public:
	Camera cam_;
	Tool tool_;
	G2OPose g2opose_;

	int cur_id_;
	Mat img_ref_;
	Mat img_cur_;
	Mat descriptor_cur_;
	Mat descriptor_ref_;
	vector<cv::KeyPoint> kpt_ref_;
	vector<cv::KeyPoint> kpt_cur_;

	cv::Ptr<cv::Feature2D> f2d_;
	cv::BFMatcher matcher_;

	vector<Frame> frames_;
	vector<cv::DMatch> matches_;

	Frame frame_cur_;

	VO();
	void addImage(int image_id);
	void addFrame();
	void processImage();
	void processFrameFirst();
	void processFrameNormal();
	double getDisperity(vector<cv::Point2f>& pt_1,vector<cv::Point2f>&pt_2,Mat R);


//############################################################################//
	//cv::viz::Viz3d window_;
	void plotTrajectoryAll(cv::viz::Viz3d& window);
	void plotPointAll(cv::viz::Viz3d& window);
	void plotPoints(cv::viz::Viz3d& window,vector<Point3d>& pt3,cv::viz::Color clr,string name);

//############################################################################//
	vector<Point3d> point3d_[11];
	vector<Point3d> point3d_cur_;
	vector<MapPoint> points_;
	void generatePoint();
	void triangulation();
	void matchFilterByDis();
	void matchFilterByPt3();
	void matchFilterByCam();
	void updateScale(double scale);
	double getScale(Point3d p,Point3d p_);
	void relocate();

};

}

#endif
