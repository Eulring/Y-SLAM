#include "Yslam/vodometry.h"

namespace Yslam
{

VO::VO()
{
	cur_id_=0;
	cam_=Camera();
	f2d_ = cv::xfeatures2d::SIFT::create();
}

//############################################################################//
//############################################################################//
//############################################################################//
//  main function

void VO::addImage(int image_id)
{
	img_cur_=cam_.getImageById(image_id);
	processImage();
	Frame frame;
	if(cur_id_==0)
	{
		processFrameFirst();
		frames_.push_back(frame_cur_);
		addFrame();
	}
	else if(cur_id_==-1)
	{

	}
	else
	{
		//cout<<" : 1"<<endl;
		processFrameNormal();
		if(frame_cur_.disperity_<50) return ;
		//cout<<" : 2"<<endl;
		generatePoint();
		//cout<<" : 3"<<endl;
		frames_.push_back(frame_cur_);
		addFrame();
	}

}

//############################################################################//
//############################################################################//
//############################################################################//
//  image process part

void VO::addFrame()
{
	cur_id_++;
	img_ref_=img_cur_;
	kpt_ref_=kpt_cur_;
	/*
	cout<<kpt_cur_.size()<<"  -  "<<kpt_ref_.size()<<endl;
	kpt_ref_.clear();
	for(int i=0;i<kpt_cur_.size();i++)
		kpt_ref_.push_back(kpt_cur_[i]);
	kpt_cur_.clear();
	*/
	descriptor_ref_=descriptor_cur_;

	matches_.clear();
}

void VO::processImage()
{
	f2d_->detect(img_cur_,kpt_cur_);
	f2d_->compute(img_cur_,kpt_cur_,descriptor_cur_);
}

void VO::processFrameFirst()
{
	frame_cur_=Frame();
	Mat R=Mat::eye(3,3,CV_64F);
	Mat t=Mat::zeros(3,1,CV_64F);
	frame_cur_.R_=R; frame_cur_.t_=t;

}

void VO::processFrameNormal()
{
	frame_cur_=Frame();

	matcher_.match(descriptor_ref_,descriptor_cur_,matches_);
	//cout<<matches_.size()<<endl;
	vector<cv::Point2f> pt_ref,pt_cur;
	for(int i=0;i<matches_.size();i++)
	{
		pt_ref.push_back(kpt_ref_[matches_[i].queryIdx].pt);
		pt_cur.push_back(kpt_cur_[matches_[i].trainIdx].pt);
	}

	vector<uchar> m_RANSACStatus;
	Mat f_matrix=findFundamentalMat(pt_ref,pt_cur,m_RANSACStatus,CV_FM_RANSAC);
	//cout<<f_matrix<<endl;

	for(int i=m_RANSACStatus.size()-1;i>=0;i--)
		if(m_RANSACStatus[i]==0)
			matches_.erase(matches_.begin()+i);

	matchFilterByDis();
	//cout<<1<<endl;
	Mat res;
	drawMatches(img_ref_,kpt_ref_,img_cur_,kpt_cur_,matches_,res);
	imshow("res",res);  cv::waitKey(1);


	pt_ref.clear(),pt_cur.clear();
	for(int i=0;i<matches_.size();i++)
	{
		pt_ref.push_back(kpt_ref_[matches_[i].queryIdx].pt);
		pt_cur.push_back(kpt_cur_[matches_[i].trainIdx].pt);
	}

	//if(cur_id_==1) for(int i=0;i<pt_ref.size();i++) cout<<pt_ref[i]<<endl;
	//cout<<1<<endl;
	Mat R,t,opt;
	Mat e_matrix=cv::findEssentialMat(
		pt_ref,pt_cur,cam_.focal_len_,cam_.principal_point_,cv::RANSAC,0.99,1.0,opt);
	recoverPose (e_matrix,pt_ref,pt_cur,R,t,cam_.focal_len_,cam_.principal_point_);

	//cout<<cam_.principal_point_<<" "<<cam_.focal_len_<<endl;
	//cout<<e_matrix<<endl;
	//cout<<R.type()<<endl;
	frame_cur_.disperity_=getDisperity(pt_ref,pt_cur,R);
	frame_cur_.R_delta_=R;
	frame_cur_.t_delta_=t;

	int cur_id=frames_.size()-1;
	frame_cur_.t_=frames_[cur_id].t_+frames_[cur_id].R_*t;
	frame_cur_.R_=R*frames_[cur_id].R_;
	frame_cur_.frameid_=cur_id;

}

double VO::getDisperity(vector<cv::Point2f>& pt_1,vector<cv::Point2f>&pt_2,Mat R)
{
	Mat H=cam_.K_*R*cam_.K_.inv();
	//cout<<2<<endl;
	vector<Point3f> pt3_0,pt3_2;
	vector<float> diss;
	int n=pt_1.size();

	for(int i=0;i<n;i++)
	{
		Point3d tmp;
		tmp.x=pt_1[i].x;
		tmp.y=pt_1[i].y;
		tmp.z=1.0;
		pt3_0.push_back(tmp);

		Mat tmt(3,1,CV_64F);
		tmt.at<double>(0,0)=pt_2[i].x;
		tmt.at<double>(1,0)=pt_2[i].y;
		tmt.at<double>(2,0)=1;
		tmt=H*tmt;
		tmt=tmt/tmt.at<double>(2,0);

		tmp.x=tmt.at<double>(0,0);
		tmp.y=tmt.at<double>(1,0);
		pt3_2.push_back(tmp);

		diss.push_back(norm(pt3_0[i]-pt3_2[i]));
	}
	sort(diss.begin(),diss.end());

	return diss[n/2];
}

//############################################################################//
//############################################################################//
//############################################################################//
//  plot function


void VO::plotTrajectoryAll(cv::viz::Viz3d& window)
{
	vector<cv::Point3d> vtra;
	cv::Point3d tmp_c;
	for(int i=0;i<frames_.size();i++)
	{
		Mat mt=frames_[i].t_;
		tmp_c.x=mt.at<double>(0,0);
		tmp_c.y=mt.at<double>(1,0);
		tmp_c.z=mt.at<double>(2,0);
		vtra.push_back(tmp_c);
	}

	for(int i=1;i<vtra.size()-1;i++)
	{
		ostringstream oss;
		oss<<i;
		cv::viz::WArrow arrow(vtra[i-1],vtra[i]);
		window.showWidget(oss.str(),arrow);
	}
}

void VO::plotPointAll(cv::viz::Viz3d& window)
{
	vector<Point3d> vpt;
	for(int i=0;i<points_.size();i++)
	{
		if(points_[i].ob_num_<2) continue;
		//if(norm(points_[i].pos_)>200) continue;
		vpt.push_back(points_[i].pos_);
	}
	cout<<vpt.size()<<endl;
	cv::viz::WCloud point(vpt,cv::viz::Color::green());
	window.showWidget("all_points",point);
}

void VO::plotPoints(
	cv::viz::Viz3d& window,
	vector<Point3d>& mpt,
	cv::viz::Color clr,
	string name)
{
	cv::viz::WCloud point(mpt, clr);
	window.showWidget(name,point);
}



















}
