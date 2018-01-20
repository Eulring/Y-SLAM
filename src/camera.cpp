#include "Yslam/camera.h"
#include "Yslam/config.h"

namespace Yslam
{

Camera::Camera()
{

	fx_=Config::get<float>("camera_fx");
	fy_=Config::get<float>("camera_fy");
	cx_=Config::get<float>("camera_cx");
	cy_=Config::get<float>("camera_cy");

	focal_len_=(fx_+fy_)/2;
	principal_point_=cv::Point2d(cx_,cy_);

	K_=Mat::zeros(3,3,CV_64F);
	K_.at<double>(0,0)=fx_; K_.at<double>(1,1)=fy_;
	K_.at<double>(0,2)=cx_; K_.at<double>(1,2)=cy_;
	K_.at<double>(2,2)=1;

	dataset_address_=Config::get<string>("dataset_dir");
	storeImageName();

	cout<<"camera created"<<endl;

}

void Camera::storeImageName()
{
	image_names_.clear();
	cout<<dataset_address_+"rgb.txt"<<endl;
	ifstream in(dataset_address_+"rgb.txt");
	char buff[111],buff1[111],buff2[111];

	while(!in.eof())
	{
		in.getline(buff,100);
		if(buff[0]=='#') continue;
		sscanf(buff,"%s %s",buff1,buff2);

		image_names_.push_back(buff2);
	}

	cout<<image_names_.size()<<endl;
}

Mat Camera::getImageById(int id)
{
	string head=dataset_address_;
	head+=image_names_[id];
	cv::Mat img(cv::imread(head,0));
	return img;
}

}
