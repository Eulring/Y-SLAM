#ifndef CAMERA_H
#define CAMERA_H

#include "global.h"
#include "Yslam/config.h"

namespace Yslam
{

class Camera
{
public:
	float fx_;
	float fy_;
	float cx_;
	float cy_;
	double focal_len_;
	cv::Point2d principal_point_;
	Mat K_;

	Camera();
	void storeImageName();
	Mat getImageById(int id);

private:
	string dataset_address_;
	vector<string> image_names_;
};

}
#endif
