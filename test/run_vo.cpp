#include "Yslam/camera.h"
#include "Yslam/config.h"
#include "Yslam/vodometry.h"


int main()
{
	Yslam::Config::setConfigFile("config.yaml");
	Yslam::VO vo=Yslam::VO();

	cv::viz::Viz3d myWindow1("main");
	cv::viz::Viz3d myWindow2("afterpose");

	for(int i=0;i<=2640;i+=20)
	{
		vo.addImage(i);
	}
	vo.addImage(0);

	vo.plotTrajectoryAll(myWindow1);
	vo.plotPointAll(myWindow1);
	myWindow1.spin();

	vo.g2opose_.writeG2oFile(vo.frames_);
	vo.g2opose_.poseGraphOpt2(vo.frames_);
	vo.g2opose_.updateFrames(vo.frames_);

	vo.plotPointAll(myWindow2);
	vo.plotTrajectoryAll(myWindow2);
	myWindow2.spin();
}
