#ifndef FRAME_H
#define FRAME_H

#include "Yslam/global.h"

namespace Yslam
{

class Frame
{
public:
	int frameid_;
	Mat R_;
	Mat R_delta_;
	Mat t_;
	Mat t_delta_;
	double disperity_;
//	SE3 T_f_w_;
//	SE3 T_f_f_;

	Frame();
};

}

#endif
