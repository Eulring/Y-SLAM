#ifndef G2OPOSE_H
#define G2OPOSE_H

#include "Yslam/global.h"
#include "Yslam/frame.h"

namespace Yslam
{


class G2OPose
{
public:
	G2OPose();
	void poseGraphOpt(vector<Frame>& frames);
	void poseGraphOpt2(vector<Frame>& frames);
	void writeG2oFile(vector<Frame>& frames);
	void updateFrames(vector<Frame>& frames);

};


}

#endif
