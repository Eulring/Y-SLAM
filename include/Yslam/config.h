#ifndef CONFIG_H
#define CONFIG_H

#include "Yslam/global.h"

namespace Yslam
{

class Config
{
public:
	static std::shared_ptr<Config> config_;
	static void setConfigFile (const string& filename);
	
	template <typename T>
	static T get (const string& key)
	{
		return T (config_->file_[key]);
	}
private:
	cv::FileStorage file_;
	Config () {};
};

}
#endif
