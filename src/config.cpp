#include "Yslam/config.h"

namespace Yslam
{

shared_ptr<Config> Config::config_ = nullptr;

void Config::setConfigFile (const std::string& filename)
{
	string file_address="../config/"+filename;

	if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);

	config_->file_=cv::FileStorage (file_address.c_str(),cv::FileStorage::READ);

	if (config_->file_.isOpened()==false)
	{
		cerr<<"file "<<filename<<" does not exist."<<endl;
		config_->file_.release();
		return ;
	}
	cout<<file_address<<endl;
}

}
