#include "vo_practice/config.h"

namespace vo_practice 
{
    
void Config::setParameterFile( const string& filename )
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);
    config_->file_ = FileStorage( filename.c_str(), FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        cerr<<"parameter file "<<filename<<" does not exist."<<endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}