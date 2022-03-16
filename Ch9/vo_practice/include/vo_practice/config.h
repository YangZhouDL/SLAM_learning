#ifndef CONFIG_H
#define CONFIG_H
#include "vo_practice/common_include.h"
namespace vo_practice
{
class Config
{
private:
    static shared_ptr<Config> config_;
    FileStorage file_;

    Config () {};   //private constructor makes a singleton
public:
    ~Config();      //close the file when deconstructing

    //set a new config file
    static void setParameterFile(const string& filename);

    //access the parameter values
    template<typename T>
    static T get(const string& key)
    {
        return T(Config::config_->file_[key]);
    }
};
}
#endif