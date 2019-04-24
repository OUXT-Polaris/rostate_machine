#include <rostate_machine/callback_func.h>

namespace rostate_machine
{
    CallbackFunc::CallbackFunc(std::function<boost::optional<rostate_machine::Event>(void)> func)
    {
        func_ = func;
    }

    CallbackFunc::~CallbackFunc()
    {

    }

    void CallbackFunc::addTag(std::string tag)
    {
        tag_.push_back(tag);
        return;
    }

    boost::optional<rostate_machine::Event> CallbackFunc::callback(std::string tag)
    {
        bool matched = false;
        for(auto itr = tag_.begin(); itr != tag_.end(); itr++)
        {
            if(*itr == tag)
            {
                matched = true;
            }
        }
        if(matched)
        {
            return func_();
        }
        return boost::none;
    }
}