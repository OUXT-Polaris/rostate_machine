#ifndef ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED
#define ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED

// Headers in STL
#include <functional>
#include <vector>

// Headers in Boost
#include <boost/optional.hpp>

// Headers in this package
#include <rostate_machine/Event.h>

namespace rostate_machine
{
    class CallbackFunc
    {
    public:
        CallbackFunc(std::function<boost::optional<rostate_machine::Event>(void)> func);
        ~CallbackFunc();
        void addTag(std::string tag);
        boost::optional<rostate_machine::Event> callback(std::string tag);
    private:
        std::function<boost::optional<rostate_machine::Event>(void)> func_;
        std::vector<std::string> tag_;
    };
}

#endif  //ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED