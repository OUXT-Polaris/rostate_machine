#ifndef ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED
#define ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED

// Headers in STL
#include <functional>
#include <vector>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

// Headers in this package
#include <rostate_machine/Event.h>
#include <rostate_machine/State.h>

namespace rostate_machine
{
    constexpr int always = 0;
    constexpr int on_entry = 1;
    constexpr int on_exit = 2;

    struct CallbackInfo
    {
        const std::string tag;
        const int when;
        const std::vector<std::string> states;
        CallbackInfo(std::string tag,int when,std::vector<std::string> states) 
            : tag(tag),when(when),states(states) {}
    };

    class CallbackFunc
    {
    public:
        CallbackFunc();
        ~CallbackFunc();
        void callback(boost::circular_buffer<rostate_machine::State> state_buf);
    private:
        std::vector<std::function<boost::optional<rostate_machine::Event>(void)> > funcs_;
        std::vector<CallbackInfo> callback_info_;
    };
}

#endif  //ROSTATE_MACHINE_CALLBACK_FUNC_H_INCLUDED