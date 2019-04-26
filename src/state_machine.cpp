#include <rostate_machine/state_machine.h>

StateMachine::StateMachine(std::string xml_filepath)
{
    using namespace boost::property_tree;
    ptree pt;
    read_xml(xml_filepath, pt);
    std::string init_state_name;
    for (const ptree::value_type& state_itr : pt.get_child("state_machine"))
    {
        if(state_itr.first == "init_state")
        {
            init_state_name = state_itr.second.get<std::string>("<xmlattr>.name");
        }
    }
    for (const ptree::value_type& state_itr : pt.get_child("state_machine"))
    {
        if(state_itr.first == "transition")
        {
            std::string from_state_name = state_itr.second.get<std::string>("<xmlattr>.from");
            std::string to_state_name = state_itr.second.get<std::string>("<xmlattr>.to");
            std::string trigger_event_name = state_itr.second.get<std::string>("<xmlattr>.name");
            addTransition(from_state_name, to_state_name, trigger_event_name);
        }
    }
    setCurrentState(init_state_name);
}

StateMachine::~StateMachine()
{
}

bool StateMachine::setCurrentState(std::string current_state)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto vertex_range = boost::vertices(state_graph_);
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
    {
        vertex_t v = *first;
        if(state_graph_[v].name == current_state)
        {
            current_state_ = v;
            return true;
        }
    }
    return false;
}

void StateMachine::addTransition(std::string from_state_name, std::string to_state_name, std::string trigger_event_name)
{
    std::lock_guard<std::mutex> lock(mtx_);
    vertex_t from_state;
    vertex_t to_state;
    edge_t transition;
    auto vertex_range = boost::vertices(state_graph_);
    if(from_state_name != to_state_name)
    {
        bool from_state_found = false;
        bool to_state_found = false;
        for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
        {
            vertex_t v = *first;
            if(state_graph_[v].name == from_state_name)
            {
                from_state_found = true;
                from_state = v;
            }
            if(state_graph_[v].name == to_state_name)
            {
                to_state_found = true;
                to_state = v;
            }
        }
        if(!from_state_found)
        {
            vertex_t v = boost::add_vertex(state_graph_);
            state_graph_[v].name = from_state_name;
            from_state = v;
        }
        if(!to_state_found)
        {
            vertex_t v = boost::add_vertex(state_graph_);
            state_graph_[v].name = to_state_name;
            to_state = v;
        }
        bool inserted = false;
        boost::tie(transition, inserted) = boost::add_edge(from_state, to_state, state_graph_);
        state_graph_[transition].trigger_event = trigger_event_name;
        state_graph_[transition].from_state = from_state_name;
        state_graph_[transition].to_state = to_state_name;
    }
    else
    {
        bool state_found = false;
        for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
        {
            vertex_t v = *first;
            if(state_graph_[v].name == from_state_name)
            {
                state_found = true;
                from_state = v;
                to_state = v;
            }
        }
        if(!state_found)
        {
            vertex_t v = boost::add_vertex(state_graph_);
            state_graph_[v].name = from_state_name;
            from_state = v;
            to_state = v;
        }
        bool inserted = false;
        boost::tie(transition, inserted) = boost::add_edge(from_state, to_state, state_graph_);
        state_graph_[transition].trigger_event = trigger_event_name;
        state_graph_[transition].from_state = from_state_name;
        state_graph_[transition].to_state = to_state_name;
    }
    return;
}

std::vector<std::string> StateMachine::getPossibeTransitionStates()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::string> ret;
    adjacency_iterator_t vi;
    adjacency_iterator_t vi_end;
    for (boost::tie(vi, vi_end) = adjacent_vertices(current_state_, state_graph_); vi != vi_end; ++vi)
    {
        ret.push_back(state_graph_[*vi].name);
    }
    return ret;
}

std::vector<std::string> StateMachine::getPossibeTransitions()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::string> ret;
    out_edge_iterator_t ei;
    out_edge_iterator_t ei_end;
    for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei)
    {
        ret.push_back(state_graph_[*ei].trigger_event);
    }
    return ret;
}

bool StateMachine::tryTransition(std::string trigger_event_name)
{
    std::lock_guard<std::mutex> lock(mtx_);
    out_edge_iterator_t ei;
    out_edge_iterator_t ei_end;
    for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei)
    {
        if(trigger_event_name == state_graph_[*ei].trigger_event)
        {
            auto vertex_range = boost::vertices(state_graph_);
            for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
            {
                vertex_t v = *first;
                if(state_graph_[v].name == state_graph_[*ei].to_state)
                {
                    current_state_ = v;
                    return true;
                }
            }
            return false;
        }
    }
    return false;
}

StateInfo StateMachine::getStateInfo()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::string current_state = state_graph_[current_state_].name;
    std::vector<std::string> possible_transitions;
    out_edge_iterator_t ei;
    out_edge_iterator_t ei_end;
    for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei)
    {
        possible_transitions.push_back(state_graph_[*ei].trigger_event);
    }
    std::vector<std::string> possible_transition_states;
    adjacency_iterator_t vi;
    adjacency_iterator_t vi_end;
    for (boost::tie(vi, vi_end) = adjacent_vertices(current_state_, state_graph_); vi != vi_end; ++vi)
    {
        possible_transition_states.push_back(state_graph_[*vi].name);
    }
    StateInfo ret(current_state, possible_transition_states, possible_transitions);
    return ret;
}

std::string StateMachine::getCurrentState()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return state_graph_[current_state_].name;
}

std::string StateMachine::getDotString()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::string dot_string;
    std::stringstream sstream(dot_string);
    boost::write_graphviz(sstream, state_graph_, boost::make_label_writer(get(&StateProperty::name, state_graph_)),
        boost::make_label_writer(get(&TransitionProperty::trigger_event, state_graph_)));
    return dot_string;
}

void StateMachine::drawStateMachine(std::string dot_filename)
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::ofstream f(dot_filename.c_str());
    boost::write_graphviz(f, state_graph_, boost::make_label_writer(get(&StateProperty::name, state_graph_)),
        boost::make_label_writer(get(&TransitionProperty::trigger_event, state_graph_)));
    return;
}