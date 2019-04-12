#ifndef STATE_MACHINE_H_INCLUDED
#define STATE_MACHINE_H_INCLUDED

//headers in boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/graph/graphviz.hpp>

//headers in STL
#include <mutex>

struct transition_property
{
    std::string trigger_event;
    std::string from_state;
    std::string to_state;
};

struct state_property
{
    std::string name;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, state_property, transition_property> graph_t;
typedef graph_t::vertex_descriptor vertex_t;
typedef graph_t::edge_descriptor edge_t;
typedef boost::graph_traits<graph_t>::adjacency_iterator adjacency_iterator_t;
typedef boost::graph_traits<graph_t>::out_edge_iterator out_edge_iterator_t;

struct state_info_t
{
    const std::vector<std::string> possibe_transition_states;
    const std::vector<std::string> possibe_transitions;
    const std::string current_state;
    state_info_t(std::string current_state_,
        std::vector<std::string> possibe_transition_states_,
        std::vector<std::string> possibe_transitions_)
            : current_state(current_state_),
                possibe_transition_states(possibe_transition_states_),
                possibe_transitions(possibe_transitions_)
            {

            }
};

class state_machine
{
public:
    state_machine(std::string xml_filepath);
    ~state_machine();
    bool try_transition(std::string trigger_event_name);
    bool set_current_state(std::string current_state);
    std::vector<std::string> get_possibe_transition_states();
    std::vector<std::string> get_possibe_transitions();
    std::string get_current_state();
    state_info_t get_state_info();
    void draw_state_machine(std::string dot_filename);
    std::string get_dot_string();
private:
    void add_transition_(std::string from_state_name, std::string to_state_name, std::string trigger_event_name);
    std::mutex mtx_;
    graph_t state_graph_;
    vertex_t current_state_;
};
#endif //STATE_MACHINE_H_INCLUDED