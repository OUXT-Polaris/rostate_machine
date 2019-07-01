#ifndef ROSTATE_MACHINE_GRAPH_WRITER_H_INCLUDED
#define ROSTATE_MACHINE_GRAPH_WRITER_H_INCLUDED

#include <ros/ros.h>

using std::map;
template <typename Map>
struct NodeWriter
{
    NodeWriter(Map& g_,std::string current_state_) 
        : g (g_), current_state(current_state_) {};
    template <class Vertex>
    void operator()(std::ostream& out, Vertex v)
    {
        if(g[v].name == current_state)
        {
            out << " [color=blue label=\"" << g[v].name << "\"]" << std::endl;
        }
        else
        {
            out << " [color=black label=\"" << g[v].name << "\"]" << std::endl;
        }
    };
    Map g;
    std::string current_state;
};

template <typename Map>
struct EdgeWriter
{
    EdgeWriter(Map& g_) : g (g_) {};
    template <class Edge>
    void operator()(std::ostream& out, Edge e) 
    {
        out << " [color=black]" << std::endl;
        //out << " [label=\"" << e  <<":" << g[e].miles << "\"]" << std::endl;
    };
    Map g;
};

struct GraphWriter
{
    void operator()(std::ostream& out) const 
    {
        out << "graph [bgcolor=lightgrey]" << std::endl;
        //out << "node [shape=box color=blue]" << std::endl;

        out << "edge [color=red]" << std::endl;
    }
};

#endif  //ROSTATE_MACHINE_GRAPH_WRITER_H_INCLUDED