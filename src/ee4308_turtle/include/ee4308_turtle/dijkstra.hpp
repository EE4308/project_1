#include "ros/ros.h"
#include "grid.hpp"
#include "common.hpp"
#include <vector>
#include <deque>

#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP
class Dijkstra 
{       
    public:
        struct Node
        {
            double g;
            Index idx, parent;
            Node();
        };
        struct Open
        {
            double f;
            Index idx;
            Open();
            Open(double f, Index idx);
        };
        Index start, goal;
        Grid & planner_grid; // REFERENCE <-- you cannot put the Planner class into containers (vectors , arrays etc.) 
        Grid grid;

        Dijkstra(Grid & planner_grid, Grid grid);
        Index find_closest_free(Index idx_start);
        Index get(Index idx);
        std::deque<Open> open_list;
    private:
        std::vector<Node> nodes; // keeps a record of the cheapest cost of every cell in the grid, as well as their parents
        
        Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

        void add_to_open(Node * node);
        Node * poll_from_open();
};
#endif