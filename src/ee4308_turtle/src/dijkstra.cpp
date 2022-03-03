#include "dijkstra.hpp"


Dijkstra::Node::Node() 
    : g(0), idx(-1, -1), parent(-1, -1) 
    {}
Dijkstra::Open::Open() 
    : f(0), idx(-1, -1) 
    {}
Dijkstra::Open::Open(double f, Index idx) 
    : f(f), idx(idx) 
    {}
Dijkstra::Dijkstra(Grid & planner_grid, Grid grid) // assumes the size of the grid is always the same
    : start(-1, -1), goal(-1, -1), planner_grid(planner_grid), nodes(grid.size.i * grid.size.j), open_list(), grid(grid)
    {
        // write the nodes' indices
        int k = 0;
        for (int i = 0; i < grid.size.i; ++i)
        {
            for (int j = 0; j < grid.size.j; ++j)
            {
                nodes[k].idx.i = i;
                nodes[k].idx.j = j;
                ++k;
            }
        }
    }
    

void Dijkstra::add_to_open(Node * node)
{   // sort node into the open list
    double node_f = node->g;

    for (int n = 0; n < open_list.size(); ++n)
    {
        Open & open_node = open_list[n];
        if (open_node.f > node_f + 1e-5)
        {   // the current node in open is guaranteed to be more expensive than the node to be inserted ==> insert at current location            
            open_list.emplace(open_list.begin() + n, node_f, node->idx);

            // emplace is equivalent to the below but more efficient:
            // Open new_open_node = Open(node_f, node->idx);
            // open_list.insert(open_list.begin() + n, new_open_node);
            return;
        }
    }
    // at this point, either open_list is empty or node_f is more expensive that all open nodes in the open list
    open_list.emplace_back(node_f, node->idx);
}
Dijkstra::Node * Dijkstra::poll_from_open()
{   
    Index & idx = open_list.front().idx; //ref is faster than copy
    int k = grid.get_key(idx);
    Node * node = &(nodes[k]);

    open_list.pop_front();

    return node;
}


Index Dijkstra::get(Index idx)
{
    std::vector<Index> path_idx; // clear previous path
    // initialise data for all nodes
    for (Node & node : nodes)
    {
        node.g = 1e5; // a reasonably large number. You can use infinity in clims as well, but clims is not included
    }

    // set start node g cost as zero
    int k = grid.get_key(idx);
    Node * node = &(nodes[k]);
    node->g = 0;

    // add start node to openlist
    add_to_open(node);

    // main loop
    while (!open_list.empty())
    {
        // (1) poll node from open
        node = poll_from_open();

        // (3) return path if node is the goal
        if (planner_grid.get_cell(node->idx))
        {   // reached the goal, return the path
            ROS_INFO("free cell found!");
            open_list.clear();
            return node->idx;

        }

        // (4) check neighbors and add them if cheaper
        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   // for each neighbor in the 8 directions
            is_cardinal = (dir & 1 == 0);
            // get their index
            Index & idx_nb_relative = NB_LUT[dir];
            Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            // get the cost if accessing from node as parent
            double g_nb = node->g;
            if (is_cardinal) 
                g_nb += 1;
            else
                g_nb += M_SQRT2;
            // the above if else can be condensed using ternary statements: g_nb += is_cardinal ? 1 : M_SQRT2;

            int nb_k = grid.get_key(idx_nb);            
            if (planner_grid.grid_inflation[nb_k] > 0) // is inflated
                g_nb += 10;
            else if (planner_grid.grid_log_odds[nb_k] > grid.log_odds_thresh) //is obstacle
               g_nb += 900;
            // else
            //     continue; // in map, not inflated, and is log odds free

            // compare the cost to any previous costs. If cheaper, mark the node as the parent
            Node & nb_node = nodes[nb_k]; // use reference so changing nb_node changes nodes[k]
            if (nb_node.g > g_nb + 1e-5)
            {   // previous cost was more expensive, rewrite with current
                nb_node.g = g_nb;
                nb_node.parent = node->idx;

                // add to open
                add_to_open(&nb_node); // & a reference means getting the pointer (address) to the reference's object.
            }

            // toggle is_cardinal
            //is_cardinal = !is_cardinal;
        }
    }

    // clear open list
    open_list.clear();
    return idx; // is empty if open list is empty
}

