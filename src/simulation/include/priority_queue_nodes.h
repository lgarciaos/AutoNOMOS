#ifndef _PRIORITY_QUEUE_NODES_
#define _PRIORITY_QUEUE_NODES_

// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
// #include <algorithm>
#include <vector>
#include <iterator>

// own libraries
// #include <node_g.cpp>


class priority_queue_nodes
{
	private:
    std::vector<node_g*> nodes;
    int num_elements;
    int lookup_node(node_g* node);

  public:

    priority_queue_nodes();
    virtual ~priority_queue_nodes();
    void push(node_g* node);
    node_g* top();
    void pop();
    bool empty();
    int size();

    std::vector<node_g*> get_vector();
    void print_vector();
    bool node_in_queue(node_g* node);
    void remove_node_from_queue(node_g* node);
    bool point_in_queue();
};

#endif
