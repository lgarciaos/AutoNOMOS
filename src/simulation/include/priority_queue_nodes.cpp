#include "priority_queue_nodes.h"

priority_queue_nodes::priority_queue_nodes()
{
  num_elements = 0;
}
priority_queue_nodes::~priority_queue_nodes()
{
}

int priority_queue_nodes::lookup_node(node_g* node)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  int start = 0;
  int end = nodes.size();
  int search_i;
  // std::cout << __LINE__ << '\n';
  while (start != end)
  {
    // std::cout << "start: " << start << "\tend: " << end << '\n';
    if(end - start == 1)
    {
      if (nodes[start] -> cost < node -> cost)
      {
        // std::cout << __LINE__ << '\n';
        return end;
      }
      else
      {
        // std::cout << __LINE__ << '\n';
        return start;
      }
    }

    search_i = ( start + end ) / 2;
    if (node -> cost == nodes[search_i] -> cost)
    {
      // std::cout << __LINE__ << '\n';
      return search_i;
    }
    else if (node -> cost < nodes[search_i] -> cost)
    {
      // std::cout << __LINE__ << '\n';
      end = search_i;
    }
    else
    {
      // std::cout << __LINE__ << '\n';
      start = search_i;
    }


  }
  return start;
}

bool priority_queue_nodes::point_in_queue()
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  for (size_t i = 0; i < nodes.size(); i++)
  {
    for (size_t j = 0; j < nodes.size(); j++) {
      if(i != j && nodes[i] -> point.x == nodes[j] -> point.x && nodes[i] -> point.y == nodes[j] -> point.y)
      {
        std::cout << "Found equal nodes:" << '\n';
        std::cout << "---n: cost = " << nodes[i] -> cost << " - (" << nodes[i] -> point.x << ", " << nodes[i] -> point.y << "). Parent " << nodes[i] -> parent << std::endl;
        std::cout << "---o: cost = " << nodes[j] -> cost << " - (" << nodes[j] -> point.x << ", " << nodes[j] -> point.y << "). Parent " << nodes[j] -> parent << std::endl;
      }
    }
  }
}

void priority_queue_nodes::push(node_g* node)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  double cost = node -> cost;
  // TODO: check if node is in the list ==> it shouldn be necessary...
  int i = lookup_node(node);
  // std::cout << __LINE__ << '\n';
  // std::cout << "----- start push" << '\n';
  // std::cout << "inserting: " << node -> cost << "\tat: " << i << std::endl;


  // point_in_queue();
  nodes.insert(nodes.begin()+i, node);
  // print_vector();
  // std::cout << "----- end push" << '\n';


}

bool priority_queue_nodes::empty()
{
  return size() == 0;
}

int priority_queue_nodes::size()
{
  return num_elements;
}

node_g* priority_queue_nodes::top()
{
  return nodes[0];
}

void priority_queue_nodes::pop()
{
  nodes.erase(nodes.begin());
}

bool priority_queue_nodes::node_in_queue(node_g* node)
{
  int i = lookup_node(node);
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  // node -> print_node_g(std::string("SEARCHING"));
  // std::cout << "Node Found: " << (node -> cost == nodes[i] -> cost & ARE_NODES_PTR_EQUAL(node, nodes[i])) << "\tat: " << i << std::endl;
  // print_vector();

  return node -> cost == nodes[i] -> cost & ARE_NODES_PTR_EQUAL(node, nodes[i]);

}

void priority_queue_nodes::remove_node_from_queue(node_g* node)
{
  int i = lookup_node(node);
  if (node -> cost == nodes[i] -> cost & ARE_NODES_PTR_EQUAL(node, nodes[i]))
  {
    nodes.erase(nodes.begin()+i);
  }
}

std::vector<node_g*> priority_queue_nodes::get_vector()
{
  return nodes;
}

void priority_queue_nodes::print_vector()
{
  int i = 0;
  // std::cout << "printing vector" << '\n';
  for (auto n : nodes)
  {
    std::cout << i <<  ": cost = " << n -> cost << " - (" << n -> point.x << ", " << n -> point.y << "). Parent " << n -> parent << std::endl;
    i++;
  }
  // std::cout << "ended printing vector" << '\n';
}
