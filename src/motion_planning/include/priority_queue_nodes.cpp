#include "priority_queue_nodes.h"

priority_queue_nodes::priority_queue_nodes()
{
  // num_elements = 0;
}
priority_queue_nodes::~priority_queue_nodes()
{
}

int priority_queue_nodes::lookup_node(node_g* node)
{
  int start = 0;
  int end = nodes.size();
  int search_i;
  while (start != end)
  {
    if(end - start == 1)
    {
      if (nodes[start] -> cost < node -> cost)
      {
        return end;
      }
      else
      {
        return start;
      }
    }

    search_i = ( start + end ) / 2;
    if (node -> cost == nodes[search_i] -> cost)
    {
      return search_i;
    }
    else if (node -> cost < nodes[search_i] -> cost)
    {
      end = search_i;
    }
    else
    {
      start = search_i;
    }


  }
  return start;
}

bool priority_queue_nodes::nodes_in_vecinity(node_g* n1, node_g* n2)
{
  double dx = fabs(n1 -> point.x - n2 -> point.x);
  double dy = fabs(n1 -> point.y - n2 -> point.y);
  double dc = fabs(n1 -> cost    - n2 -> cost);

  return dx <= TOLERANCE_RAD && dy <= TOLERANCE_RAD && dc <= TOLERANCE_COST;
}

bool priority_queue_nodes::point_in_queue()
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  bool res = false;
  for (size_t i = 0; i < nodes.size(); i++)
  {
    for (size_t j = 0; j < nodes.size(); j++) {
      // if(i != j && nodes[i] -> point.x == nodes[j] -> point.x && nodes[i] -> point.y == nodes[j] -> point.y)
      if(i != j && nodes_in_vecinity(nodes[i], nodes[j]))
      {
        std::cout << "Found equal nodes:" << '\n';
        std::cout << "---n: cost = " << nodes[i] -> cost << " - (" << nodes[i] -> point.x << ", " << nodes[i] -> point.y << "). Parent " << nodes[i] -> parent << std::endl;
        std::cout << "---o: cost = " << nodes[j] -> cost << " - (" << nodes[j] -> point.x << ", " << nodes[j] -> point.y << "). Parent " << nodes[j] -> parent << std::endl;
        res = true;
      }
    }
  }
  return res;
}

void priority_queue_nodes::push(node_g* node)
{
  double cost = node -> cost;
  int i = lookup_node(node);

  nodes.insert(nodes.begin()+i, node);

}

bool priority_queue_nodes::empty()
{
  return size() == 0;
}

int priority_queue_nodes::size()
{
  return nodes.size();
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
  if (0 < i && i == nodes.size())
  {
    return false;
  }
  else
  {
    return nodes_in_vecinity(node, nodes[i]);
  }
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
  for (auto n : nodes)
  {
    std::cout << i <<  ": cost = " << n -> cost << " - (" << n -> point.x << ", " << n -> point.y << "). Parent " << n -> parent << std::endl;
    i++;
  }
}

void priority_queue_nodes::reset_queue()
{
  nodes.clear();
}
