#include "rrt_ros.hpp"



rrt_ros_t::~rrt_ros_t(){}

std_msgs::Float64MultiArray rrt_ros_t::get_vector_path()
{
  std_msgs::Float64MultiArray res;
  if(last_solution_path.size() > 1)
  {
    // svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
    for(unsigned i = 0; i < last_solution_path.size() - 1; i++)
    {
      // traj_line<<system->visualize_point(last_solution_path[i]->point,dim);
      res.data.push_back(last_solution_path[i] -> point[0]);
      res.data.push_back(last_solution_path[i] -> point[1]);
      res.data.push_back(last_solution_path[i + 1] -> point[0]);
      res.data.push_back(last_solution_path[i + 1] -> point[1]);
    }
    // doc<<traj_line;
  }
  return res;

}

std_msgs::Float64MultiArray rrt_ros_t::get_vector_tree()
{
  create_floatmultiarray_tree(root);
  return tree_floatmultiarray;
}

void rrt_ros_t::create_floatmultiarray_tree(tree_node_t* node)
{
  // tree_node_t* node = root;
  // std_msgs::Float64MultiArray res;
  for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		// svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

    tree_floatmultiarray.data.push_back(node -> point[0]);
    tree_floatmultiarray.data.push_back(node -> point[1]);
    tree_floatmultiarray.data.push_back((*i) -> point[0]);
    tree_floatmultiarray.data.push_back((*i) -> point[1]);

		// traj_line<<system->visualize_point(node->point,dim);
		// traj_line<<system->visualize_point((*i)->point,dim);
		// doc<<traj_line;

    create_floatmultiarray_tree(*i);
		// visualize_edge(*i,doc,dim);

	}
}
