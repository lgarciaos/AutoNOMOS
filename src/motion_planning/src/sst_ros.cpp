#include "sst_ros.hpp"



sst_ros_t::~sst_ros_t(){}

// void sst_ros_t::random_sample()
// {
//   system -> random_state(sample_state);
//   switch (ctrl_to_use)
//   {
//     case RANDOM_CTRL:
//       system -> random_control(sample_control);
//     break;
//     case BANG_BANG:
//       system -> bang_bang_ctrl(sample_control);
//     break;
//   }
// }


std_msgs::Float64MultiArray sst_ros_t::get_vector_path()
{
  std_msgs::Float64MultiArray res;
  std::stringstream x_ss, y_ss;

  if(last_solution_path.size() > 1)
  {
    for(unsigned i = 0; i < last_solution_path.size() - 1; i++)
    {
      x_ss << last_solution_path[i] -> point[0] << ",";
      y_ss << last_solution_path[i] -> point[1] << ",";
      res.data.push_back(last_solution_path[i] -> point[0]);
      res.data.push_back(last_solution_path[i] -> point[1]);
      res.data.push_back(last_solution_path[i + 1] -> point[0]);
      res.data.push_back(last_solution_path[i + 1] -> point[1]);
    }
  }
  std::cout << "x: " << x_ss.str() << '\n';
  std::cout << "y: " << y_ss.str() << '\n';
  return res;

}

tree_node_t* sst_ros_t::get_root()
{
  return root;
}

void sst_ros_t::get_last_solution_path(std::vector<tree_node_t*> & last_sln)
{
  last_sln = last_solution_path;
}


std_msgs::Float64MultiArray sst_ros_t::get_vector_tree()
{
  create_floatmultiarray_tree(root);
  return tree_floatmultiarray;
}

void sst_ros_t::create_floatmultiarray_tree(tree_node_t* node)
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

void sst_ros_t::dealloc_tree()
{

  // dealloc_tree_aux(root);
  //
  // delete best_goal;
  // delete start_state;
  // delete witness_sample;
  // delete metric_query;
  // delete system;

}


void sst_ros_t::dealloc_metric()
{

}

void sst_ros_t::dealloc_tree_aux(tree_node_t* node)
{
  for (std::list<tree_node_t*>::iterator i = node->children.begin();
    i != node->children.end(); ++i)
	{
    // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    // std::cout << "node:\t" << node << '\n';
    dealloc_tree_aux(*i);
    // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    // if (i == node -> children.end())
    // {
    //   std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    // }

	}
  free(node);
}
