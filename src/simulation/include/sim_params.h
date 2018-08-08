#ifndef SIMULATION_PARAMS
#define SIMULATION_PARAMS

#include <string>

// simulation,  true);
// sim_iters,  1);
// s", gz_total_lines,  0);
// plot_lines,  false);
// obstacles_radius,  0);
namespace sim_params
{
	extern bool simulation;
	extern int sim_iters;
	extern int gz_total_lines;
  extern bool plot_lines;
	extern bool publish_ctrl_path;
}



#endif
