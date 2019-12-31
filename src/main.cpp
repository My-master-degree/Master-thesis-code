#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/vertex.hpp"
#include "utils/util.hpp"
#include "utils/cplex/compact_model.hpp"
#include "SampleConfig.h"

#include <string>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;

int main (int argc, char **argv)
{ 

  Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_PATH + string("S4_S2_4i8s.txt"));
  unsigned int time_limit = 180;
  //cout<<gvrp_instance<<endl;
  Compact_model compact_model(gvrp_instance, time_limit, 1);
  list<list<Vertex> > routes = compact_model.run();
  Gvrp_solution gvrp_solution (routes, gvrp_instance);
  //print
  cout<<gvrp_solution;
  return 0;
}
