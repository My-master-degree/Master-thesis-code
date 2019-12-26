#include "models/gvrp_instance.hpp"
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
//  cout<<gvrp_instance<<endl;
  Compact_model compact_model(gvrp_instance);
  compact_model.run();
  //model

  return 0;
}
