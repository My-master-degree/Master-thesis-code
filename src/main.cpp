#include <map>
#include <set>
#include <queue>
#include <ilcplex/ilocplex.h>
#include <string>
#include <list>
#include <vector>
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"
#include "utils/cplex/quadratic_model.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"
ILOSTLBEGIN

using namespace std;
using namespace models;
//using namespace utils;

int main (int argc, char **argv)
{ 

  Gvrp_instance gvrp_instance = utils::erdogan_instance_reader(PROJECT_PATH + string("S4_S2_4i8s.txt"));
  //model

  return 0;
}
