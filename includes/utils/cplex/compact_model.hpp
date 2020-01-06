#ifndef COMPACT_MODEL_HPP_
#define COMPACT_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/cplex/subcycle_lazy_constraint_compact_model.hpp"

#include <map>
#include <list>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace models;

typedef map<int, Vertex> IDVertex;
typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;

namespace utils {
  namespace cplex {
    class Compact_model {
      public:
        explicit Compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
        pair<Gvrp_solution, Mip_solution_info> run();
        Gvrp_instance gvrp_instance;
        Gvrp_solution* gvrp_solution;
        unsigned int time_limit;//seconds
        unsigned int max_num_feasible_integer_sol;//0 to 2100000000
        bool VERBOSE;
        friend class Subcycle_lazy_constraint_compact_model; 
      private:
        IloModel model;
        IloEnv env;
        IloCplex cplex;
        Matrix3DVar x;
        IloNumVarArray e;
        Matrix3DVal x_vals;
        IDVertex all;
        int ub_edge_visit;
        void createVariables();
        void createObjectiveFunction();
        void createModel();
        void setCustomParameters();
        void fillX_vals();
        void createGvrp_solution();
    };

  } 
}
#endif
